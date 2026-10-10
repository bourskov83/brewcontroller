#!/usr/bin/env python3
"""
sensor_config.py

Standalone commissioning tool for the ESP32 DS18B20 gateway.

Pushes the slot -> ROM mapping from sensors.yaml into the ESP32's
holding-register config area, tells it to commit to NVS (Preferences)
through the slot command register, and then reads back the live bus
discovery table so you can confirm every configured sensor is actually
present on the wire.

Calibration offsets are not sent to the firmware: the softPLC applies
them to the raw readings and the ESP32 stays raw I/O.

Lives next to server.py / sensors.yaml and reuses the same building
blocks the softPLC uses (ModbusRTUClient, sensor_map, rom helpers) so
there is exactly one definition of the ROM<->register encoding, shared
with plc/sensors.py.

Register map (must match esp32_firmware/src/config.h):
    100-139   ROM_CONFIG (10 slots x 4 regs, 64-bit ROM each)   [write, then APPLY_MAPPING]
    200       SLOT_CMD         PLC writes a command code, firmware clears it on accept
    201       SLOT_CMD_STATUS  firmware writes: 0 idle, 1 busy, 2 done, 3 error
    300       PRESENT_COUNT    (read after a SCAN command has reported DONE)
    301..     PRESENT_ROMS     (read after a SCAN command has reported DONE, 4 regs per sensor)

Command codes written to SLOT_CMD:
    1         SCAN             rescan the 1-Wire bus and refresh the discovery table
    0xA5A5    APPLY_MAPPING    commit the staged ROM table to NVS and rebind
    0xDEAD    CLEAR_CONFIG     wipe the stored slot mapping

    210       LOG_LEVEL_ALL    R/W, writing sets every tag (0 none .. 5 verbose)
    211..     LOG_LEVEL per tag, R/W, 211 + index in LOG_TAGS (not persisted)

Command handshake (see send_slot_command):
    1. write command code to SLOT_CMD
    2. poll SLOT_CMD_STATUS until DONE or ERROR
    3. write 0 to SLOT_CMD to acknowledge (status returns to IDLE)

Usage:
    python sensor_config.py discover --port /dev/tty.usbserial-XXXX
    python sensor_config.py apply --port /dev/tty.usbserial-XXXX
    python sensor_config.py apply --port /dev/tty.usbserial-XXXX --dry-run
    python sensor_config.py clear --port /dev/tty.usbserial-XXXX --yes
    python sensor_config.py loglevel --port /dev/tty.usbserial-XXXX --show
    python sensor_config.py loglevel --port /dev/tty.usbserial-XXXX --level info
    python sensor_config.py loglevel --port /dev/tty.usbserial-XXXX --tag DS18B20Manager --level debug
"""
import argparse
import sys
import time

from logger_config import log
from modbus_rtu_client import ModbusRTUClient, ModbusRTUError
from plc.sensor_map import load_sensor_map, SensorMapping
from plc.helpers import encode_rom, decode_sensor_rom
from plc.sensors import (
    ROM_CONFIG_WRITE_ADDR,   # 100
    SENSOR_COUNT_ADDR,       # 300 (present/discovered count)
    ROM_TABLE_ADDR,          # 301 (present ROMs)
)

# --- Firmware interface (mirror of esp32_firmware/src/config.h) -------------
DS_MAX_SLOTS = 10                # DS_MAX_SLOTS on the firmware side
ROM_REG_STRIDE = 4               # 4 x 16-bit regs per 64-bit ROM
MAX_DISC = 16                    # DS18B20Manager::MAX_DISC on the firmware side

REG_SLOT_CMD = 200
REG_SLOT_CMD_STATUS = 201

SLOT_CMD_IDLE = 0
SLOT_CMD_SCAN = 1
SLOT_CMD_APPLY_MAPPING = 0xA5A5
SLOT_CMD_CLEAR_CONFIG = 0xC1EA

SLOT_STATUS_IDLE = 0
SLOT_STATUS_BUSY = 1
SLOT_STATUS_DONE = 2
SLOT_STATUS_ERROR = 3

CMD_TIMEOUT_S = 5.0
SCAN_TIMEOUT_S = 10.0            # the 1-Wire search runs in the DS18B20 task, allow it more time
CMD_POLL_S = 0.05

DS18B20_FAMILY_CODE = 0x28

# Log level registers (R/W, applied immediately by the firmware, not persisted)
REG_LOG_LEVEL_ALL = 210          # writing sets every tag
REG_LOG_LEVEL_BASE = 211         # 211 + index into LOG_TAGS
LOG_LEVEL_MAX = 5
LOG_LEVELS = {"none": 0, "error": 1, "warn": 2, "info": 3, "debug": 4, "verbose": 5}
# Order must match LOG_TAGS in esp32_firmware/src/config.h (register index = position)
LOG_TAGS = [
    "MAIN",
    "CFGSTR",
    "SLOTCFG",
    "DS18B20",
    "GPIO",
    "MODBUS",
]


class SlotCommandError(Exception):
    """The firmware rejected a slot command, was busy, or never answered."""


def rom_str_to_bytes(rom_str: str) -> bytes:
    return bytes.fromhex(rom_str.replace('-', '').replace(':', ''))


def send_slot_command(rtu: ModbusRTUClient, cmd: int, name: str,
                      timeout: float = CMD_TIMEOUT_S) -> None:
    """Run one command through the SLOT_CMD / SLOT_CMD_STATUS handshake.

    Raises SlotCommandError if the firmware is busy, reports ERROR, or does
    not finish within `timeout` seconds.
    """
    # Clear any stale DONE/ERROR left over from an earlier run (0 = acknowledge)
    status = rtu.read_hregs(REG_SLOT_CMD_STATUS, 1)[0]
    if status == SLOT_STATUS_BUSY:
        raise SlotCommandError(f"{name}: firmware is busy with another command")
    if status != SLOT_STATUS_IDLE:
        rtu.write_hregs(REG_SLOT_CMD, [SLOT_CMD_IDLE])

    rtu.write_hregs(REG_SLOT_CMD, [cmd])

    deadline = time.monotonic() + timeout
    while True:
        status = rtu.read_hregs(REG_SLOT_CMD_STATUS, 1)[0]
        if status in (SLOT_STATUS_DONE, SLOT_STATUS_ERROR):
            break
        if time.monotonic() > deadline:
            raise SlotCommandError(
                f"{name}: timed out after {timeout:.1f}s (status={status})"
            )
        time.sleep(CMD_POLL_S)

    # Acknowledge: status goes back to IDLE
    rtu.write_hregs(REG_SLOT_CMD, [SLOT_CMD_IDLE])

    if status == SLOT_STATUS_ERROR:
        raise SlotCommandError(f"{name}: rejected by firmware (status=ERROR)")
    log.info(f"{name}: done")


def validate_sensor_map(sensors: list[SensorMapping]) -> None:
    slots_seen: dict[int, str] = {}
    roms_seen: dict[str, str] = {}
    for m in sensors:
        if not (0 <= m.slot < DS_MAX_SLOTS):
            raise ValueError(
                f"Sensor '{m.sensor}': slot {m.slot} out of range "
                f"(firmware supports 0..{DS_MAX_SLOTS - 1})"
            )
        if m.slot in slots_seen:
            raise ValueError(
                f"Duplicate slot {m.slot}: used by both "
                f"'{slots_seen[m.slot]}' and '{m.sensor}'"
            )
        slots_seen[m.slot] = m.sensor

        rom_bytes = rom_str_to_bytes(m.rom)
        if len(rom_bytes) != 8:
            raise ValueError(f"Sensor '{m.sensor}': ROM '{m.rom}' is not 8 bytes")
        if rom_bytes[0] != DS18B20_FAMILY_CODE:
            log.warning(
                f"Sensor '{m.sensor}': ROM '{m.rom}' has family code "
                f"0x{rom_bytes[0]:02X}, expected 0x{DS18B20_FAMILY_CODE:02X} "
                f"(DS18B20) - double-check this ROM"
            )
        rom_norm = m.rom.upper()
        if rom_norm in roms_seen:
            raise ValueError(
                f"Duplicate ROM '{m.rom}': used by both "
                f"'{roms_seen[rom_norm]}' and '{m.sensor}'"
            )
        roms_seen[rom_norm] = m.sensor


def print_mapping_table(sensors: list[SensorMapping]) -> None:
    print(f"\n{'slot':<5}{'sensor':<16}{'rom':<26}{'roc':>6}")
    print("-" * 53)
    for m in sensors:
        print(f"{m.slot:<5}{m.sensor:<16}{m.rom:<26}{str(m.roc_enabled):>6}")
    unmapped = sorted(set(range(DS_MAX_SLOTS)) - {m.slot for m in sensors})
    if unmapped:
        print(f"\nSlots left empty (ROM will be zeroed): {unmapped}")
    print()


def build_rom_registers(sensors: list[SensorMapping]) -> list[int]:
    """Build the full-width, zero-padded ROM register block for all DS_MAX_SLOTS."""
    rom_regs = [0] * (DS_MAX_SLOTS * ROM_REG_STRIDE)
    for m in sensors:
        base = m.slot * ROM_REG_STRIDE
        rom_regs[base:base + ROM_REG_STRIDE] = encode_rom(m.rom)
    return rom_regs


def read_present_roms(rtu: ModbusRTUClient) -> list[str]:
    count = rtu.read_hregs(SENSOR_COUNT_ADDR, 1)[0]
    if count == 0:
        return []
    count = min(count, MAX_DISC)
    regs = rtu.read_hregs(ROM_TABLE_ADDR, count * ROM_REG_STRIDE)
    roms = []
    for i in range(0, len(regs), ROM_REG_STRIDE):
        rom_bytes = decode_sensor_rom(regs[i:i + ROM_REG_STRIDE])
        roms.append(rom_bytes.hex('-').upper())
    return roms


def scan_bus(rtu: ModbusRTUClient) -> list[str]:
    """Ask the firmware to rescan the 1-Wire bus, then read the discovery table.

    The table is only read after the firmware reports DONE, so it is never
    read while the DS18B20 task is still filling it.
    """
    send_slot_command(rtu, SLOT_CMD_SCAN, "SCAN", timeout=SCAN_TIMEOUT_S)
    return read_present_roms(rtu)


def connect(port: str, baud: int, timeout: float) -> ModbusRTUClient:
    rtu = ModbusRTUClient(port=port, baudrate=baud, parity="N", stopbits=1,
                           bytesize=8, timeout=timeout)
    rtu.connect()
    log.info(f"Connected to {port} @ {baud} baud")
    return rtu


def cmd_discover(args: argparse.Namespace) -> int:
    rtu = connect(args.port, args.baud, args.timeout)
    try:
        roms = scan_bus(rtu)
        if not roms:
            print("No sensors detected on the 1-Wire bus.")
        else:
            print(f"\n{len(roms)} sensor(s) detected on the bus:\n")
            for i, rom in enumerate(roms):
                print(f"  [{i}] {rom}")
            print("\nCopy the ROM string(s) you need into sensors.yaml.\n")
        return 0
    finally:
        rtu.close()


def cmd_clear(args: argparse.Namespace) -> int:
    if not args.yes:
        resp = input("This will WIPE all sensor ROM mappings in ESP32 NVS. Type 'yes' to continue: ")
        if resp.strip().lower() != "yes":
            print("Aborted.")
            return 1
    rtu = connect(args.port, args.baud, args.timeout)
    try:
        send_slot_command(rtu, SLOT_CMD_CLEAR_CONFIG, "CLEAR_CONFIG")
        log.info("NVS wiped, all slots reset")
        return 0
    finally:
        rtu.close()


def cmd_apply(args: argparse.Namespace) -> int:
    sensors = load_sensor_map(args.sensors)
    validate_sensor_map(sensors)
    print_mapping_table(sensors)

    rom_regs = build_rom_registers(sensors)

    if args.dry_run:
        print(f"[dry-run] Would write {len(rom_regs)} ROM regs at "
              f"HREG[{ROM_CONFIG_WRITE_ADDR}..{ROM_CONFIG_WRITE_ADDR + len(rom_regs) - 1}]")
        print(f"[dry-run] Would send APPLY_MAPPING (0x{SLOT_CMD_APPLY_MAPPING:04X}) "
              f"to HREG[{REG_SLOT_CMD}] and wait on status at HREG[{REG_SLOT_CMD_STATUS}]")
        print("[dry-run] No connection opened, nothing written.")
        return 0

    if not args.yes:
        resp = input(f"About to write this mapping to {args.port} and commit to NVS. "
                      f"Type 'yes' to continue: ")
        if resp.strip().lower() != "yes":
            print("Aborted.")
            return 1

    rtu = connect(args.port, args.baud, args.timeout)
    try:
        if args.clear_first:
            send_slot_command(rtu, SLOT_CMD_CLEAR_CONFIG, "CLEAR_CONFIG")
            log.info("Cleared existing NVS config before applying new mapping")

        rtu.write_hregs(ROM_CONFIG_WRITE_ADDR, rom_regs)
        log.info(f"Wrote ROM table ({len(rom_regs)} regs @ {ROM_CONFIG_WRITE_ADDR})")

        send_slot_command(rtu, SLOT_CMD_APPLY_MAPPING, "APPLY_MAPPING")
        log.info("Mapping committed to NVS and rebound")

        if args.no_verify:
            return 0

        # Fresh bus scan, so verification no longer depends on a fixed delay
        present = set(scan_bus(rtu))
        expected = {m.rom.upper(): m.sensor for m in sensors}

        print("\nVerification against live bus scan:")
        ok = True
        for rom, name in expected.items():
            if rom in present:
                print(f"  OK      {name:<16} {rom}")
            else:
                ok = False
                print(f"  MISSING {name:<16} {rom}  (not seen on bus - check wiring)")
        extra = present - set(expected.keys())
        for rom in extra:
            print(f"  UNMAPPED  {'-':<16} {rom}  (present on bus, not in {args.sensors})")

        if not ok:
            log.warning("One or more configured sensors were not detected on the bus")
            return 2
        return 0
    except (ModbusRTUError, SlotCommandError) as e:
        log.error(f"Commissioning failure: {e}")
        return 1
    finally:
        rtu.close()


def parse_level(text: str) -> int:
    t = text.strip().lower()
    if t in LOG_LEVELS:
        return LOG_LEVELS[t]
    if t.isdigit() and int(t) <= LOG_LEVEL_MAX:
        return int(t)
    raise argparse.ArgumentTypeError(
        f"invalid level '{text}' (use 0-{LOG_LEVEL_MAX} or one of: {', '.join(LOG_LEVELS)})"
    )


def cmd_loglevel(args: argparse.Namespace) -> int:
    if not args.show and args.level is None:
        log.error("--level is required unless --show is used")
        return 1

    rtu = connect(args.port, args.baud, args.timeout)
    try:
        if args.show:
            regs = rtu.read_hregs(REG_LOG_LEVEL_ALL, 1 + len(LOG_TAGS))
            names = {v: k for k, v in LOG_LEVELS.items()}
            print(f"\n{'tag':<20}{'level':>8}")
            print("-" * 28)
            print(f"{'(last set-all)':<20}{names.get(regs[0], regs[0]):>8}")
            for tag, val in zip(LOG_TAGS, regs[1:]):
                print(f"{tag:<20}{names.get(val, val):>8}")
            print()
            return 0

        if args.tag:
            addr = REG_LOG_LEVEL_BASE + LOG_TAGS.index(args.tag)
            target = args.tag
        else:
            addr = REG_LOG_LEVEL_ALL
            target = "ALL tags"
        rtu.write_hregs(addr, [args.level])
        log.info(f"Log level for {target} set to {args.level}")
        return 0
    finally:
        rtu.close()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Apply / inspect DS18B20 sensor config on the ESP32 gateway")
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--port", required=True, help="Serial port, e.g. /dev/cu.usbserial-A50285BI")
    common.add_argument("--baud", type=int, default=38400, help="Must match MODBUS_BAUD in config.h (default 38400)")
    common.add_argument("--timeout", type=float, default=2.0, help="Serial timeout in seconds")

    sub = p.add_subparsers(dest="command", required=True)

    d = sub.add_parser("discover", parents=[common], help="List ROMs currently present on the 1-Wire bus")
    d.set_defaults(func=cmd_discover)

    a = sub.add_parser("apply", parents=[common], help="Push sensors.yaml slot->ROM mapping to the ESP32 and commit to NVS")
    a.add_argument("--sensors", default="sensors.yaml", help="Path to sensors.yaml (default: ./sensors.yaml)")
    a.add_argument("--dry-run", action="store_true", help="Validate and show planned writes without opening the port")
    a.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    a.add_argument("--clear-first", action="store_true", help="Wipe NVS before applying (recommended if slot count changed)")
    a.add_argument("--no-verify", action="store_true", help="Skip the post-apply bus-scan verification")
    a.set_defaults(func=cmd_apply)

    c = sub.add_parser("clear", parents=[common], help="Wipe all sensor ROM mappings from ESP32 NVS")
    c.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    c.set_defaults(func=cmd_clear)

    ll = sub.add_parser("loglevel", parents=[common], help="Show or set the ESP32 runtime log level (all tags or one tag)")
    ll.add_argument("--tag", choices=LOG_TAGS, help="Only change this tag (default: all tags)")
    ll.add_argument("--level", type=parse_level, help=f"0-{LOG_LEVEL_MAX} or: {', '.join(LOG_LEVELS)}")
    ll.add_argument("--show", action="store_true", help="Read back the current level of every tag")
    ll.set_defaults(func=cmd_loglevel)

    return p


def main() -> int:
    args = build_parser().parse_args()
    try:
        return args.func(args)
    except (ModbusRTUError, SlotCommandError) as e:
        log.error(f"Modbus I/O failure: {e}")
        return 1
    except (ValueError, FileNotFoundError) as e:
        log.error(str(e))
        return 1


if __name__ == "__main__":
    sys.exit(main())
