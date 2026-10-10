#!/usr/bin/env python3
"""
sensor_config.py

Standalone commissioning tool for the ESP32 DS18B20 gateway.

Pushes the slot -> ROM mapping and per-slot calibration offsets from
sensors.yaml into the ESP32's holding-register config area, tells it to
commit them to NVS (Preferences), and then reads back the live bus
discovery table so you can confirm every configured sensor is actually
present on the wire.

Lives next to server.py / sensors.yaml and reuses the same building
blocks the softPLC uses (ModbusRTUClient, sensor_map, rom/scaling
helpers) so there is exactly one definition of the ROM<->register
encoding, shared with plc/sensors.py.

Register map (must match esp32_firmware/src/config.h):
    100-139   ROM_CONFIG (10 slots x 4 regs, 64-bit ROM each)   [write, then APPLY]
    50-59     TEMPS_OFFSET (10 slots x 1 reg, int16 degC x100)  [write, then APPLY]
    200       APPLY_MAPPING  (write 0xA5A5 to commit HR -> NVS and rebind)
    201       CLEAR_CONFIG   (write 0xDEAD to wipe NVS)
    300       PRESENT_COUNT  (read, live bus scan)
    301..     PRESENT_ROMS   (read, 4 regs per discovered sensor)

Usage:
    python sensor_config.py discover --port /dev/tty.usbserial-XXXX
    python sensor_config.py apply --port /dev/tty.usbserial-XXXX
    python sensor_config.py apply --port /dev/tty.usbserial-XXXX --dry-run
    python sensor_config.py clear --port /dev/tty.usbserial-XXXX --yes
"""
import argparse
import sys
import time

from logger_config import log
from modbus_rtu_client import ModbusRTUClient, ModbusRTUError
from plc.sensor_map import load_sensor_map, SensorMapping
from plc.helpers import encode_rom, decode_sensor_rom
from plc.scaling import float_to_reg
from plc.sensors import (
    ROM_CONFIG_WRITE_ADDR,   # 100
    OFFSET_BASE_ADDR,        # 50
    OFFSET_COUNT,            # 10  (== DS_MAX_SLOTS)
    APPLY_CONFIG_ADDR,       # 200
    SENSOR_COUNT_ADDR,       # 300 (present/discovered count)
    ROM_TABLE_ADDR,          # 301 (present ROMs)
)

DS_MAX_SLOTS = OFFSET_COUNT
ROM_REG_STRIDE = 4               # 4 x 16-bit regs per 64-bit ROM
APPLY_CONFIG_VALUE = 0xA5A5
CLEAR_CONFIG_ADDR = 201
CLEAR_CONFIG_VALUE = 0xDEAD
DS18B20_FAMILY_CODE = 0x28
MAX_DISC = 16                    # DS18B20Manager::MAX_DISC on the firmware side


def to_u16(v: int) -> int:
    """Pack a signed 16-bit value into the unsigned 0-65535 range Modbus expects."""
    return v & 0xFFFF


def rom_str_to_bytes(rom_str: str) -> bytes:
    return bytes.fromhex(rom_str.replace('-', '').replace(':', ''))


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
    print(f"\n{'slot':<5}{'sensor':<16}{'rom':<26}{'offset':>8}{'roc':>6}")
    print("-" * 61)
    for m in sensors:
        print(f"{m.slot:<5}{m.sensor:<16}{m.rom:<26}{m.offset:>8.2f}{str(m.roc_enabled):>6}")
    unmapped = sorted(set(range(DS_MAX_SLOTS)) - {m.slot for m in sensors})
    if unmapped:
        print(f"\nSlots left empty (ROM/offset will be zeroed): {unmapped}")
    print()


def build_registers(sensors: list[SensorMapping]) -> tuple[list[int], list[int]]:
    """Build full-width, zero-padded ROM and offset register blocks for all DS_MAX_SLOTS."""
    rom_regs = [0] * (DS_MAX_SLOTS * ROM_REG_STRIDE)
    offset_regs = [0] * DS_MAX_SLOTS

    for m in sensors:
        base = m.slot * ROM_REG_STRIDE
        rom_regs[base:base + ROM_REG_STRIDE] = encode_rom(m.rom)
        offset_regs[m.slot] = to_u16(float_to_reg(m.offset)[0])

    return rom_regs, offset_regs


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


def connect(port: str, baud: int, timeout: float) -> ModbusRTUClient:
    rtu = ModbusRTUClient(port=port, baudrate=baud, parity="N", stopbits=1,
                           bytesize=8, timeout=timeout)
    rtu.connect()
    log.info(f"Connected to {port} @ {baud} baud")
    return rtu


def cmd_discover(args: argparse.Namespace) -> int:
    rtu = connect(args.port, args.baud, args.timeout)
    try:
        roms = read_present_roms(rtu)
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
        resp = input("This will WIPE all sensor ROMs/offsets in ESP32 NVS. Type 'yes' to continue: ")
        if resp.strip().lower() != "yes":
            print("Aborted.")
            return 1
    rtu = connect(args.port, args.baud, args.timeout)
    try:
        rtu.write_hregs(CLEAR_CONFIG_ADDR, [CLEAR_CONFIG_VALUE])
        log.info("Sent CLEAR_CONFIG - NVS wiped, all slots reset")
        return 0
    finally:
        rtu.close()


def cmd_apply(args: argparse.Namespace) -> int:
    sensors = load_sensor_map(args.sensors)
    validate_sensor_map(sensors)
    print_mapping_table(sensors)

    rom_regs, offset_regs = build_registers(sensors)

    if args.dry_run:
        print(f"[dry-run] Would write {len(rom_regs)} ROM regs at "
              f"HREG[{ROM_CONFIG_WRITE_ADDR}..{ROM_CONFIG_WRITE_ADDR + len(rom_regs) - 1}]")
        print(f"[dry-run] Would write {len(offset_regs)} offset regs at "
              f"HREG[{OFFSET_BASE_ADDR}..{OFFSET_BASE_ADDR + len(offset_regs) - 1}]")
        print(f"[dry-run] Would write APPLY_MAPPING=0x{APPLY_CONFIG_VALUE:04X} at HREG[{APPLY_CONFIG_ADDR}]")
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
            rtu.write_hregs(CLEAR_CONFIG_ADDR, [CLEAR_CONFIG_VALUE])
            log.info("Cleared existing NVS config before applying new mapping")
            time.sleep(0.2)

        rtu.write_hregs(ROM_CONFIG_WRITE_ADDR, rom_regs)
        log.info(f"Wrote ROM table ({len(rom_regs)} regs @ {ROM_CONFIG_WRITE_ADDR})")

        rtu.write_hregs(OFFSET_BASE_ADDR, offset_regs)
        log.info(f"Wrote offset table ({len(offset_regs)} regs @ {OFFSET_BASE_ADDR})")

        rtu.write_hregs(APPLY_CONFIG_ADDR, [APPLY_CONFIG_VALUE])
        log.info("Sent APPLY_MAPPING - committed to NVS and rebound OneWire mapping")

        if args.no_verify:
            return 0

        # Give the DS18B20Manager task a couple of conversion cycles to rescan
        time.sleep(1.5)
        present = set(read_present_roms(rtu))
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
    except ModbusRTUError as e:
        log.error(f"Modbus I/O failure: {e}")
        return 1
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

    a = sub.add_parser("apply", parents=[common], help="Push sensors.yaml mapping + offsets to the ESP32 and commit to NVS")
    a.add_argument("--sensors", default="sensors.yaml", help="Path to sensors.yaml (default: ./sensors.yaml)")
    a.add_argument("--dry-run", action="store_true", help="Validate and show planned writes without opening the port")
    a.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    a.add_argument("--clear-first", action="store_true", help="Wipe NVS before applying (recommended if slot count changed)")
    a.add_argument("--no-verify", action="store_true", help="Skip the post-apply bus-scan verification")
    a.set_defaults(func=cmd_apply)

    c = sub.add_parser("clear", parents=[common], help="Wipe all sensor ROMs/offsets from ESP32 NVS")
    c.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    c.set_defaults(func=cmd_clear)

    return p


def main() -> int:
    args = build_parser().parse_args()
    try:
        return args.func(args)
    except ModbusRTUError as e:
        log.error(f"Modbus I/O failure: {e}")
        return 1
    except (ValueError, FileNotFoundError) as e:
        log.error(str(e))
        return 1


if __name__ == "__main__":
    sys.exit(main())