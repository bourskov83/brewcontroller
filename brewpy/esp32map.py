"""
Typed view of esp32.yaml: the ESP32 gateway's register interface.

Pure data, no I/O. plc/sensors.py, scan.py, server.py and sensor_config.py
all take their ESP32 addresses from here instead of keeping their own
copies of the constants.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path

from yamlutil import load_yaml


class Esp32ConfigError(ValueError):
    """esp32.yaml is malformed or inconsistent."""


@dataclass(frozen=True)
class Link:
    port: str
    baud: int
    device_id: int
    timeout: float


@dataclass(frozen=True)
class Temperatures:
    hreg_base: int
    count: int
    scale: int
    invalid: int
    stale_after_s: int


@dataclass(frozen=True)
class Commands:
    scan: int
    apply_mapping: int
    clear_config: int


@dataclass(frozen=True)
class Status:
    idle: int
    busy: int
    done: int
    error: int


@dataclass(frozen=True)
class Commissioning:
    max_slots: int
    max_disc: int
    rom_reg_stride: int
    rom_config_base: int
    slot_cmd: int
    slot_cmd_status: int
    present_count: int
    present_roms: int
    log_level_all: int
    log_level_base: int
    log_tags: tuple[str, ...]
    commands: Commands
    status: Status


@dataclass(frozen=True)
class Esp32Map:
    device: str
    link: Link
    temps: Temperatures
    coils: dict[int, str]            # ESP32 coil address -> PLC tag name
    commissioning: Commissioning

    def temp_hreg(self, slot: int) -> int:
        """Holding register the firmware publishes slot `slot` in."""
        return self.temps.hreg_base + slot

    def coil_block(self) -> list[str]:
        """Tag names in coil-address order (0, 1, 2, ...) for one write_coils()."""
        return [self.coils[a] for a in sorted(self.coils)]

    def hreg_ranges(self) -> list[tuple[str, int, int]]:
        """(name, first, last_inclusive) for every holding-register block."""
        c = self.commissioning
        return [
            ("temperatures", self.temps.hreg_base, self.temps.hreg_base + self.temps.count - 1),
            ("rom_config", c.rom_config_base, c.rom_config_base + c.max_slots * c.rom_reg_stride - 1),
            ("slot_cmd", c.slot_cmd, c.slot_cmd),
            ("slot_cmd_status", c.slot_cmd_status, c.slot_cmd_status),
            ("log_level_all", c.log_level_all, c.log_level_all),
            ("log_levels", c.log_level_base, c.log_level_base + len(c.log_tags) - 1),
            ("present_count", c.present_count, c.present_count),
            ("present_roms", c.present_roms, c.present_roms + c.max_disc * c.rom_reg_stride - 1),
        ]


# ----------------------------------------------------------------------

def _build(cls, raw, where: str):
    """Make dataclass `cls` from a dict, rejecting missing and unknown keys."""
    if not isinstance(raw, dict):
        raise Esp32ConfigError(f"{where}: expected a mapping")
    names = {f.name for f in fields(cls)}
    unknown = set(raw) - names
    missing = names - set(raw)
    if unknown:
        raise Esp32ConfigError(f"{where}: unknown key(s) {sorted(unknown)}")
    if missing:
        raise Esp32ConfigError(f"{where}: missing key(s) {sorted(missing)}")
    return cls(**raw)


def load_esp32_map(path: str | Path = "esp32.yaml") -> Esp32Map:
    doc = load_yaml(path) or {}
    if not isinstance(doc, dict):
        raise Esp32ConfigError(f"{path}: top level must be a mapping")
    for key in ("device", "link", "inputs", "outputs", "commissioning"):
        if key not in doc:
            raise Esp32ConfigError(f"{path}: missing section '{key}'")

    try:
        link = _build(Link, doc["link"], "link")
        temps = _build(Temperatures, doc["inputs"].get("temperatures"), "inputs.temperatures")
        coils = doc["outputs"].get("coils")
        if not isinstance(coils, dict) or not coils:
            raise Esp32ConfigError("outputs.coils: expected a mapping of coil address -> tag")

        com = dict(doc["commissioning"])
        com["commands"] = _build(Commands, com.get("commands"), "commissioning.commands")
        com["status"] = _build(Status, com.get("status"), "commissioning.status")
        com["log_tags"] = tuple(com.get("log_tags", ()))
        commissioning = _build(Commissioning, com, "commissioning")
    except AttributeError as e:             # e.g. "inputs" is not a mapping
        raise Esp32ConfigError(f"{path}: {e}") from e

    esp = Esp32Map(
        device=str(doc["device"]), link=link, temps=temps,
        coils={int(a): str(t) for a, t in coils.items()},
        commissioning=commissioning,
    )
    errors = validate_esp32_map(esp)
    if errors:
        raise Esp32ConfigError(f"{path}:\n  " + "\n  ".join(errors))
    return esp


def validate_esp32_map(esp: Esp32Map) -> list[str]:
    """Internal consistency of the device description. Returns error strings."""
    errors: list[str] = []

    # Coils must be 0..n-1 with no gaps: the PLC writes them as one block.
    if sorted(esp.coils) != list(range(len(esp.coils))):
        errors.append(
            f"outputs.coils addresses must be contiguous from 0 (got {sorted(esp.coils)})"
        )
    dup = {t for t in esp.coils.values() if list(esp.coils.values()).count(t) > 1}
    if dup:
        errors.append(f"outputs.coils: tag(s) bound to more than one coil: {sorted(dup)}")

    c = esp.commissioning
    if esp.temps.count > c.max_slots:
        errors.append(
            f"inputs.temperatures.count ({esp.temps.count}) exceeds "
            f"commissioning.max_slots ({c.max_slots})"
        )
    if esp.temps.scale < 1:
        errors.append("inputs.temperatures.scale must be >= 1")
    if not (-32768 <= esp.temps.invalid <= 32767):
        errors.append("inputs.temperatures.invalid must fit in int16")
    if len(set(c.log_tags)) != len(c.log_tags):
        errors.append("commissioning.log_tags contains duplicates")

    # No two holding-register blocks may overlap.
    ranges = sorted(esp.hreg_ranges(), key=lambda r: r[1])
    for (n1, _, hi1), (n2, lo2, _) in zip(ranges, ranges[1:]):
        if lo2 <= hi1:
            errors.append(f"holding-register blocks overlap: '{n1}' and '{n2}'")

    cmds = {esp.commissioning.commands.scan, c.commands.apply_mapping, c.commands.clear_config}
    if len(cmds) != 3 or c.status.idle in cmds:
        errors.append("commissioning.commands must be three distinct non-idle codes")
    return errors
