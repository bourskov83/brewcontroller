#!/usr/bin/env python3
"""
check_config.py

Cross-checks tags.yaml, esp32.yaml and sensors.yaml against each other and
prints, for every tag, where its value comes from or goes to.

    python check_config.py            # overview + problems, exit 1 on errors
    python check_config.py --quiet    # problems only

SoftPLC calls validate() at startup, so a typo stops the PLC at boot with a
clear message instead of producing log spam every scan.
"""
from __future__ import annotations

import argparse
import sys

from esp32map import Esp32Map, load_esp32_map
from tagmap import TABLE_FC, Tag, load_tags


class ConfigError(Exception):
    """One or more configuration problems; the message lists them all."""


# Tags the scan logic reads or writes by literal name, and who must own them.
# When scan.py changes, change this list (or later: let scan.py declare it).
REQUIRED_TAGS: dict[str, str] = {
    "sensor_fault_word": "plc",
    "mash_target_temp": "hmi", "sparge_target_temp": "hmi", "pitch_target_temp": "hmi",
    "mash_target_delta": "plc", "sparge_target_delta": "plc", "pitch_target_delta": "plc",
    "chill_delta": "plc", "chill_max_wort_delta": "plc", "chill_effective_ratio": "plc",
    "mlt_temp": "plc", "hlt_temp": "plc", "bk_temp": "plc",
    "chill_inlet_temp": "plc", "chill_outlet_temp": "plc",
}


def validate(tags: dict[str, Tag], esp: Esp32Map, sensors) -> tuple[list[str], list[str]]:
    """Return (errors, warnings). Errors must stop startup."""
    errors: list[str] = []
    warnings: list[str] = []

    # --- sensors -> tags -----------------------------------------------
    slots: dict[int, str] = {}
    roms: dict[str, str] = {}
    sensor_tags: set[str] = set()
    for s in sensors:
        if not (0 <= s.slot < esp.temps.count):
            errors.append(
                f"sensor '{s.tag}': slot {s.slot} is outside the polled range "
                f"0..{esp.temps.count - 1} (inputs.temperatures.count)"
            )
        if s.slot in slots:
            errors.append(f"sensor '{s.tag}': slot {s.slot} is also used by '{slots[s.slot]}'")
        slots[s.slot] = s.tag

        rom = s.rom.upper()
        if rom in roms:
            errors.append(f"sensor '{s.tag}': ROM {s.rom} is also used by '{roms[rom]}'")
        roms[rom] = s.tag

        if s.tag in sensor_tags:
            errors.append(f"sensor '{s.tag}' appears more than once in sensors.yaml")
        sensor_tags.add(s.tag)

        t = tags.get(s.tag)
        if t is None:
            errors.append(f"sensor '{s.tag}' is not in tags.yaml")
        elif (t.table, t.dtype, t.owner) != ("ir", "int16", "plc"):
            errors.append(
                f"sensor '{s.tag}': tag should be ir/int16/plc "
                f"(is {t.table}/{t.dtype}/{t.owner})"
            )
        elif t.scale != esp.temps.scale:
            errors.append(
                f"sensor '{s.tag}': tag scale {t.scale} differs from the "
                f"ESP32 temperature scale {esp.temps.scale} (offsets and values would be off)"
            )

        if s.roc_enabled:
            r = tags.get(s.roc_tag)
            if r is None:
                errors.append(f"sensor '{s.tag}': roc_enabled but '{s.roc_tag}' is not in tags.yaml")
            elif (r.table, r.dtype, r.owner) != ("hr", "int16", "plc"):
                errors.append(
                    f"tag '{s.roc_tag}' should be hr/int16/plc (is {r.table}/{r.dtype}/{r.owner})"
                )

    # --- ESP32 coils -> tags -------------------------------------------
    for addr, name in sorted(esp.coils.items()):
        t = tags.get(name)
        if t is None:
            errors.append(f"esp32.yaml coil {addr}: tag '{name}' is not in tags.yaml")
        elif t.table != "coil":
            errors.append(f"esp32.yaml coil {addr}: tag '{name}' is a {t.table} tag, not a coil")

    bound = set(esp.coils.values())
    for t in tags.values():
        if t.table == "coil" and t.name not in bound:
            warnings.append(
                f"coil tag '{t.name}' is not bound to any ESP32 coil in esp32.yaml "
                f"(the HMI can set it, but nothing will happen)"
            )

    # --- tags the scan logic needs -------------------------------------
    for name, owner in REQUIRED_TAGS.items():
        t = tags.get(name)
        if t is None:
            errors.append(f"tag '{name}' is used by the scan logic but is not in tags.yaml")
        elif t.owner != owner:
            errors.append(f"tag '{name}' must be owner '{owner}' for the scan logic (is '{t.owner}')")

    # --- PLC-owned tags nothing seems to write -------------------------
    produced = set(sensor_tags) | {s.roc_tag for s in sensors if s.roc_enabled} | set(REQUIRED_TAGS)
    for t in tags.values():
        if t.owner == "plc" and t.name not in produced:
            warnings.append(f"tag '{t.name}' is plc-owned but nothing in the config produces it")

    return errors, warnings


def describe(tags: dict[str, Tag], esp: Esp32Map, sensors) -> str:
    """One row per tag: PLC address, type, owner and where the value comes from."""
    sensor_of = {s.tag: s for s in sensors}
    roc_of = {s.roc_tag: s for s in sensors if s.roc_enabled}
    coil_of = {name: addr for addr, name in esp.coils.items()}

    rows = []
    for t in sorted(tags.values(), key=lambda t: (TABLE_FC[t.table], t.addr)):
        if t.name in sensor_of:
            s = sensor_of[t.name]
            src = (f"sensor slot {s.slot} -> ESP32 hreg {esp.temp_hreg(s.slot)}"
                   f"  (offset {s.offset:+.2f})")
        elif t.name in roc_of:
            src = f"computed: rate of change of {roc_of[t.name].tag}"
        elif t.name in coil_of:
            src = f"HMI command -> ESP32 coil {coil_of[t.name]}"
        elif t.owner == "hmi":
            src = "HMI setpoint (no device)"
        else:
            src = "computed by the PLC"
        scale = f"x{t.scale} {t.unit}".strip() if t.dtype != "bool" else "bool"
        rows.append((f"{t.table.upper() if t.table != 'coil' else 'coil'} {t.addr}",
                     t.name, t.dtype, scale, t.owner, src))

    head = ("PLC addr", "tag", "dtype", "scale", "owner", "source")
    widths = [max(len(str(r[i])) for r in rows + [head]) for i in range(5)]
    fmt = "  ".join(f"{{:<{w}}}" for w in widths) + "  {}"
    lines = [fmt.format(*head), fmt.format(*("-" * w for w in widths), "-" * 6)]
    lines += [fmt.format(*r) for r in rows]
    return "\n".join(lines)


def load_all(tags_path="tags.yaml", esp32_path="esp32.yaml", sensors_path="sensors.yaml"):
    from plc.sensor_map import load_sensor_map      # imported late: keeps this module light
    return load_tags(tags_path), load_esp32_map(esp32_path), load_sensor_map(sensors_path)


def validate_or_raise(tags, esp, sensors, log=None) -> None:
    """Startup helper: log warnings, raise ConfigError listing every error."""
    errors, warnings = validate(tags, esp, sensors)
    for w in warnings:
        if log:
            log.warning(w)
    if errors:
        raise ConfigError("invalid configuration:\n  " + "\n  ".join(errors))


def main() -> int:
    ap = argparse.ArgumentParser(description="Check and describe the brewpy configuration")
    ap.add_argument("--tags", default="tags.yaml")
    ap.add_argument("--esp32", default="esp32.yaml")
    ap.add_argument("--sensors", default="sensors.yaml")
    ap.add_argument("--quiet", action="store_true", help="print problems only, no overview table")
    args = ap.parse_args()

    try:
        tags, esp, sensors = load_all(args.tags, args.esp32, args.sensors)
    except (ValueError, OSError) as e:
        print(f"ERROR loading configuration: {e}", file=sys.stderr)
        return 1

    if not args.quiet:
        print(describe(tags, esp, sensors))
        print()
    errors, warnings = validate(tags, esp, sensors)
    for w in warnings:
        print(f"warning: {w}")
    for e in errors:
        print(f"ERROR:   {e}")
    if not errors:
        print(f"OK: {len(tags)} tags, {len(sensors)} sensors, {len(esp.coils)} ESP32 coils"
              + (f", {len(warnings)} warning(s)" if warnings else ""))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
