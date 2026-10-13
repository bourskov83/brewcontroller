"""
The softPLC tag database.

A *tag* is a name laid over one address in the PLC's Modbus datastore. The
datastore is the process image: PLC code reads/writes it by name, and FUXA
reads/writes the very same addresses over Modbus TCP. This file knows nothing
about the ESP32 (that is esp32.yaml / esp32map.py).

Every tag declares:

    table   coil | di | hr | ir     which Modbus table it lives in
    addr    0..65535                address inside that table
    dtype   bool | int16 | uint16   how the register is interpreted
    owner   plc | hmi               who is allowed to write it
    scale   int (default 1)         register value = engineering value x scale
    unit    str (optional)          for humans / the overview table
    desc    str (optional)

Registers always hold the scaled integer (65.23 degC -> 6523). Nothing on the
Modbus TCP side changes because of `scale`; it only tells the PLC code how to
read the integer. Two accessor pairs are provided:

    get_raw / set_raw   the integer exactly as FUXA sees it (signed for int16)
    get_eng / set_eng   engineering float, scaled and saturating

Use the raw pair for PLC logic that stays in x100 integers (the default); use
the eng pair when you want to think in degC.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from yamlutil import load_yaml

# Table name -> Modbus function code. pymodbus' datastore picks the right
# block from the function code: 1 = coils, 2 = discrete inputs,
# 3 = holding registers, 4 = input registers.
TABLE_FC = {"coil": 1, "di": 2, "hr": 3, "ir": 4}
BIT_TABLES = ("coil", "di")
REG_TABLES = ("hr", "ir")
READONLY_TABLES = ("di", "ir")      # a Modbus client cannot write these
DTYPES = ("bool", "int16", "uint16")
OWNERS = ("plc", "hmi")

_ALLOWED_KEYS = {"table", "addr", "dtype", "owner", "scale", "unit", "desc"}
_REQUIRED_KEYS = ("table", "addr", "dtype", "owner")


class TagConfigError(ValueError):
    """tags.yaml is malformed or inconsistent."""


class TagAccessError(RuntimeError):
    """The datastore refused a read or write (address outside the allocated block)."""


@dataclass(frozen=True)
class Tag:
    name: str
    table: str
    addr: int
    dtype: str
    owner: str
    scale: int = 1
    unit: str = ""
    desc: str = ""

    @property
    def fc(self) -> int:
        return TABLE_FC[self.table]

    @property
    def raw_min(self) -> int:
        return {"bool": 0, "int16": -32768, "uint16": 0}[self.dtype]

    @property
    def raw_max(self) -> int:
        return {"bool": 1, "int16": 32767, "uint16": 65535}[self.dtype]


# ----------------------------------------------------------------------
# Loading + validation
# ----------------------------------------------------------------------

def _parse_tag(name: str, spec: Any) -> Tag:
    def err(msg: str) -> TagConfigError:
        return TagConfigError(f"tag '{name}': {msg}")

    if not isinstance(spec, dict):
        raise err("expected a mapping like { table: ir, addr: 0, ... }")

    unknown = set(spec) - _ALLOWED_KEYS
    if unknown:
        raise err(f"unknown key(s) {sorted(unknown)} (allowed: {sorted(_ALLOWED_KEYS)})")
    missing = [k for k in _REQUIRED_KEYS if k not in spec]
    if missing:
        raise err(f"missing required key(s) {missing}")

    table, dtype, owner = spec["table"], spec["dtype"], spec["owner"]
    addr, scale = spec["addr"], spec.get("scale", 1)

    if table not in TABLE_FC:
        raise err(f"table '{table}' must be one of {list(TABLE_FC)}")
    if dtype not in DTYPES:
        raise err(f"dtype '{dtype}' must be one of {list(DTYPES)}")
    if owner not in OWNERS:
        raise err(f"owner '{owner}' must be one of {list(OWNERS)}")
    if table in BIT_TABLES and dtype != "bool":
        raise err(f"table '{table}' holds bits, so dtype must be bool (got {dtype})")
    if table in REG_TABLES and dtype == "bool":
        raise err(f"table '{table}' holds 16-bit registers, dtype bool is not allowed")
    if table in READONLY_TABLES and owner != "plc":
        raise err(f"table '{table}' is read-only for Modbus clients, so owner must be plc")
    if isinstance(addr, bool) or not isinstance(addr, int) or not (0 <= addr <= 0xFFFF):
        raise err(f"addr must be an integer 0..65535 (got {addr!r})")
    if isinstance(scale, bool) or not isinstance(scale, int) or scale < 1:
        raise err(f"scale must be an integer >= 1 (got {scale!r})")
    if dtype == "bool" and scale != 1:
        raise err("a bool tag cannot have a scale")

    return Tag(
        name=name, table=table, addr=addr, dtype=dtype, owner=owner,
        scale=scale, unit=str(spec.get("unit", "")), desc=str(spec.get("desc", "")),
    )


def load_tags(path: str | Path = "tags.yaml") -> dict[str, Tag]:
    """Parse tags.yaml. Keys starting with '_' are YAML templates, not tags."""
    doc = load_yaml(path) or {}
    if not isinstance(doc, dict):
        raise TagConfigError(f"{path}: top level must be a mapping of tag name -> spec")

    tags: dict[str, Tag] = {}
    at: dict[tuple[str, int], str] = {}
    for name, spec in doc.items():
        if not isinstance(name, str):
            raise TagConfigError(f"{path}: tag name {name!r} must be a string")
        if name.startswith("_"):
            continue
        tag = _parse_tag(name, spec)
        key = (tag.table, tag.addr)
        if key in at:
            raise TagConfigError(
                f"{path}: '{name}' and '{at[key]}' both use {tag.table} address {tag.addr}"
            )
        at[key] = name
        tags[name] = tag
    return tags


def table_sizes(tags: dict[str, Tag], minimum: int = 0) -> dict[str, int]:
    """Number of addresses (0..size-1) each datastore table must cover.

    Build the pymodbus blocks with a start address of 1 and exactly this many
    entries: ModbusSequentialDataBlock(1, [0] * size). pymodbus adds 1 to
    every address internally, so a block that starts at 0 comes up one entry
    short and the highest tag silently fails.
    """
    sizes = {t: minimum for t in TABLE_FC}
    for tag in tags.values():
        sizes[tag.table] = max(sizes[tag.table], tag.addr + 1)
    return sizes


# ----------------------------------------------------------------------
# Runtime accessor
# ----------------------------------------------------------------------

class Tags:
    """Typed read/write access to the datastore by tag name."""

    def __init__(self, store, tags: dict[str, Tag], writer: str = "plc"):
        self._store = store
        self._tags = tags
        self._writer = writer

    @property
    def defs(self) -> dict[str, Tag]:
        """All tag definitions, by name."""
        return self._tags

    def tag(self, name: str) -> Tag:
        try:
            return self._tags[name]
        except KeyError:
            raise KeyError(f"unknown tag '{name}' (not in tags.yaml)") from None

    # --- raw: the integer exactly as Modbus clients see it ---------------

    def get_raw(self, name: str) -> int | bool:
        t = self.tag(name)
        result = self._store.getValues(t.fc, t.addr, count=1)
        if not isinstance(result, (list, tuple)):
            # pymodbus returns an ExcCodes value instead of raising
            raise TagAccessError(
                f"read of '{name}' ({t.table} {t.addr}) rejected by the datastore: {result!r}"
            )
        value = result[0]
        if t.dtype == "bool":
            return bool(value)
        value = int(value)
        if t.dtype == "int16" and value >= 0x8000:
            value -= 0x10000
        return value

    def set_raw(self, name: str, value: int | bool, *, override: bool = False) -> None:
        t = self.tag(name)
        self._check_owner(t, override)
        if t.dtype == "bool":
            if not isinstance(value, (bool, int)) or value not in (0, 1):
                raise ValueError(f"tag '{name}' is bool, got {value!r}")
            self._write(t, [bool(value)])
            return
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(
                f"tag '{name}' ({t.dtype}) needs an int, got {type(value).__name__} "
                f"{value!r} (use set_eng for floats)"
            )
        if not (t.raw_min <= value <= t.raw_max):
            raise ValueError(
                f"tag '{name}': {value} is outside the {t.dtype} range "
                f"{t.raw_min}..{t.raw_max}"
            )
        self._write(t, [value & 0xFFFF])        # two's complement for negatives

    # --- eng: engineering units, scaled -----------------------------------

    def get_eng(self, name: str) -> float:
        t = self.tag(name)
        if t.dtype == "bool":
            raise TypeError(f"tag '{name}' is bool, use get_raw")
        return self.get_raw(name) / t.scale

    def set_eng(self, name: str, value: float, *, override: bool = False) -> None:
        """Scale, round and SATURATE at the dtype limits (like the old float_to_reg)."""
        t = self.tag(name)
        if t.dtype == "bool":
            raise TypeError(f"tag '{name}' is bool, use set_raw")
        raw = int(round(value * t.scale))
        raw = max(t.raw_min, min(t.raw_max, raw))
        self.set_raw(name, raw, override=override)

    def _write(self, t: Tag, values: list) -> None:
        result = self._store.setValues(t.fc, t.addr, values)
        if result is not None:                  # None = success; else an ExcCodes value
            raise TagAccessError(
                f"write of '{t.name}' ({t.table} {t.addr}) rejected by the datastore: {result!r}"
            )

    def _check_owner(self, t: Tag, override: bool) -> None:
        if t.owner != self._writer and not override:
            raise PermissionError(
                f"tag '{t.name}' is owned by '{t.owner}'; the {self._writer} "
                f"may not write it (pass override=True for startup/reset)"
            )
