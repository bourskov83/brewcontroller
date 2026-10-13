"""
Run from the project root:   python -m pytest tests/ -q
Uses a real pymodbus datastore, so it also proves the table -> function-code
mapping (coil=1, hr=3, ir=4) against the pymodbus version you have installed.
"""
import sys
from pathlib import Path

import pytest
import yaml
from pymodbus.datastore import ModbusDeviceContext, ModbusSequentialDataBlock

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from tagmap import TagAccessError, TagConfigError, Tags, load_tags, table_sizes   # noqa: E402
from esp32map import Esp32ConfigError, load_esp32_map             # noqa: E402
import check_config                                               # noqa: E402


def write_yaml(tmp_path, text, name="t.yaml"):
    p = tmp_path / name
    p.write_text(text)
    return p


def make_store(tag_defs, minimum=0):
    """Build the datastore the way server.py should: blocks start at 1, exact size."""
    n = table_sizes(tag_defs, minimum=minimum)
    return ModbusDeviceContext(
        di=ModbusSequentialDataBlock(1, [0] * max(n["di"], 1)),
        co=ModbusSequentialDataBlock(1, [False] * n["coil"]),
        ir=ModbusSequentialDataBlock(1, [0] * n["ir"]),
        hr=ModbusSequentialDataBlock(1, [0] * n["hr"]),
    )


@pytest.fixture
def tags_and_store():
    defs = load_tags(ROOT / "tags.yaml")
    store = make_store(defs)
    return Tags(store, defs), store, defs


# --- the shipped config -------------------------------------------------

def test_shipped_config_is_valid():
    tag_defs, esp, sensors = check_config.load_all(
        ROOT / "tags.yaml", ROOT / "esp32.yaml", ROOT / "sensors.yaml")
    errors, warnings = check_config.validate(tag_defs, esp, sensors)
    assert errors == []
    assert warnings == []


# --- signedness: the bug class this file exists to kill -----------------

def test_negative_temperature_roundtrip(tags_and_store):
    tags, store, defs = tags_and_store
    tags.set_raw("mlt_temp", -500)                       # -5.00 degC
    assert tags.get_raw("mlt_temp") == -500
    assert store.getValues(4, defs["mlt_temp"].addr, count=1)[0] == 65036   # what Modbus carries
    assert tags.get_eng("mlt_temp") == pytest.approx(-5.0)


def test_uint16_not_signed(tags_and_store):
    tags, _, _ = tags_and_store
    tags.set_raw("sensor_fault_word", 0x8001)
    assert tags.get_raw("sensor_fault_word") == 0x8001


def test_set_eng_scales_and_saturates(tags_and_store):
    tags, _, _ = tags_and_store
    tags.set_eng("mlt_temp", 65.23)
    assert tags.get_raw("mlt_temp") == 6523
    tags.set_eng("mlt_temp", 1000.0)                     # 100000 would overflow int16
    assert tags.get_raw("mlt_temp") == 32767
    tags.set_eng("chill_effective_ratio", 0.456)
    assert tags.get_raw("chill_effective_ratio") == 46


def test_set_raw_rejects_float_and_out_of_range(tags_and_store):
    tags, _, _ = tags_and_store
    with pytest.raises(TypeError):
        tags.set_raw("mlt_temp", 65.23)
    with pytest.raises(ValueError):
        tags.set_raw("mlt_temp", 40000)
    with pytest.raises(ValueError):
        tags.set_raw("sensor_fault_word", -1)


def test_coil_roundtrip_and_override(tags_and_store):
    tags, store, defs = tags_and_store
    with pytest.raises(PermissionError):
        tags.set_raw("mlt_pump", True)                    # hmi-owned, PLC may not write
    tags.set_raw("mlt_pump", True, override=True)
    assert tags.get_raw("mlt_pump") is True
    assert store.getValues(1, defs["mlt_pump"].addr, count=1)[0] in (True, 1)


def test_plc_cannot_write_hmi_setpoint_and_unknown_tag(tags_and_store):
    tags, _, _ = tags_and_store
    with pytest.raises(PermissionError):
        tags.set_raw("mash_target_temp", 6700)
    with pytest.raises(KeyError, match="unknown tag"):
        tags.get_raw("mlt_tmep")


# --- tags.yaml validation -----------------------------------------------

def test_duplicate_yaml_key_is_rejected(tmp_path):
    p = write_yaml(tmp_path, """
a: { table: ir, addr: 0, dtype: int16, owner: plc }
a: { table: ir, addr: 1, dtype: int16, owner: plc }
""")
    with pytest.raises(yaml.YAMLError, match="duplicate key"):
        load_tags(p)


def test_duplicate_address_is_rejected(tmp_path):
    p = write_yaml(tmp_path, """
a: { table: ir, addr: 0, dtype: int16, owner: plc }
b: { table: ir, addr: 0, dtype: int16, owner: plc }
""")
    with pytest.raises(TagConfigError, match="both use ir address 0"):
        load_tags(p)


@pytest.mark.parametrize("spec, msg", [
    ("{ table: ir, addr: 0, dtype: int16 }", "missing required"),
    ("{ table: ir, addr: 0, dtype: int16, owner: plc, scal: 100 }", "unknown key"),
    ("{ table: xx, addr: 0, dtype: int16, owner: plc }", "table"),
    ("{ table: coil, addr: 0, dtype: int16, owner: hmi }", "bits"),
    ("{ table: ir, addr: 0, dtype: int16, owner: hmi }", "read-only"),
    ("{ table: hr, addr: 70000, dtype: int16, owner: plc }", "addr"),
    ("{ table: hr, addr: 0, dtype: int16, owner: plc, scale: 0 }", "scale"),
])
def test_bad_tag_specs(tmp_path, spec, msg):
    p = write_yaml(tmp_path, f"a: {spec}\n")
    with pytest.raises(TagConfigError, match=msg):
        load_tags(p)


def test_merge_key_override_is_allowed(tmp_path):
    p = write_yaml(tmp_path, """
_t: &t { table: ir, dtype: int16, scale: 100, owner: plc }
a: { <<: *t, addr: 0, scale: 10 }
""")
    assert load_tags(p)["a"].scale == 10


def test_table_sizes():
    defs = load_tags(ROOT / "tags.yaml")
    sizes = table_sizes(defs)
    assert sizes["ir"] == 20 and sizes["hr"] == 42 and sizes["coil"] == 3


# --- cross-file validation ----------------------------------------------

def load_shipped():
    return check_config.load_all(ROOT / "tags.yaml", ROOT / "esp32.yaml", ROOT / "sensors.yaml")


def test_sensor_pointing_at_missing_tag_is_an_error():
    tag_defs, esp, sensors = load_shipped()
    sensors[0].tag = "mlt_tmep"
    errors, _ = check_config.validate(tag_defs, esp, sensors)
    assert any("mlt_tmep" in e and "not in tags.yaml" in e for e in errors)


def test_roc_tag_missing_is_an_error():
    tag_defs, esp, sensors = load_shipped()
    del tag_defs["mlt_temp_roc"]
    errors, _ = check_config.validate(tag_defs, esp, sensors)
    assert any("mlt_temp_roc" in e for e in errors)


def test_duplicate_slot_and_rom_are_errors():
    tag_defs, esp, sensors = load_shipped()
    sensors[1].slot = sensors[0].slot
    sensors[2].rom = sensors[0].rom
    errors, _ = check_config.validate(tag_defs, esp, sensors)
    assert any("slot 0" in e for e in errors)
    assert any("ROM" in e for e in errors)


def test_unbound_coil_tag_is_a_warning():
    tag_defs, esp, sensors = load_shipped()
    from tagmap import Tag
    tag_defs["spare_pump"] = Tag("spare_pump", "coil", 3, "bool", "hmi")
    errors, warnings = check_config.validate(tag_defs, esp, sensors)
    assert errors == []
    assert any("spare_pump" in w for w in warnings)


# --- esp32.yaml validation ----------------------------------------------

def test_esp32_coil_gap_and_overlap_are_rejected(tmp_path):
    base = yaml.safe_load((ROOT / "esp32.yaml").read_text())

    gap = yaml.safe_load(yaml.safe_dump(base))
    gap["outputs"]["coils"] = {0: "a", 2: "b"}
    p = write_yaml(tmp_path, yaml.safe_dump(gap), "gap.yaml")
    with pytest.raises(Esp32ConfigError, match="contiguous"):
        load_esp32_map(p)

    overlap = yaml.safe_load(yaml.safe_dump(base))
    overlap["commissioning"]["slot_cmd_status"] = 200       # same register as slot_cmd
    p = write_yaml(tmp_path, yaml.safe_dump(overlap), "overlap.yaml")
    with pytest.raises(Esp32ConfigError, match="overlap"):
        load_esp32_map(p)


def test_esp32_unknown_key_is_rejected(tmp_path):
    base = yaml.safe_load((ROOT / "esp32.yaml").read_text())
    base["link"]["baudrate"] = 9600
    p = write_yaml(tmp_path, yaml.safe_dump(base))
    with pytest.raises(Esp32ConfigError, match="unknown key"):
        load_esp32_map(p)


def test_esp32_helpers():
    esp = load_esp32_map(ROOT / "esp32.yaml")
    assert esp.temp_hreg(3) == 13
    assert esp.coil_block() == ["mlt_pump", "hlt_pump", "bk_pump"]
    assert esp.commissioning.commands.clear_config == 0xC1EA


# --- datastore sizing (pymodbus shifts every address by one) --------------

def test_highest_address_of_every_table_is_usable():
    """With blocks sized by table_sizes() and started at 1, the last tag works."""
    defs = load_tags(ROOT / "tags.yaml")
    tags = Tags(make_store(defs), defs)
    tags.set_raw("chill_outlet_temp", -1234)           # highest ir address (19)
    tags.set_raw("chill_effective_ratio", 55)          # highest hr address (41)
    tags.set_raw("bk_pump", True, override=True)       # highest coil address (2)
    assert tags.get_raw("chill_outlet_temp") == -1234
    assert tags.get_raw("chill_effective_ratio") == 55
    assert tags.get_raw("bk_pump") is True


def test_datastore_that_is_too_small_fails_loudly():
    """The old behaviour was a silent no-op; now it is an exception."""
    defs = load_tags(ROOT / "tags.yaml")
    n = table_sizes(defs)
    small = ModbusDeviceContext(                       # the classic start-at-0 mistake
        di=ModbusSequentialDataBlock(0, [0]),
        co=ModbusSequentialDataBlock(0, [False] * n["coil"]),
        ir=ModbusSequentialDataBlock(0, [0] * n["ir"]),
        hr=ModbusSequentialDataBlock(0, [0] * n["hr"]),
    )
    tags = Tags(small, defs)
    with pytest.raises(TagAccessError):
        tags.set_raw("chill_outlet_temp", 100)
    with pytest.raises(TagAccessError):
        tags.get_raw("chill_outlet_temp")
