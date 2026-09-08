"""
Register <-> engineering-value conversions, and DS18B20 ROM encode/decode.

Pulled out of SoftPLC because they're pure functions with no dependency
on PLC state — easier to unit test in isolation, and they were cluttering
the class body.
"""
from typing import List


def float_to_reg(x: float, scale: int = 100, clamp: tuple[int, int] = (-32768, 32767)) -> List[int]:
    """Scale a float into a single signed 16-bit Modbus register."""
    v = int(round(x * scale))
    lo, hi = clamp
    if v < lo:
        v = lo
    if v > hi:
        v = hi
    return [v]


def reg_to_float(x: int, scale: int = 100) -> float:
    """Unscale a single Modbus register back into a float."""
    return float(x / scale)


def unsigned_to_signed16(v: int) -> int:
    """
    pymodbus hands back holding registers as unsigned 0..65535 ints, but
    the ESP32 packs temperatures/offsets as signed int16 (degC x100).
    Reinterpret the raw unsigned word as its signed two's-complement value.
    """
    return v - 0x10000 if v >= 0x8000 else v