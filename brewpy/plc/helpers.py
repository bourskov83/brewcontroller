"""
Various helper methods
"""

from typing import List

def decode_sensor_rom(registers: List[int]) -> bytes:
    """
    Decode 4 holding registers into an 8-byte DS18B20 ROM code.

    Byte order: little-endian within each register (low byte first).
    """
    if len(registers) != 4:
        raise ValueError("Expected exactly 4 registers per DS18B20 ROM")

    b = bytearray()
    for w in registers:
        b.append(w & 0xFF)
        b.append((w >> 8) & 0xFF)

    return bytes(b)


def encode_rom(rom_str: str) -> List[int]:
    """
    Encode an 8-byte ROM string (e.g. "28-8C-63-5B-0F-00-00-00") into
    4 holding registers, matching decode_sensor_rom's byte order.
    """
    rom = bytes.fromhex(rom_str.replace('-', '').replace(':', ''))

    if len(rom) != 8:
        raise ValueError("ROM must be exactly 8 bytes")

    regs = []
    # fixed: byte-order=little, word-order=big
    for i in range(0, 8, 2):
        regs.append(rom[i] | (rom[i + 1] << 8))
    return regs
