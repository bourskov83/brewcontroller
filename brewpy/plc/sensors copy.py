"""
Everything to do with the DS18B20 temperature sensors on the RTU slave:

- the slow polling loop that refreshes engineering + raw temp tags
- reading/writing the sensor ROM table (the "which physical sensor maps
  to which channel" config)
- reading/writing per-channel calibration offsets

Split out of SoftPLC because it's a self-contained concern: it only needs
an RTU client and a tag accessor, not any PID/scan state.
"""
import asyncio
from typing import List, Optional

from logger_config import log
from plc.helpers import decode_sensor_rom, encode_rom
from plc.sensor_map import SensorMapping
from plc.scaling import float_to_reg


APPLY_CONFIG_ADDR = 200
APPLY_CONFIG_VALUE = [0xA5A5]

SENSOR_COUNT_ADDR = 300
ROM_TABLE_ADDR = 301
ROM_CONFIG_WRITE_ADDR = 100
OFFSET_BASE_ADDR = 50
OFFSET_COUNT = 10  # 5 channels x 2 regs? kept as original magic number


class SensorIO:
    """
    Talks to the RTU slave on behalf of the temperature sensors:
    polling, ROM table management, and offset calibration.
    """

    def __init__(self, rtu, tags, sensor_map: List[SensorMapping], temp_base_addr: int = 30,
                temp_count: int = 8):
        self.rtu = rtu
        self.tags = tags
        self.sensor_map = sensor_map
        self._temp_base_addr = temp_base_addr
        self._temp_count = temp_count

    async def _apply_config_to_rtu(self) -> bool:
        """Tell the RTU slave to latch in whatever config registers we just wrote."""
        return await self.rtu.write_hregs(APPLY_CONFIG_ADDR, APPLY_CONFIG_VALUE)

    # ------------------------------------------------------------------
    # Polling loop
    # ------------------------------------------------------------------

    async def sensor_update_task(self, poll_freq: float = 0.8):
        """Background loop: refresh engineering + raw temperature tags."""
        log.info("sensor_update_task started")
        loop = asyncio.get_event_loop()

        while True:
            t0 = loop.time()

            await self._poll_block(
                base_addr=self._temp_base_addr,
                suffix="_temp",
            )

            elapsed = loop.time() - t0
            await asyncio.sleep(max(0, poll_freq - elapsed))

    async def _poll_block(self, base_addr: int, suffix: str) -> None:
        """Read one block of temp registers and write them to the matching tags."""
        try:
            regs = await self.rtu.read_hregs(base_addr, self._temp_count)
        except Exception as e:
            log.warning(f"Temperature block read failed: {e}")
            return

        if not regs:
            return
        try:
            for m in self.sensor_map:
                if 0 <= m.slot < len(regs):
                    # Apply sensor calibration offset before writing tag to modbus server
                    corrected_temp = regs[m.slot] + float_to_reg(m.offset)[0]
                    #self.tags.write(f"{m.sensor}{suffix}", regs[m.slot])
                    #log.debug(f"sensor: {m.sensor}: raw temp: {regs[m.slot]}, offset: {m.offset}, corrected_temp: {corrected_temp}")
                    self.tags.write(f"{m.sensor}{suffix}", corrected_temp)
        except Exception as e:
            log.error(e)
    # ------------------------------------------------------------------
    # ROM table (sensor identity) management
    # ------------------------------------------------------------------

    async def read_sensor_roms(self) -> Optional[List[str]]:
        """Read the sensor count + ROM table from the RTU slave."""
        try:
            sensor_count = await self.rtu.read_hregs(SENSOR_COUNT_ADDR, 1)
            log.info(f"Temperature sensor count: {sensor_count}")
            if sensor_count[0] == 0:
                log.warning("Sensor count is 0!")
                return None

            rom_list = await self.rtu.read_hregs(ROM_TABLE_ADDR, 4 * sensor_count[0])
            roms = []
            for i in range(0, len(rom_list), 4):
                rom = decode_sensor_rom(rom_list[i:i + 4])
                log.debug(f"regs: {rom_list[i:i + 4]} str: {rom.hex('-').upper()}")
                roms.append(rom.hex('-').upper())

            return roms

        except Exception as e:
            log.warning(f"Sensor ROM read failure: {e}")
            return None

    async def write_sensor_roms(self) -> bool:
        """
        Write the sensor ROM table to the RTU slave.
        """
        try:

            config_regs = []
            for m in self.sensor_map:
                config_regs.extend(encode_rom(m.rom))

            log.debug(config_regs)
            result = await self.rtu.write_hregs(ROM_CONFIG_WRITE_ADDR, config_regs)
            log.debug(f"config_regs write status: {result}")
            await self._apply_config_to_rtu()

            return True
        except Exception as e:
            log.warning(f"Sensor ROM write failure: {e}")
            return False

    # ------------------------------------------------------------------
    # Calibration offsets
    # ------------------------------------------------------------------

    async def read_offsets(self) -> bool:
        return None

    async def write_offsets(self) -> bool:
        return None
