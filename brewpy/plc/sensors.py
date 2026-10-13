"""
Everything to do with the DS18B20 temperature sensors on the RTU slave:

- the slow polling loop that refreshes the shared SensorStore
- applying the per-sensor calibration offset from sensors.yaml
  (the ROM table itself is pushed by sensor_config.py, not here)

Split out of SoftPLC because it's a self-contained concern: it only needs
an RTU client and a SensorStore to publish into, not any PID/scan state
or Modbus-server tag access. Publishing readings to tags/historian is
scan.py's job (single-writer principle) — this module never touches tags.
"""
import asyncio
from logger_config import log
from plc.scaling import unsigned_to_signed16
from plc.sensor_store import SensorStatus


class SensorIO:
    """
    Talks to the RTU slave on behalf of the temperature sensors:
    polling, ROM table management, and offset calibration.
    """

    def __init__(self, rtu, store, sensor_map, device):
        self.rtu = rtu
        self.store = store
        self.sensor_map = sensor_map
        self.device = device


    # ------------------------------------------------------------------
    # Polling loop
    # ------------------------------------------------------------------

    async def sensor_update_task(self, poll_freq: float = 0.8):
        """Background loop: refresh the shared SensorStore from the RTU slave."""
        log.info("sensor_update_task started")
        loop = asyncio.get_event_loop()
        while True:
            t0 = loop.time()
            try:
                await self._poll_block()
            except Exception as E:
                log.error(E)

            elapsed = loop.time() - t0
            await asyncio.sleep(max(0, poll_freq - elapsed))

    async def _poll_block(self) -> None:
        """Read one block of temp registers and publish each to the SensorStore."""
        try:
            regs = await self.rtu.read_hregs(self.device.temps.hreg_base, self.device.temps.count)
        except Exception as e:
            log.warning(f"Temperature block read failed: {e}")
            return

        if not regs:
            return
        try:
            for m in self.sensor_map:
                if not (0 <= m.slot < len(regs)):
                    continue

                raw = unsigned_to_signed16(regs[m.slot])
                if raw == self.device.temps.invalid:
                    # ESP32 has no reading for this slot right now (sensor
                    # missing / not yet discovered / CRC failure on the bus).
                    await self.store.update(m.tag, None, SensorStatus.FAULT)
                    continue

                # Apply sensor calibration offset before publishing (still
                # in signed degC x100 register space).
                corrected_temp = raw + round(m.offset * self.device.temps.scale)
                await self.store.update(m.tag, corrected_temp, SensorStatus.OK)

        except Exception as e:
            log.error(e)
