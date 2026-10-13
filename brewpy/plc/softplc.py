"""
Top-level SoftPLC: owns shared state (tags, sensor store, RTU client) and wires
together the three background tasks. The actual task logic now lives in
plc/scan.py, plc/bridge.py, and plc/sensors.py — this class is just
construction + lifecycle (start/stop).
"""
import asyncio

from logger_config import log
from tagmap import Tags

from plc.sensors import SensorIO
from plc.scan import ScanTask
from plc.bridge import BridgeTask
from plc.sensor_store import SensorStore


class SoftPLC:
    def __init__(self, context, rtu, tag_defs, device, sensor_map):
        self.device = device
        self.tags = Tags(store=context[0x00], tags=tag_defs)
        self.sensor_store = SensorStore()
        self.sensors = SensorIO(rtu, self.sensor_store, sensor_map, device)
        self.scan_task = ScanTask(self.tags, rtu, self.sensor_store, sensor_map, device)
        self.bridge_task = BridgeTask(self.tags)
        self._tasks = []

    async def start(self):
        """Start the PLC runtime: background tasks for scan, temps, bridge, etc."""
        log.info("SoftPLC.start()")
        self._tasks = [
            asyncio.create_task(self.scan_task.run(), name="scan_task"),
            asyncio.create_task(self.sensors.sensor_update_task(), name="sensor_update_task"),
            asyncio.create_task(self.bridge_task.run(), name="modbus_bridge_task"),
        ]

    async def stop(self):
        """Gracefully stop all PLC tasks."""
        log.info("SoftPLC.stop()")
        for t in self._tasks:
            t.cancel()
        if self._tasks:
            await asyncio.gather(*self._tasks, return_exceptions=True)
        self._tasks.clear()