"""
Top-level SoftPLC: owns shared state (tags, PID, RTU client) and wires
together the three background tasks. The actual task logic now lives in
plc/scan.py, plc/bridge.py, and plc/sensors.py — this class is just
construction + lifecycle (start/stop).
"""
import asyncio

from logger_config import log
from tagmap import Tags, load_tags
from pymodbus.datastore import ModbusServerContext

from modbus_rtu_client import AsyncModbusRTUClient
from pid_control import CascadePIDController
from output_control import SSRController

from plc.sensors import SensorIO
from plc.scan import ScanTask
from plc.bridge import BridgeTask
from plc.sensor_map import SensorMapping, load_sensor_map
from plc.sensor_store import SensorStore


class SoftPLC:
    def __init__(self, context: ModbusServerContext, rtu: AsyncModbusRTUClient,
                 tagmap_path: str = "tags.yaml", sensor_map_file: str = "sensors.yaml"):
        self.context = context
        self.rtu = rtu

        self._tagmap = load_tags(tagmap_path)
        self.tags = Tags(store=context[0x00], mapping=self._tagmap)
        

        self.pid = CascadePIDController()
        self.ssr = SSRController(cycle_time=4.0)


        self._sensormap = load_sensor_map(sensor_map_file)
        self.sensor_store = SensorStore()
        self.sensors = SensorIO(rtu=self.rtu, store=self.sensor_store, sensor_map=self._sensormap)
        self.scan_task = ScanTask(tags=self.tags, rtu=self.rtu, pid=self.pid, ssr=self.ssr,
                                   store=self.sensor_store)
        self.bridge_task = BridgeTask(tags=self.tags, pid=self.pid, scan_task=self.scan_task)
        self.bridge_task.attach_sensors(self.sensors)

        self._tasks: list[asyncio.Task] = []

    async def start(self):
        """Start the PLC runtime: background tasks for scan, temps, bridge, etc."""
        log.info("SoftPLC.start()")
        self._tasks = [
            asyncio.create_task(self.scan_task.run(), name="scan_task"),
            asyncio.create_task(self.sensors.sensor_update_task(), name="sensor_update_task"),
            asyncio.create_task(self.bridge_task.run(self._tagmap), name="modbus_bridge_task"),
        ]

    async def stop(self):
        """Gracefully stop all PLC tasks."""
        log.info("SoftPLC.stop()")
        for t in self._tasks:
            t.cancel()
        if self._tasks:
            await asyncio.gather(*self._tasks, return_exceptions=True)
        self._tasks.clear()