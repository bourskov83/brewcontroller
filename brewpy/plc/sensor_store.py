import asyncio
import time
from dataclasses import dataclass, replace
from enum import Enum
from logger_config import log

class SensorStatus(Enum):
    OK = 0
    FAULT = 1
    STALE = 2

@dataclass(frozen=True)
class SensorReading:
    value: float | None
    status: SensorStatus
    last_update: float

class SensorStore:
    def __init__(self, stale_after_s: float):
        self._readings: dict[str, SensorReading] = {}
        self._stale_after_s = stale_after_s

    async def update(self, sensor_id: str, value: float | None, status: SensorStatus):
        self._readings[sensor_id] = SensorReading(value, status, time.monotonic())

    async def get_all(self) -> dict[str, SensorReading]:
        return dict(self._readings)  # shallow copy

    async def snapshot(self):
        now = time.monotonic()
        out = {}
        for tag, sensor_data in self._readings.items():
            if (
                sensor_data.status is SensorStatus.OK
                and now - sensor_data.last_update > self._stale_after_s
            ):
                sensor_data = replace(sensor_data, status=SensorStatus.STALE)
            out[tag] = sensor_data
        return out