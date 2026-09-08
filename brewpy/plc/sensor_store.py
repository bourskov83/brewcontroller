import asyncio
import time
from dataclasses import dataclass
from enum import Enum

class SensorStatus(Enum):
    OK = 0
    FAULT = 1
    STALE = 2

@dataclass
class SensorReading:
    value: float | None
    status: SensorStatus
    last_update: float

class SensorStore:
    def __init__(self):
        self._lock = asyncio.Lock()
        self._readings: dict[str, SensorReading] = {}

    async def update(self, sensor_id: str, value: float | None, status: SensorStatus):
        async with self._lock:
            self._readings[sensor_id] = SensorReading(value, status, time.monotonic())

    async def get_all(self) -> dict[str, SensorReading]:
        async with self._lock:
            return dict(self._readings)  # shallow copy