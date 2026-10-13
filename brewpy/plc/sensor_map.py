"""
Mapping of physical sensors to ESP32 slots and PLC tags (sensors.yaml).

The tag IS the sensor's name: "hlt_temp" in sensors.yaml is the same
"hlt_temp" in tags.yaml, in FUXA, in the logs and in SensorStore.
"""
from dataclasses import dataclass

from yamlutil import load_yaml


@dataclass
class SensorMapping:
    """One row of sensors.yaml."""
    tag: str                    # tags.yaml tag AND the sensor's name, e.g. "mlt_temp"
    slot: int                   # channel index on the ESP32 (hreg = hreg_base + slot)
    rom: str                    # DS18B20 ROM code as "28-8C-63-..." hex string
    roc_enabled: bool = False   # also track rate of change -> tag "<tag>_roc"
    offset: float = 0.0         # calibration offset in degC

    @property
    def roc_tag(self) -> str:
        return f"{self.tag}_roc"


def load_sensor_map(filename: str = "sensors.yaml") -> list[SensorMapping]:
    """Read sensors.yaml into a list of SensorMapping, sorted by slot."""
    sensor_list = load_yaml(filename)
    if not isinstance(sensor_list, list):
        raise TypeError(f"{filename}: expected a list of sensors")

    sensors = []
    for row in sensor_list:
        try:
            s = SensorMapping(**row)
        except TypeError as e:
            raise ValueError(f"{filename}: bad sensor row {row}: {e}") from e
        sensors.append(s)
    sensors.sort(key=lambda m: m.slot)
    return sensors
