"""
Mapping of sensor roms, offsets etc.
"""

import yaml


from dataclasses import dataclass
from typing import List


@dataclass
class SensorMapping:
    """One row of sensor_map.yaml: which physical sensor lives in which slot."""
    sensor: str                 # tag-name prefix, e.g. "mlt_temp"
    slot: int                   # channel index in the RTU's sensor table
    rom: str                    # DS18B20 ROM code as "28-8C-63-..." hex string
    roc_enabled: bool = False   # Enable/Disable Rate of Change tracking for the sensor
    offset: float = 0.0         # calibration offset in engineering units




def load_sensor_map(filename : str = "sensors.yaml") -> List[SensorMapping]:
    """
    Reads yaml config file for sensors and map to list of sensors
    """
    with open(filename, "r") as sensors_file:
        sensor_list = yaml.safe_load(sensors_file)

    sensors =  []
    for row in sensor_list:
        try:
            s = SensorMapping(**row) # unpack dict to SensorMapping
        except TypeError as e:
            raise ValueError(f"Error unpacking sensor mapping for {row}") from e
        sensors.append(s)
    sensors.sort(key=lambda m: m.slot)
    return sensors

