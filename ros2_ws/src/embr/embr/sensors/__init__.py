"""
Hardware Abstraction Layer for EMBR sensors.
Provides real and simulated sensor implementations for testing.
"""

from .base import Sensor, SensorConfig
from .factory import SensorFactory, create_sensor
from .temperature import TemperatureSensor, RealTemperatureSensor, SimTemperatureSensor
from .cube import CubeSensor, RealCubeSensor, SimCubeSensor
from .thermal import ThermalSensor, RealThermalSensor, SimThermalSensor
from .mavlink import MavlinkConnection, RealMavlinkConnection, SimMavlinkConnection

__all__ = [
    'Sensor',
    'SensorConfig',
    'SensorFactory',
    'create_sensor',
    'TemperatureSensor',
    'RealTemperatureSensor',
    'SimTemperatureSensor',
    'CubeSensor',
    'RealCubeSensor',
    'SimCubeSensor',
    'ThermalSensor',
    'RealThermalSensor',
    'SimThermalSensor',
    'MavlinkConnection',
    'RealMavlinkConnection',
    'SimMavlinkConnection',
]
