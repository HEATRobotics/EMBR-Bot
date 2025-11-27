"""
Hardware Abstraction Layer for EMBR sensors.
Provides real and simulated sensor implementations for testing.
"""

from .base import Sensor, SensorConfig
from .factory import SensorFactory, create_sensor
from .temperature import TemperatureSensor, RealTemperatureSensor, SimTemperatureSensor
from .cube import CubeSensor, RealCubeSensor, SimCubeSensor
from .radio import RadioConnection, RealRadioConnection, SimRadioConnection

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
    'RadioConnection',
    'RealRadioConnection',
    'SimRadioConnection',
]
