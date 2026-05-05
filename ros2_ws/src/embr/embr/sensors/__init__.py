"""
Hardware Abstraction Layer for EMBR sensors.
Provides real and simulated sensor implementations for testing.
"""

from .base import Sensor, SensorConfig
from .factory import SensorFactory, create_sensor
from .temperature import TemperatureSensor, RealTemperatureSensor, SimTemperatureSensor
from .cube import CubeSensor, RealCubeSensor, SimCubeSensor
from .radio import RadioConnection, RealRadioConnection, SimRadioConnection
from .probeMotor import RealProbeMotor, SimProbeMotor
from .bms import BMS_Handler, RealBMSSensor, SimBMSSensor

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
    'RealProbeMotor', 
    'SimProbeMotor',
    'BMS_Handler',
    'RealBMSSensor',
    'SimBMSSensor'
]
