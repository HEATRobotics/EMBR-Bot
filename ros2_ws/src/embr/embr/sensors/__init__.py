"""
Hardware Abstraction Layer for EMBR sensors.
Provides real and simulated sensor implementations for testing.
"""

from .base import Sensor, SensorConfig
from .factory import SensorFactory, ProbeMotorFactory, create_sensor, create_probeMotor
from .temperature import TemperatureSensor, RealTemperatureSensor, SimTemperatureSensor
from .cube import CubeSensor, RealCubeSensor, SimCubeSensor
from .radio import RadioConnection, RealRadioConnection, SimRadioConnection
from .probeMotorBase import ProbeMotorConfig, RealProbeMotor, SimProbeMotor

__all__ = [
    'Sensor',
    'SensorConfig',
    'ProbeMotorConfig',
    'SensorFactory',
    'MotorFactory',
    'create_sensor',
    'create_probeMotor',
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
    'SimProbeMotorr'
]
