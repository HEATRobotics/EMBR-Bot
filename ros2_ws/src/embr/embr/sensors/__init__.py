"""
Hardware Abstraction Layer for EMBR sensors.
Provides real and simulated sensor implementations for testing.
"""

from .base import Sensor, SensorConfig


__all__ = [
    "Sensor",
    "SensorConfig",
    "SensorFactory",
    "create_sensor",
    "TemperatureSensor",
    "RealTemperatureSensor",
    "SimTemperatureSensor",
    "CubeSensor",
    "RealCubeSensor",
    "SimCubeSensor",
    "RadioConnection",
    "RealRadioConnection",
    "SimRadioConnection",
    "RealProbeMotor",
    "SimProbeMotor",
    "BMS_Handler",
    "RealBMSSensor",
    "SimBMSSensor",
]


def __getattr__(name):
    """Load sensor implementations only when they are requested."""
    if name in {"SensorFactory", "create_sensor"}:
        from .factory import SensorFactory, create_sensor

        return {"SensorFactory": SensorFactory, "create_sensor": create_sensor}[name]

    if name in {"TemperatureSensor", "RealTemperatureSensor", "SimTemperatureSensor"}:
        from .temperature import (
            RealTemperatureSensor,
            SimTemperatureSensor,
            TemperatureSensor,
        )

        return {
            "TemperatureSensor": TemperatureSensor,
            "RealTemperatureSensor": RealTemperatureSensor,
            "SimTemperatureSensor": SimTemperatureSensor,
        }[name]

    if name in {"CubeSensor", "RealCubeSensor", "SimCubeSensor"}:
        from .cube import CubeSensor, RealCubeSensor, SimCubeSensor

        return {
            "CubeSensor": CubeSensor,
            "RealCubeSensor": RealCubeSensor,
            "SimCubeSensor": SimCubeSensor,
        }[name]

    if name in {"RadioConnection", "RealRadioConnection", "SimRadioConnection"}:
        from .radio import RadioConnection, RealRadioConnection, SimRadioConnection

        return {
            "RadioConnection": RadioConnection,
            "RealRadioConnection": RealRadioConnection,
            "SimRadioConnection": SimRadioConnection,
        }[name]

    if name in {"RealProbeMotor", "SimProbeMotor"}:
        from .probeMotor import RealProbeMotor, SimProbeMotor

        return {
            "RealProbeMotor": RealProbeMotor,
            "SimProbeMotor": SimProbeMotor,
        }[name]

    if name in {"BMS_Handler", "RealBMSSensor", "SimBMSSensor"}:
        from .bms import BMS_Handler, RealBMSSensor, SimBMSSensor

        return {
            "BMS_Handler": BMS_Handler,
            "RealBMSSensor": RealBMSSensor,
            "SimBMSSensor": SimBMSSensor,
        }[name]

    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
