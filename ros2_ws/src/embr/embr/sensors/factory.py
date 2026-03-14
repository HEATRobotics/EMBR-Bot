"""Factory for creating sensor instances based on configuration."""

import os
import json
from typing import Dict, Any, Optional
from pathlib import Path

from .base import Sensor, SensorConfig
from .temperature import RealTemperatureSensor, SimTemperatureSensor
from .cube import RealCubeSensor, SimCubeSensor
from .radio import RealRadioConnection, SimRadioConnection
from .probeMotor import ProbeMotorBase, RealProbeMotor, SimProbeMotor

class SensorFactory:
    """Factory for creating sensor instances."""
    
    # Mapping of sensor types to their real/sim implementations
    SENSOR_MAP = {
        'temperature': {
            'real': RealTemperatureSensor,
            'sim': SimTemperatureSensor,
        },
        'cube': {
            'real': RealCubeSensor,
            'sim': SimCubeSensor,
        },
        'radio': {
            'real': RealRadioConnection,
            'sim': SimRadioConnection,
        },
        'probeMotor' : {
            'real' : RealProbeMotor,
            'sim': SimProbeMotor,
        }
    }
    
    @classmethod
    def create(cls, sensor_type: str, config_path: Optional[str] = None):
        """
        Create a sensor instance.
        
        Args:
            sensor_type: Type of sensor ('temperature', 'cube', 'radio', 'probeMotor)
            config: Sensor configuration
        
        Returns:
            Sensor instance
        
        Raises:
            ValueError: If sensor type is unknown
            RuntimeError: If sensor creation fails
        """
        if sensor_type not in cls.SENSOR_MAP:
            raise ValueError(f"Unknown sensor type: {sensor_type}")

        
        config = cls.load_config(sensor_type, config_path)
        
        # Determine mode - default to 'real' unless specified otherwise
        mode = cls._determine_mode(sensor_type, config)
        
        # Get the appropriate class
        sensor_classes = cls.SENSOR_MAP[sensor_type]
        sensor_class = sensor_classes.get(mode)

        if not sensor_class:
            raise ValueError(f"Unknown mode '{mode}' for sensor '{sensor_type}'")
        
        # Create and return instance
        try:
            return sensor_class(config)
        except Exception as e:
            raise RuntimeError(f"Failed to create {sensor_type} sensor in {mode} mode: {e}")
    
    @classmethod
    def _determine_mode(cls, sensor_type: str, config: SensorConfig) -> str:
        """
        Determine the sensor mode based on configuration.
        
        Priority order:
        1. Explicit mode in config (if 'real' or 'sim')
        2. Default to 'real'
        
        Returns:
            'real' or 'sim'
        """
        # If mode is explicitly set to 'real' or 'sim', use it
        if config.mode in ('real', 'sim'):
            return config.mode
        
        # Default to real hardware
        return 'real'
    
    @classmethod
    def load_config(cls, device_type: str ,config_path: Optional[str] = None) -> SensorConfig:
        """
        Load sensor configuration from JSON file.
        
        Args:
            config_path: Path to config file (default: config/sensors.json in workspace)
        
        Returns:
            Dictionary mapping sensor names to SensorConfig objects
        """

        if not config_path:
            # Look for config in common locations within workspace
            search_paths = [
                'src/embr/config/sensors.json',
                'sensors.json',
                os.path.expanduser('~/.embr/sensors.json'),
                '/etc/embr/sensors.json',
            ]
            
            for path in search_paths:
                if os.path.exists(path):
                    config_path = path
                    break

        if config_path is None or not os.path.exists(config_path):
            raise RuntimeError(f"Failed to find configuration file: cannot find configuration file from {config_path} or default config paths")
        
        try:
            with open(config_path, 'r') as f:
                data = json.load(f)
                return SensorConfig(**data[device_type])
        
        except Exception as e:
            raise RuntimeError(f"Failed to load config from {config_path}: {e}")


def create_sensor(sensor_type: str, config: Optional[SensorConfig] = None) -> Sensor:
    """
    Convenience function to create a sensor.
    
    Args:
        sensor_type: Type of sensor
        config: Optional sensor configuration
    
    Returns:
        Sensor instance
    """
    return SensorFactory.create(sensor_type, config)