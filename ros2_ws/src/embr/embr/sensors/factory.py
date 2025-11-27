"""Factory for creating sensor instances based on configuration."""

import os
import json
from typing import Dict, Any, Optional
from pathlib import Path

from .base import Sensor, SensorConfig
from .temperature import RealTemperatureSensor, SimTemperatureSensor
from .cube import RealCubeSensor, SimCubeSensor
from .radio import RealRadioConnection, SimRadioConnection


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
    }
    
    @classmethod
    def create(cls, sensor_type: str, config: Optional[SensorConfig] = None) -> Sensor:
        """
        Create a sensor instance.
        
        Args:
            sensor_type: Type of sensor ('temperature', 'cube', 'radio')
            config: Sensor configuration
        
        Returns:
            Sensor instance
        
        Raises:
            ValueError: If sensor type is unknown
            RuntimeError: If sensor creation fails
        """
        if sensor_type not in cls.SENSOR_MAP:
            raise ValueError(f"Unknown sensor type: {sensor_type}")
        
        config = config or SensorConfig()
        
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
    def load_config(cls, config_path: Optional[str] = None) -> Dict[str, SensorConfig]:
        """
        Load sensor configuration from JSON file.
        
        Args:
            config_path: Path to config file (default: config/sensors.json in workspace)
        
        Returns:
            Dictionary mapping sensor names to SensorConfig objects
        """
        if config_path is None:
            # Look for config in common locations within workspace
            search_paths = [
                'config/sensors.json',
                'sensors.json',
                os.path.expanduser('~/.embr/sensors.json'),
                '/etc/embr/sensors.json',
            ]
            
            for path in search_paths:
                if os.path.exists(path):
                    config_path = path
                    break
        
        if config_path is None or not os.path.exists(config_path):
            return {}
        
        try:
            with open(config_path, 'r') as f:
                data = json.load(f)
            
            configs = {}
            for sensor_name, sensor_data in data.items():
                # Skip non-sensor entries (like comments or metadata)
                if not isinstance(sensor_data, dict):
                    continue
                
                # Get mode from config, default to 'real'
                mode = sensor_data.get('mode', 'real')
                
                configs[sensor_name] = SensorConfig(
                    mode=mode,
                    device=sensor_data.get('device'),
                    baud=sensor_data.get('baud', 9600),
                    params=sensor_data.get('params', {})
                )
            
            return configs
        
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
