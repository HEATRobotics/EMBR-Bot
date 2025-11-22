"""Factory for creating sensor instances based on configuration."""

import os
import json
from typing import Dict, Any, Optional
from pathlib import Path

from .base import Sensor, SensorConfig
from .temperature import RealTemperatureSensor, SimTemperatureSensor
from .cube import RealCubeSensor, SimCubeSensor
from .thermal import RealThermalSensor, SimThermalSensor
from .mavlink import RealMavlinkConnection, SimMavlinkConnection


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
        'thermal': {
            'real': RealThermalSensor,
            'sim': SimThermalSensor,
        },
        'mavlink': {
            'real': RealMavlinkConnection,
            'sim': SimMavlinkConnection,
        },
    }
    
    @classmethod
    def create(cls, sensor_type: str, config: Optional[SensorConfig] = None) -> Sensor:
        """
        Create a sensor instance.
        
        Args:
            sensor_type: Type of sensor ('temperature', 'cube', 'thermal', 'mavlink')
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
        
        # Determine mode
        mode = config.mode
        if mode == "auto":
            mode = cls._probe_hardware(sensor_type, config)
        
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
    def _probe_hardware(cls, sensor_type: str, config: SensorConfig) -> str:
        """
        Probe for hardware availability.
        
        Returns:
            'real' if hardware is available, 'sim' otherwise
        """
        # Check environment variable override
        env_mode = os.getenv('EMBR_SENSOR_MODE', '').lower()
        if env_mode in ('real', 'sim'):
            return env_mode
        
        # Check for specific sensor environment variable
        sensor_env = os.getenv(f'EMBR_{sensor_type.upper()}_MODE', '').lower()
        if sensor_env in ('real', 'sim'):
            return sensor_env
        
        # Attempt hardware probe
        try:
            if sensor_type == 'temperature':
                device = config.device or '/dev/ttyACM0'
                return 'real' if os.path.exists(device) else 'sim'
            
            elif sensor_type == 'cube':
                device = config.device or '/dev/ttyAMA0'
                return 'real' if os.path.exists(device) else 'sim'
            
            elif sensor_type == 'thermal':
                # Try to import cv2 and check camera
                import cv2
                camera_id = config.params.get('camera_id', 1) if config.params else 1
                cap = cv2.VideoCapture(camera_id)
                available = cap.isOpened()
                cap.release()
                return 'real' if available else 'sim'
            
            elif sensor_type == 'mavlink':
                device = config.device or '/dev/ttyAMA1'
                return 'real' if os.path.exists(device) else 'sim'
            
        except Exception:
            pass
        
        # Default to simulation
        return 'sim'
    
    @classmethod
    def load_config(cls, config_path: Optional[str] = None) -> Dict[str, SensorConfig]:
        """
        Load sensor configuration from JSON file.
        
        Args:
            config_path: Path to config file (default: sensors.json in workspace)
        
        Returns:
            Dictionary mapping sensor names to SensorConfig objects
        """
        if config_path is None:
            # Look for config in common locations
            search_paths = [
                'sensors.json',
                'config/sensors.json',
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
                configs[sensor_name] = SensorConfig(
                    mode=sensor_data.get('mode', 'auto'),
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
