"""Base sensor interface and configuration."""

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional
from dataclasses import dataclass, field


@dataclass
class SensorConfig:
    """Configuration for a sensor instance."""
    mode: str = "auto"  # auto, real, sim
    device: Optional[str] = None
    baud: int = 9600
    params: Dict[str, Any] = field(default_factory=dict)


class Sensor(ABC):
    """Abstract base class for all sensors."""
    
    def __init__(self, config: Optional[SensorConfig] = None):
        self.config = config or SensorConfig()
        self._running = False
    
    @abstractmethod
    def start(self) -> None:
        """Initialize and start the sensor."""
        pass
    
    @abstractmethod
    def read(self) -> Any:
        """Read data from the sensor. Return type depends on sensor."""
        pass
    
    @abstractmethod
    def stop(self) -> None:
        """Stop and cleanup the sensor."""
        pass
    
    @property
    def is_running(self) -> bool:
        """Check if sensor is running."""
        return self._running
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
        return False
