"""Cube Orange flight controller sensor implementations."""

import time
import math
from typing import Optional, Dict, Any
from dataclasses import dataclass
from .base import Sensor, SensorConfig


@dataclass
class GpsData:
    """GPS data structure."""
    lat: int  # latitude * 1e7
    lon: int  # longitude * 1e7
    alt: int  # altitude in mm
    vel: float  # ground speed in m/s


class CubeSensor(Sensor):
    """Abstract Cube Orange sensor interface."""
    pass


class RealCubeSensor(CubeSensor):
    """Real Cube Orange sensor using DroneKit."""
    
    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        self.device = config.device if config else '/dev/ttyAMA0'
        self.baud = config.baud if config else 57600
        self.vehicle = None
    
    def start(self) -> None:
        """Connect to the Cube Orange."""
        if self._running:
            return
        
        try:
            from dronekit import connect
            self.vehicle = connect(self.device, wait_ready=False, baud=self.baud)
            self._running = True
        except Exception as e:
            raise RuntimeError(f"Failed to connect to Cube Orange on {self.device}: {e}")
    
    def read(self) -> GpsData:
        """Read GPS data from Cube Orange."""
        if not self._running:
            raise RuntimeError("Sensor not started")
        
        try:
            location = self.vehicle.location.global_frame
            return GpsData(
                lat=int(location.lat * 1e7),
                lon=int(location.lon * 1e7),
                alt=int(location.alt * 1000),
                vel=self.vehicle.groundspeed
            )
        except Exception as e:
            raise RuntimeError(f"Failed to read GPS data: {e}")
    
    def stop(self) -> None:
        """Disconnect from Cube Orange."""
        if self.vehicle:
            self.vehicle.close()
        self._running = False


class SimCubeSensor(CubeSensor):
    """Simulated Cube Orange sensor with realistic GPS movements."""
    
    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        params = config.params if config else {}
        # Starting position (default: somewhere in US)
        self.start_lat = params.get('start_lat', 37.7749)
        self.start_lon = params.get('start_lon', -122.4194)
        self.start_alt = params.get('start_alt', 100.0)  # meters
        self.velocity = params.get('velocity', 5.0)  # m/s
        self.pattern = params.get('pattern', 'circle')  # circle, line, hover
        self._start_time = 0.0
    
    def start(self) -> None:
        """Initialize simulated sensor."""
        self._start_time = time.time()
        self._running = True
    
    def read(self) -> GpsData:
        """Generate simulated GPS data with movement patterns."""
        if not self._running:
            raise RuntimeError("Sensor not started")
        
        elapsed = time.time() - self._start_time
        
        if self.pattern == 'circle':
            # Circle pattern
            radius = 0.0001  # ~11 meters
            angle = elapsed / 10.0
            lat = self.start_lat + radius * math.cos(angle)
            lon = self.start_lon + radius * math.sin(angle)
            alt = self.start_alt + 5.0 * math.sin(elapsed / 5.0)
        elif self.pattern == 'line':
            # Linear movement
            lat = self.start_lat + 0.00001 * elapsed
            lon = self.start_lon
            alt = self.start_alt
        else:  # hover
            # Hovering with small variations
            import random
            lat = self.start_lat + random.uniform(-0.000001, 0.000001)
            lon = self.start_lon + random.uniform(-0.000001, 0.000001)
            alt = self.start_alt + random.uniform(-0.5, 0.5)
        
        return GpsData(
            lat=int(lat * 1e7),
            lon=int(lon * 1e7),
            alt=int(alt * 1000),
            vel=self.velocity
        )
    
    def stop(self) -> None:
        """Stop simulated sensor."""
        self._running = False
