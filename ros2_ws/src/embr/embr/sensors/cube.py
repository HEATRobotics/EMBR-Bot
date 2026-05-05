"""Cube Orange flight controller sensor implementations."""

import time
import math
from typing import Optional, Dict, Any
from dataclasses import dataclass
from math import degrees
from .base import Sensor, SensorConfig


@dataclass
class GpsData:
    """GPS data structure with optional IMU fields."""
    lat: float  # latitude in degrees
    lon: float  # longitude in degrees
    alt: float  # altitude in meters
    vel: float  # ground speed in m/s
    # IMU fields in degrees
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0


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
            attitude = self.vehicle.attitude
            yaw = attitude.yaw
            pitch = attitude.pitch
            roll = attitude.roll
            # Return floats: lat/lon in degrees, alt in meters, vel in m/s
            return GpsData(
                lat=float(location.lat),
                lon=float(location.lon),
                alt=float(location.alt),
                vel=float(self.vehicle.groundspeed),
                pitch=float(degrees(pitch)),
                yaw=float(degrees(yaw)),
                roll=float(degrees(roll)),
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
        params = config["params"] if config else {}
        # Starting position (default: somewhere in US)
        self.start_lat = params.get('start_lat', 37.7749)
        self.start_lon = params.get('start_lon', -122.4194)
        self.start_alt = params.get('start_alt', 100.0)  # meters
        self.velocity = params.get('velocity', 5.0)  # m/s
        self.pattern = params.get('pattern', 'circle')  # circle, line, hover
        self.pause_interval = params.get('pause_interval', 10.0)  # seconds between pauses
        self.pause_duration = params.get('pause_duration', 5.0)  # seconds to pause
        self._start_time = 0.0
        self._last_pause_time = 0.0
        self._pause_position = None  # Store position during pause
    
    def start(self) -> None:
        """Initialize simulated sensor."""
        self._start_time = time.time()
        self._last_pause_time = self._start_time
        self._running = True
    
    def read(self) -> GpsData:
        """Generate simulated GPS data with movement patterns and random pauses."""
        if not self._running:
            raise RuntimeError("Sensor not started")
        
        current_time = time.time()
        elapsed = current_time - self._start_time
        time_since_last_pause = current_time - self._last_pause_time
        
        # Check if we should pause
        if time_since_last_pause >= self.pause_interval and time_since_last_pause < (self.pause_interval + self.pause_duration):
            # Currently paused - return stored position with zero velocity
            if self._pause_position is None:
                # First read during pause - store current position
                lat, lon, alt = self._calculate_position(elapsed)
                self._pause_position = (lat, lon, alt)
            else:
                lat, lon, alt = self._pause_position
            
            return GpsData(
                lat=float(lat),
                lon=float(lon),
                alt=float(alt),
                vel=0.0,  # Zero velocity during pause
                pitch=0.0,
                yaw=0.0,
                roll=0.0,
            )
        elif time_since_last_pause >= (self.pause_interval + self.pause_duration):
            # Pause ended - reset for next pause cycle
            self._last_pause_time = current_time
            self._pause_position = None
        
        # Normal movement
        lat, lon, alt = self._calculate_position(elapsed)
        
        return GpsData(
            lat=float(lat),
            lon=float(lon),
            alt=float(alt),
            vel=float(self.velocity),
            pitch=5.0 * math.sin(elapsed / 5.0),
            yaw=(elapsed * 10.0) % 360.0,
            roll=2.0 * math.sin(elapsed / 3.0),
        )
    
    def _calculate_position(self, elapsed: float) -> tuple:
        """Calculate position based on movement pattern."""
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
        
        return lat, lon, alt
    
    def stop(self) -> None:
        """Stop simulated sensor."""
        self._running = False
