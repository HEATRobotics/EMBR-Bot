"""MAVLink connection implementations."""

import time
from typing import Optional, Dict, Any
from .base import Sensor, SensorConfig


class MavlinkConnection(Sensor):
    """Abstract MAVLink connection interface."""
    pass


class RealMavlinkConnection(MavlinkConnection):
    """Real MAVLink serial connection."""
    
    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        self.device = config.device if config else '/dev/ttyAMA1'
        self.baud = config.baud if config else 57600
        self.connection = None
        self.mav = None
    
    def start(self) -> None:
        """Open MAVLink connection."""
        if self._running:
            return
        
        try:
            from pymavlink import mavutil
            from pymavlink.dialects.v20 import common as mavlink2
            
            self.connection = mavutil.mavserial(device=self.device, baud=self.baud)
            self.mav = mavlink2.MAVLink(self.connection)
            self._running = True
        except Exception as e:
            raise RuntimeError(f"Failed to open MAVLink connection on {self.device}: {e}")
    
    def read(self) -> Optional[Any]:
        """Read MAVLink message (non-blocking)."""
        if not self._running:
            raise RuntimeError("Connection not started")
        
        return self.connection.recv_match(blocking=False)
    
    def send_temperature(self, temperature: float) -> None:
        """Send temperature via MAVLink."""
        if not self._running:
            raise RuntimeError("Connection not started")
        
        timems = int((time.time() - time.mktime(time.gmtime(0))) * 1000) % 4294967296
        self.mav.named_value_float_send(
            time_boot_ms=timems,
            name=b'temp',
            value=temperature
        )
    
    def send_gps(self, lat: int, lon: int, alt: int, vel: int) -> None:
        """Send GPS data via MAVLink."""
        if not self._running:
            raise RuntimeError("Connection not started")
        
        timems = int((time.time() - time.mktime(time.gmtime(0))) * 1000) % 4294967296
        self.mav.global_position_int_send(timems, lat, lon, alt, 0, vel, 0, 0, 0)
    
    def stop(self) -> None:
        """Close MAVLink connection."""
        if self.connection:
            self.connection.close()
        self._running = False


class SimMavlinkConnection(MavlinkConnection):
    """Simulated MAVLink connection for testing."""
    
    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        self.sent_messages = []
        self.received_messages = []
    
    def start(self) -> None:
        """Initialize simulated connection."""
        self.sent_messages = []
        self.received_messages = []
        self._running = True
    
    def read(self) -> Optional[Dict[str, Any]]:
        """Read from simulated message queue."""
        if not self._running:
            raise RuntimeError("Connection not started")
        
        if self.received_messages:
            return self.received_messages.pop(0)
        return None
    
    def send_temperature(self, temperature: float) -> None:
        """Record sent temperature."""
        if not self._running:
            raise RuntimeError("Connection not started")
        
        self.sent_messages.append({
            'type': 'temperature',
            'value': temperature,
            'time': time.time()
        })
    
    def send_gps(self, lat: int, lon: int, alt: int, vel: int) -> None:
        """Record sent GPS data."""
        if not self._running:
            raise RuntimeError("Connection not started")
        
        self.sent_messages.append({
            'type': 'gps',
            'lat': lat,
            'lon': lon,
            'alt': alt,
            'vel': vel,
            'time': time.time()
        })
    
    def inject_message(self, message: Dict[str, Any]) -> None:
        """Inject a message to be read (for testing)."""
        self.received_messages.append(message)
    
    def stop(self) -> None:
        """Stop simulated connection."""
        self._running = False
