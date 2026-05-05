"""Radio connection implementations."""

import time
from typing import Optional, Dict, Any
from .base import Sensor, SensorConfig

import struct

class RadioConnection(Sensor):
    """Abstract Radio connection interface."""
    pass


class RealRadioConnection(RadioConnection):
    """Real Radio serial connection."""
    
    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        self.device = config.device if config else '/dev/serial0'
        self.baud =  config.baud if config else 57600
        self.connection = None
        self.mav = None
    
    def start(self) -> None:
        """Open Radio connection."""
        if self._running:
            return
        
        try:
            from pymavlink import mavutil
            from pymavlink.dialects.v20 import common as mavlink2
            
            self.connection = mavutil.mavserial(device=self.device, baud=self.baud)
            self.mav = mavlink2.MAVLink(self.connection)
            self._running = True
        except Exception as e:
            raise RuntimeError(f"Failed to open Radio connection on {self.device}: {e}")
    
    def read(self, msg_type = None, is_blocking = False) -> Optional[Any]:
        """Read Radio message (non-blocking)."""
        if not self._running:
            raise RuntimeError("Connection not started")
        if type:
            return self.connection.recv_match(type=msg_type, blocking=is_blocking)
        else:
            return self.connection.recv_match(blocking=is_blocking)
    
    def read_mission_data(self):
        mission_data = self.read('LOGGING_DATA_ACKED',True)
        self.mav.logging_ack_send(
            target_system=1,
            target_component=0,
            sequence=0
        )

        byte_array = bytes(mission_data.data[:36]) 
        
        # Format: i (1 int), 4i (4 ints), 4i (4 ints)
        # Total: 9 integers * 4 bytes = 36 bytes
        unpacked = struct.unpack('<i4i4i', byte_array)
        
        num_temp_readings = unpacked[0]
        decoded_lats = [lat / 1e7 for lat in unpacked[1:5]]
        decoded_lons = [lon / 1e7 for lon in unpacked[5:9]]
        
        return num_temp_readings, decoded_lats, decoded_lons
    
    def send_temperature(self, temperature: float) -> None:
        """Send temperature via Radio."""
        if not self._running:
            raise RuntimeError("Connection not started")
        
        timems = int((time.time() - time.mktime(time.gmtime(0))) * 1000) % 4294967296
        self.mav.named_value_float_send(
            time_boot_ms=timems,
            name=b'temp',
            value=temperature
        )
    
    def send_gps(self, lat: int, lon: int, alt: int, vel: int) -> None:
        """Send GPS data via Radio."""
        if not self._running:
            raise RuntimeError("Connection not started")
        
        timems = int((time.time() - time.mktime(time.gmtime(0))) * 1000) % 4294967296
        self.mav.global_position_int_send(timems, lat, lon, alt, 0, vel, 0, 0, 0)
    
    def stop(self) -> None:
        """Close Radio connection."""
        if self.connection:
            self.connection.close()
        self._running = False


class SimRadioConnection(RadioConnection):
    """Simulated Radio connection for testing."""
    
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
