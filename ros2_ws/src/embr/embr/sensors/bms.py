from .factory import SensorFactory

class BMS_Handler:
    """Abstract BMS Handler Interface."""
    pass

class RealBMSSensor(BMS_Handler):
    """Real BMS sensor implementation."""
    
    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        self.device = config.device if config else '/dev/ttyACM1'
        self.baud = config.baud if config else 9600
        self.connection = None
    
    def start(self) -> None:
        """Open BMS connection."""
        if self.connection and self.connection.is_open:
            return
        
        try:
            import serial
            self.connection = serial.Serial(port=self.device, baudrate=self.baud, timeout=1)
        except Exception as e:
            raise RuntimeError(f"Failed to open BMS connection on {self.device}: {e}")
    
    def read(self) -> None:
        """Read BMS data (non-blocking)."""

        if not self.connection or not self.connection.is_open:
            raise RuntimeError("Connection not started")
        
        try:
            line = self.connection.readline().decode('utf-8').strip()
            return line  # Return raw data for now; parsing can be added later
        except Exception as e:
            raise RuntimeError(f"Failed to read from BMS: {e}")
    
    def stop(self):
        """Close BMS connection."""
        if self.connection and self.connection.is_open:
            self.connection.close()

class SimBMSSensor(BMS_Handler):
    """Simulated BMS sensor implementation."""
    
    def __init__(self, config):
        self.sim_data = config.sim_data if config else ["Simulated BMS Data"]
        self.index = 0
    
    def start(self):
        """Start simulated BMS (no-op)."""
        pass
    
    def read(self):
        """Return simulated BMS data."""
        data = self.sim_data[self.index]
        self.index = (self.index + 1) % len(self.sim_data)
        return data
    
    def stop(self):
        """Stop simulated BMS (no-op)."""
        pass