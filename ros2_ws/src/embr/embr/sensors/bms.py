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
    
    def read(self) -> bytes:
        """Read BMS data (non-blocking)."""

        if not self.connection or not self.connection.is_open:
            raise RuntimeError("Connection not started")
        
        try:
            # Request basic battery info (register 0x03)
            request = bytes([0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77])
            self.connection.write(request)

            # Read & Return response (should be 10 bytes)
            return self.connection.read(10)
        except Exception as e:
            raise RuntimeError(f"Failed to read from BMS: {e}")
    
    def stop(self):
        """Close BMS connection."""
        if self.connection and self.connection.is_open:
            self.connection.close()

class SimBMSSensor(BMS_Handler):
    """Simulated BMS sensor implementation."""
    
    def __init__(self, config):
        super().__init__(config)
        params = config["params"] if config else {}
        self.num_cells = params.get("num_cells", 6)
        self.cell_voltage_range = params.get("cell_voltage_range", (3.0, 4.2))
        self.current_range = params.get("current_range", (-100, 100))  # Amps
        self.temperature_range = params.get("temperature_range", (20, 40))  # Celsius
    
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