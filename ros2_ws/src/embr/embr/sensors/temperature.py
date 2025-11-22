"""Temperature sensor implementations."""

import time
import math
from typing import Optional
from .base import Sensor, SensorConfig


class TemperatureSensor(Sensor):
    """Abstract temperature sensor interface."""
    pass


class RealTemperatureSensor(TemperatureSensor):
    """Real temperature sensor reading from Arduino via serial."""
    
    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        self.ser = None
        self.device = config.device if config else '/dev/ttyACM0'
        self.baud = config.baud if config else 9600
    
    def start(self) -> None:
        """Open serial connection to temperature sensor."""
        if self._running:
            return
        
        try:
            import serial
            import re
            self.serial = serial
            self.re = re
            self.ser = serial.Serial(self.device, self.baud)
            time.sleep(2)  # Wait for Arduino to initialize
            self._running = True
        except Exception as e:
            raise RuntimeError(f"Failed to open temperature sensor on {self.device}: {e}")
    
    def read(self) -> float:
        """Read temperature from serial port."""
        if not self._running:
            raise RuntimeError("Sensor not started")
        
        try:
            raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
            match = self.re.search(r"[-+]?\d*\.\d+|\d+", raw)
            if match:
                return float(match.group())
            return 0.0
        except Exception as e:
            raise RuntimeError(f"Failed to read temperature: {e}")
    
    def stop(self) -> None:
        """Close serial connection."""
        if self.ser:
            self.ser.close()
        self._running = False


class SimTemperatureSensor(TemperatureSensor):
    """Simulated temperature sensor with realistic variations."""
    
    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        params = config.params if config else {}
        self.base_temp = params.get('base_temp', 22.0)
        self.variation = params.get('variation', 2.0)
        self.noise = params.get('noise', 0.1)
        self._start_time = 0.0
    
    def start(self) -> None:
        """Initialize simulated sensor."""
        self._start_time = time.time()
        self._running = True
    
    def read(self) -> float:
        """Generate simulated temperature with sine wave and noise."""
        if not self._running:
            raise RuntimeError("Sensor not started")
        
        elapsed = time.time() - self._start_time
        # Sine wave for gradual changes
        sine_component = self.variation * math.sin(elapsed / 10.0)
        # Random noise
        import random
        noise_component = random.uniform(-self.noise, self.noise)
        
        return self.base_temp + sine_component + noise_component
    
    def stop(self) -> None:
        """Stop simulated sensor."""
        self._running = False
