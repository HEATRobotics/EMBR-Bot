"""Thermal camera sensor implementations."""

import time
import numpy as np
from typing import Optional, Dict, Any
from .base import Sensor, SensorConfig


class ThermalCameraSensor(Sensor):
    """Abstract thermal camera sensor interface."""
    
    def get_frame_shape(self) -> tuple:
        """Get the shape of thermal frames (height, width)."""
        raise NotImplementedError


class RealThermalCameraSensor(ThermalCameraSensor):
    """Real thermal camera sensor using Lepton via flirpy."""
    
    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        self.camera = None
        self.lepton_model = config.params.get('model', '3.1R') if config else '3.1R'
        
    def start(self) -> None:
        """Initialize Lepton camera."""
        if self._running:
            return
        
        try:
            from flirpy.camera.lepton import Lepton
            self.camera = Lepton()
            self._running = True
        except Exception as e:
            raise RuntimeError(f"Failed to initialize Lepton camera: {e}")
    
    def read(self) -> np.ndarray:
        """
        Read radiometric frame from camera.
        
        Returns:
            2D numpy array of uint16 values (Kelvin * 100)
        """
        if not self._running:
            raise RuntimeError("Sensor not started")
        
        try:
            frame = self.camera.grab()
            if frame is None:
                raise RuntimeError("Failed to grab frame from camera")
            return frame
        except Exception as e:
            raise RuntimeError(f"Failed to read thermal frame: {e}")
    
    def get_frame_shape(self) -> tuple:
        """Get the shape of thermal frames based on Lepton model."""
        if self.lepton_model == '2.5':
            return (60, 80)
        elif self.lepton_model == '3.1R':
            return (120, 160)
        else:
            # Default to 3.1R
            return (120, 160)
    
    def stop(self) -> None:
        """Close camera connection."""
        if self.camera:
            try:
                self.camera.close()
            except Exception:
                pass
        self._running = False


class SimThermalCameraSensor(ThermalCameraSensor):
    """Simulated thermal camera with realistic temperature patterns."""
    
    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        params = config.params if config else {}
        # Resolution derived from Lepton model
        model = params.get('model')
        self.width, self.height = self._model_default_dims(model)
        self.base_temp = params.get('base_temp', 22.0)  # Celsius
        self.temp_variation = params.get('temp_variation', 5.0)
        self.hotspot_temp = params.get('hotspot_temp', 40.0)
        self.num_hotspots = params.get('num_hotspots', 2)
        self._start_time = 0.0
        self._hotspot_positions = []

    @staticmethod
    def _model_default_dims(model: Optional[str]) -> tuple:
        """Return default (width, height) for given Lepton model name."""
        if model == '2.5':
            return (80, 60)
        # Treat anything else (including None, '3.1R') as 3.1R
        return (160, 120)
    
    def start(self) -> None:
        """Initialize simulated camera."""
        self._start_time = time.time()
        self._running = True
        # Generate random hotspot positions
        import random
        self._hotspot_positions = [
            (random.randint(20, self.width - 20), random.randint(20, self.height - 20))
            for _ in range(self.num_hotspots)
        ]
    
    def read(self) -> np.ndarray:
        """
        Generate simulated thermal frame with hotspots.
        
        Returns:
            2D numpy array of uint16 values (Kelvin * 100)
        """
        if not self._running:
            raise RuntimeError("Sensor not started")
        
        import random
        
        # Create base temperature field with some variation
        frame = np.random.uniform(
            self.base_temp - self.temp_variation,
            self.base_temp + self.temp_variation,
            (self.height, self.width)
        )
        
        # Add hotspots with Gaussian distribution
        for hx, hy in self._hotspot_positions:
            y, x = np.ogrid[:self.height, :self.width]
            # Create Gaussian hotspot
            sigma = 10.0
            hotspot = np.exp(-((x - hx)**2 + (y - hy)**2) / (2 * sigma**2))
            frame += hotspot * (self.hotspot_temp - self.base_temp)
        
        # Add temporal variation (simulated movement/flicker)
        elapsed = time.time() - self._start_time
        temporal_noise = np.sin(elapsed) * 2.0
        frame += temporal_noise
        
        # Convert to Kelvin * 100 format (same as Lepton output)
        frame_kelvin = (frame + 273.15) * 100.0
        frame_uint16 = np.clip(frame_kelvin, 0, 65535).astype(np.uint16)
        
        # Simulate camera frame rate delay
        time.sleep(1.0 / 9.0)  # ~9 FPS like Lepton
        
        return frame_uint16
    
    def get_frame_shape(self) -> tuple:
        """Get the shape of simulated thermal frames."""
        return (self.height, self.width)
    
    def stop(self) -> None:
        """Stop simulated camera."""
        self._running = False
