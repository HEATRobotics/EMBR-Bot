"""Thermal camera sensor implementations."""

import time
import numpy as np
from typing import Optional, Dict, Any
from .base import Sensor, SensorConfig
import cv2


class ThermalCameraSensor(Sensor):
    """Abstract thermal camera sensor interface."""


class RealThermalCameraSensor(ThermalCameraSensor):
    """Real thermal camera sensor using Lepton via flirpy."""
    
    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        self.camera = None
        self.lepton_model = config.params.get('model', '3.1R') if config else '3.1R'
        camera_parameters = { 'camera matrix': [[104.65403680863373, 0.0, 79.12313258957062],
                                                      [0.0, 104.48251047202757, 55.689070170705634],
                                                      [0.0, 0.0, 1.0]],
                                    'distortion coeff': [[-0.39758308581607127,
                                                          0.18068641745671193,
                                                          0.004626461618389028,
                                                          0.004197358204037882,
                                                          -0.03381399499591463]],
                                    'new camera matrix':[[66.54581451416016, 0.0, 81.92717558174809],
                                                             [0.0, 64.58526611328125, 56.23740168870427], 
                                                             [0.0, 0.0, 1.0]]}
        self.camera_matrix = np.array(camera_parameters.get('camera matrix'))
        self.distortion_coeff = np.array(camera_parameters.get('distortion coeff'))
        self.new_camera_matrix = np.array(camera_parameters.get('new camera matrix'))

        
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
    
    def get_undistorted_img(self, img):
        '''
            Undistort the image

            Args:
                img = numpy array,
                    distorted image in uint16
            
            Output:
                undistorted_img = numpy array,
                    corrected image, cropped
        '''

        # Keep all pixels from input after dewarp
        undistorted_img = cv2.undistort(img, self.camera_matrix,
                                        self.distortion_coeff,
                                        None,
                                        self.new_camera_matrix)
        # Get image dimension
        img_dim = undistorted_img.shape
        row = img_dim[0]
        col = img_dim[1]

        # OpenCV generated cropping matrix still retains a few black pixels,
        # return the corrected image with those pixels cropped out
        undistorted_img = undistorted_img[14:row-18, 12:col-12]        
        return undistorted_img

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
    
    def stop(self) -> None:
        """Stop simulated camera."""
        self._running = False
