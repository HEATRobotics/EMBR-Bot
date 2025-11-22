"""Thermal camera sensor implementations."""

import time
import math
from typing import Optional, Dict, Any
from dataclasses import dataclass
from .base import Sensor, SensorConfig


@dataclass
class BoundingBox:
    """Bounding box data structure."""
    x1: int
    y1: int
    x2: int
    y2: int
    x_center: int
    y_center: int
    dx: float
    dy: float
    angle_degrees: float


class ThermalSensor(Sensor):
    """Abstract thermal camera interface."""
    pass


class RealThermalSensor(ThermalSensor):
    """Real thermal camera using OpenCV."""
    
    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        params = config.params if config else {}
        self.camera_id = params.get('camera_id', 1)
        self.webcam_stream = None
    
    def start(self) -> None:
        """Open camera stream."""
        if self._running:
            return
        
        try:
            import cv2
            self.cv2 = cv2
            self.webcam_stream = cv2.VideoCapture(self.camera_id)
            if not self.webcam_stream.isOpened():
                raise RuntimeError(f"Could not open camera with ID {self.camera_id}")
            self._running = True
        except Exception as e:
            raise RuntimeError(f"Failed to open thermal camera: {e}")
    
    def read(self) -> tuple:
        """Read frame and detect hotspots. Returns (frame, all_boxes, largest_box)."""
        if not self._running:
            raise RuntimeError("Sensor not started")
        
        import numpy as np
        
        ret, frame = self.webcam_stream.read()
        if not ret:
            raise RuntimeError("Could not read frame from camera")
        
        return self._process_frame(frame)
    
    def _process_frame(self, frame, nms_threshold=0.4):
        """Process frame to detect hotspots."""
        gray_frame = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2GRAY)
        blur_frame = self.cv2.GaussianBlur(gray_frame, (1, 1), 0)
        
        _, maxVal, _, _ = self.cv2.minMaxLoc(blur_frame)
        threshold_value = maxVal * 0.85
        _, thresh_frame = self.cv2.threshold(blur_frame, threshold_value, 255, self.cv2.THRESH_BINARY)
        
        contours, _ = self.cv2.findContours(thresh_frame, self.cv2.RETR_EXTERNAL, self.cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for cnt in contours:
            x, y, w, h = self.cv2.boundingRect(cnt)
            boxes.append([x, y, x + w, y + h])
        
        if len(boxes) == 0:
            return frame, [], {}
        
        import numpy as np
        boxes = np.array(boxes)
        
        indices = self.cv2.dnn.NMSBoxes(
            bboxes=boxes.tolist(),
            scores=[1.0] * len(boxes),
            score_threshold=0.0,
            nms_threshold=nms_threshold
        )
        
        all_boxes = []
        largest_area = 0
        largest_box = {}
        
        for i in indices:
            i = i[0] if isinstance(i, (list, tuple)) else i
            x1, y1, x2, y2 = boxes[i]
            self.cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 1)
            box_dict = {"x1": int(x1), "y1": int(y1), "x2": int(x2), "y2": int(y2)}
            all_boxes.append(box_dict)
            
            width = x2 - x1
            height = y2 - y1
            area = width * height
            if area > largest_area:
                largest_area = area
                x_center = x1 + width // 2
                y_center = y1 + height // 2
                frame_height, frame_width = frame.shape[:2]
                bottom_center_x = frame_width / 2
                bottom_center_y = frame_height
                dx = x_center - bottom_center_x
                dy = bottom_center_y - y_center
                angle_radians = math.atan2(dx, dy)
                angle_degrees = angle_radians * 180 / math.pi
                
                largest_box = {
                    "x1": int(x1),
                    "y1": int(y1),
                    "x2": int(x2),
                    "y2": int(y2),
                    "x_center": int(x_center),
                    "y_center": int(y_center),
                    "dx": float(dx),
                    "dy": float(dy),
                    "angle_degrees": float(angle_degrees)
                }
        
        return frame, all_boxes, largest_box
    
    def stop(self) -> None:
        """Release camera."""
        if self.webcam_stream and self.webcam_stream.isOpened():
            self.webcam_stream.release()
        self._running = False


class SimThermalSensor(ThermalSensor):
    """Simulated thermal camera with synthetic hotspots."""
    
    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        params = config.params if config else {}
        self.width = params.get('width', 640)
        self.height = params.get('height', 480)
        self.hotspot_count = params.get('hotspot_count', 1)
        self.moving = params.get('moving', True)
        self._start_time = 0.0
    
    def start(self) -> None:
        """Initialize simulated sensor."""
        self._start_time = time.time()
        self._running = True
    
    def read(self) -> tuple:
        """Generate simulated frame with hotspots."""
        if not self._running:
            raise RuntimeError("Sensor not started")
        
        import numpy as np
        
        # Create blank frame
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        elapsed = time.time() - self._start_time
        all_boxes = []
        largest_box = {}
        largest_area = 0
        
        for i in range(self.hotspot_count):
            if self.moving:
                # Moving hotspot in circular pattern
                angle = elapsed / 5.0 + (i * 2 * math.pi / self.hotspot_count)
                radius = min(self.width, self.height) / 4
                cx = int(self.width / 2 + radius * math.cos(angle))
                cy = int(self.height / 2 + radius * math.sin(angle))
            else:
                # Static hotspot
                cx = self.width // 2
                cy = self.height // 2
            
            # Draw hotspot
            size = 30
            x1 = max(0, cx - size)
            y1 = max(0, cy - size)
            x2 = min(self.width, cx + size)
            y2 = min(self.height, cy + size)
            
            # Draw on frame
            import cv2
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), -1)
            
            box = {
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
                "x_center": cx,
                "y_center": cy
            }
            all_boxes.append(box)
            
            area = (x2 - x1) * (y2 - y1)
            if area > largest_area:
                largest_area = area
                bottom_center_x = self.width / 2
                bottom_center_y = self.height
                dx = cx - bottom_center_x
                dy = bottom_center_y - cy
                angle_radians = math.atan2(dx, dy)
                angle_degrees = angle_radians * 180 / math.pi
                
                largest_box = {
                    **box,
                    "dx": float(dx),
                    "dy": float(dy),
                    "angle_degrees": float(angle_degrees)
                }
        
        return frame, all_boxes, largest_box
    
    def stop(self) -> None:
        """Stop simulated sensor."""
        self._running = False
