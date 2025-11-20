#!/usr/bin/env python3
"""
Thermal Camera Streaming Node with Temperature Overlay and Frame Publishing
Uses flirpy to capture Lepton thermal data, streams via HDMI with ffmpeg,
and publishes radiometric frames when vehicle velocity is 0.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension, MultiArrayLayout
from msg_interface.msg import GPSAndIMU
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess
import threading
import queue
from flirpy.camera.lepton import Lepton


class ThermalStreamNode(Node):
    def __init__(self):
        super().__init__('thermal_stream_node')
        
        # Configuration - can be modified here directly
        self.temp_threshold = 20.0  # Celsius - temperature threshold for hotspot detection
        self.video_fps = 9.0  # Lepton typical fps
        self.display_width = 160
        self.display_height = 120
        self.colormap = cv2.COLORMAP_JET
        self.min_temp = 10.0  # Celsius - minimum temperature for colormap scaling
        self.max_temp = 80.0  # Celsius - maximum temperature for colormap scaling
        
        # State variables
        self.current_velocity = None
        self.velocity_lock = threading.Lock()
        
        # Publishers
        self.frame_publisher = self.create_publisher(
            Image, 
            'thermal/radiometric_frame', 
            10
        )
        # Optional: publish radiometric array (uint16, Kelvin x 100) for direct analysis
        self.array_publisher = self.create_publisher(
            UInt16MultiArray,
            'thermal/radiometric_array',
            10
        )
        
        # Subscribers
        self.gps_subscription = self.create_subscription(
            GPSAndIMU,
            'gps',
            self.gps_callback,
            10
        )
        
        # CV Bridge for ROS image messages
        self.bridge = CvBridge()
        
        # Initialize camera
        self.get_logger().info('Initializing Lepton camera...')
        try:
            self.camera = Lepton()
            self.get_logger().info('Lepton camera initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize camera: {e}')
            raise
        
        # FFmpeg process for HDMI streaming
        self.ffmpeg_process = None
        self.frame_queue = queue.Queue(maxsize=2)
        
        # Start video streaming thread
        self.streaming_active = True
        self.stream_thread = threading.Thread(target=self._streaming_loop, daemon=True)
        self.stream_thread.start()
        
        # Start FFmpeg process
        self._start_ffmpeg()
        
        self.get_logger().info('Thermal stream node initialized')
    
    def gps_callback(self, msg):
        """Handle GPS messages and update velocity"""
        with self.velocity_lock:
            self.current_velocity = msg.vel
        
        # Publish frame and temperature array if velocity is ~0
        if abs(msg.vel) < 0.01:  # m/s threshold for stationary
            self._capture_and_publish()
    
    def _start_ffmpeg(self):
        """Start FFmpeg process for HDMI output"""
        # FFmpeg command to output to HDMI (framebuffer)
        # Adjust the output device based on your system configuration
        ffmpeg_cmd = [
            'ffmpeg',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{self.display_width}x{self.display_height}',
            '-r', str(self.video_fps),
            '-i', '-',  # Read from stdin
            '-f', 'fbdev',
            '-vf', 'scale=1920:1080',  # Scale to HDMI resolution
            '/dev/fb0'  # HDMI framebuffer device
        ]
        
        try:
            self.ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=10**8
            )
            self.get_logger().info('FFmpeg HDMI streaming started')
        except Exception as e:
            self.get_logger().error(f'Failed to start FFmpeg: {e}')
            # Fallback to OpenCV display if FFmpeg fails
            self.ffmpeg_process = None
    
    def _streaming_loop(self):
        """Main loop for capturing and processing thermal frames"""
        self.get_logger().info('Starting thermal streaming loop')
        
        while self.streaming_active and rclpy.ok():
            try:
                # Capture radiometric frame from camera
                radiometric_frame = self.camera.grab()
                
                if radiometric_frame is None:
                    self.get_logger().warn('Failed to grab frame from camera')
                    continue
                
                # Convert to Celsius (Lepton outputs in Kelvin * 100)
                temp_celsius = (radiometric_frame / 100.0) - 273.15
                
                # Create display frame with colormap and overlays
                display_frame = self._create_display_frame(temp_celsius)
                
                # Send to FFmpeg or display with OpenCV
                if self.ffmpeg_process is not None and self.ffmpeg_process.poll() is None:
                    try:
                        self.ffmpeg_process.stdin.write(display_frame.tobytes())
                    except BrokenPipeError:
                        self.get_logger().error('FFmpeg pipe broken')
                        self.ffmpeg_process = None
                else:
                    # Fallback: display with OpenCV
                    cv2.imshow('Thermal Stream', display_frame)
                    cv2.waitKey(1)
                
            except Exception as e:
                self.get_logger().error(f'Error in streaming loop: {e}')
                import traceback
                traceback.print_exc()
    
    def _create_display_frame(self, temp_celsius):
        """
        Create a color-mapped display frame with temperature overlays
        
        Args:
            temp_celsius: 2D numpy array of temperature values in Celsius
        
        Returns:
            BGR color image ready for display/streaming
        """
        # Normalize temperature to 0-255 range for colormap
        temp_normalized = np.clip(
            (temp_celsius - self.min_temp) / (self.max_temp - self.min_temp) * 255,
            0, 
            255
        ).astype(np.uint8)
        
        # Apply colormap
        colored_frame = cv2.applyColorMap(temp_normalized, self.colormap)
        
        # Resize for display
        display_frame = cv2.resize(
            colored_frame,
            (self.display_width, self.display_height),
            interpolation=cv2.INTER_LINEAR
        )
        
        # Find and annotate hot spots above threshold
        hot_mask = temp_celsius > self.temp_threshold
        
        if np.any(hot_mask):
            # Find contours of hot regions
            hot_mask_uint8 = hot_mask.astype(np.uint8) * 255
            contours, _ = cv2.findContours(
                hot_mask_uint8,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )
            
            # Scale factor for contours (from thermal resolution to display resolution)
            scale_x = self.display_width / temp_celsius.shape[1]
            scale_y = self.display_height / temp_celsius.shape[0]
            
            for contour in contours:
                # Calculate temperature statistics for this region
                mask_region = np.zeros_like(temp_celsius, dtype=np.uint8)
                cv2.drawContours(mask_region, [contour], -1, 1, -1)
                region_temps = temp_celsius[mask_region == 1]
                
                if len(region_temps) > 0:
                    max_temp = np.max(region_temps)
                    mean_temp = np.mean(region_temps)
                    
                    # Scale contour to display size
                    scaled_contour = contour.astype(np.float32)
                    scaled_contour[:, 0, 0] *= scale_x
                    scaled_contour[:, 0, 1] *= scale_y
                    scaled_contour = scaled_contour.astype(np.int32)
                    
                    # Draw contour
                    cv2.drawContours(display_frame, [scaled_contour], -1, (0, 255, 0), 2)
                    
                    # Get centroid for text placement
                    M = cv2.moments(scaled_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                    else:
                        cx, cy = scaled_contour[0][0]
                    
                    # Draw temperature text
                    temp_text = f"{max_temp:.1f}C"
                    cv2.putText(
                        display_frame,
                        temp_text,
                        (cx - 30, cy),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )
        
        # Add informational overlays
        self._add_info_overlay(display_frame, temp_celsius)
        
        return display_frame
    
    def _add_info_overlay(self, frame, temp_celsius):
        """Add informational text overlay to the frame"""
        # Overall temperature statistics
        min_temp = np.min(temp_celsius)
        max_temp = np.max(temp_celsius)
        mean_temp = np.mean(temp_celsius)
        
        # Timestamp
        timestamp = self.get_clock().now().to_msg()
        time_str = f"{timestamp.sec}.{timestamp.nanosec // 1000000:03d}"
        
        # Velocity info
        with self.velocity_lock:
            vel = self.current_velocity
        
        vel_str = f"Vel: {vel:.2f} m/s" if vel is not None else "Vel: N/A"
        
        # Create overlay text
        info_lines = [
            f"Min: {min_temp:.1f}C  Max: {max_temp:.1f}C  Avg: {mean_temp:.1f}C",
            f"Threshold: {self.temp_threshold:.1f}C",
            vel_str,
            f"Time: {time_str}"
        ]
        
        # Draw semi-transparent background for text
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (self.display_width, 100), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
        
        # Draw text
        y_offset = 20
        for line in info_lines:
            cv2.putText(
                frame,
                line,
                (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )
            y_offset += 22
    
    def _capture_and_publish(self):
        """Capture once and publish both radiometric image and temperature array"""
        try:
            # Capture fresh radiometric frame
            radiometric_frame = self.camera.grab()
            
            if radiometric_frame is None:
                self.get_logger().warn('Failed to capture radiometric frame')
                return
            
            
            # Also publish radiometric array (UInt16), units: Kelvin x 100
            rad_u16 = radiometric_frame.astype(np.uint16, copy=False)
            h, w = rad_u16.shape
            array_msg = UInt16MultiArray()
            array_msg.layout = MultiArrayLayout(
                dim=[
                    MultiArrayDimension(label='height', size=h, stride=h * w),
                    MultiArrayDimension(label='width', size=w, stride=w),
                ],
                data_offset=0,
            )
            array_msg.data = rad_u16.flatten().tolist()
            self.array_publisher.publish(array_msg)

            self.get_logger().info('Published radiometric array only (uint16, Kx100) (velocity = 0)')
            
        except Exception as e:
            self.get_logger().error(f'Failed to capture/publish radiometric data: {e}')
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down thermal stream node...')
        
        # Stop streaming loop
        self.streaming_active = False
        
        # Wait for thread to finish
        if self.stream_thread.is_alive():
            self.stream_thread.join(timeout=2.0)
        
        # Close camera
        try:
            if hasattr(self, 'camera'):
                self.camera.close()
        except Exception as e:
            self.get_logger().error(f'Error closing camera: {e}')
        
        # Terminate FFmpeg
        if self.ffmpeg_process is not None:
            try:
                self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.terminate()
                self.ffmpeg_process.wait(timeout=2.0)
            except Exception as e:
                self.get_logger().error(f'Error closing FFmpeg: {e}')
        
        # Close OpenCV windows
        cv2.destroyAllWindows()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ThermalStreamNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
