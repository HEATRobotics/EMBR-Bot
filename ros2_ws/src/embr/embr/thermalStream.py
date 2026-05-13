#!/usr/bin/env python3
"""
Thermal Camera Streaming Node with Temperature Overlay.
Uses sensor abstraction to support both real and simulated thermal cameras.
Streams via HDMI with direct framebuffer access and publishes the radiometric array (UInt16, Kelvin*100) when stationary.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension, MultiArrayLayout
from msg_interface.msg import GPSAndIMU
from embr.sensors import create_sensor, SensorConfig, SensorFactory
import cv2
import numpy as np
import subprocess
import threading
import queue


class ThermalStreamNode(Node):
    def __init__(self):
        super().__init__('thermal_stream_node')
        
        # Declare parameter for config file path
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value
        
        # Use default config path if not provided
        if not config_file:
            config_file = 'src/embr/config/sensors.json'
        
        # Try to load from config file
        try:
            config = SensorFactory.load_config('thermal', config_file)
            
            if config is None:
                self.get_logger().warning(f'Thermal sensor not found in config file: {config_file}. Using default real mode.')
                config = SensorConfig(mode='real', params={'model': '3.1R'})
            else:
                self.get_logger().info(f'Loaded thermal camera config from {config_file}')
        except Exception as e:
            self.get_logger().warning(f'Failed to load config file: {e}. Using default real mode.')
            config = SensorConfig(mode='real', params={'model': '3.1R'})
        
        # Configuration - load from config params with defaults
        params = config.params if config else {}
        # Temp threshold will be set via radio commands, default here
        self.temp_threshold = 30.0  # Celsius - will be updated via radio
        self.video_fps = 9.0  # Lepton typical fps
        self.display_width = params.get('display_width', 640)
        self.display_height = params.get('display_height', 480)
        # use a reddish/inferno colormap for thermal-style colors
        colormap_name = params.get('colormap', 'INFERNO')
        self.colormap = getattr(cv2, f'COLORMAP_{colormap_name}', cv2.COLORMAP_INFERNO)
        self.min_temp = params.get('min_temp_c', 10.0)  # Celsius - minimum temperature for colormap scaling
        self.max_temp = params.get('max_temp_c', 80.0)  # Celsius - maximum temperature for colormap scaling
        
        # State variables
        self.current_velocity = None
        self.velocity_lock = threading.Lock()
        
        # Publisher: radiometric array (uint16, Kelvin x 100) for direct analysis
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
        
        # Initialize camera using sensor abstraction
        self.get_logger().info('Initializing thermal camera...')
        try:
            self.camera = create_sensor('thermal', config_file)
            self.camera.start()
            
            sensor_type = 'simulated' if 'Sim' in self.camera.__class__.__name__ else 'real'
            self.get_logger().info(f'Thermal camera initialized in {config.mode} mode (using {sensor_type} sensor)')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize camera: {e}')
            raise
        
        # FFmpeg process for HDMI streaming
        self.ffmpeg_process = None
        # queue used to buffer frames for ffmpeg writer thread; small size to keep latency low
        self.frame_queue = queue.Queue(maxsize=4)
        # control how often expensive overlays (contours/stats) are computed
        self.overlay_every_n_frames = 1  # set >1 to do overlays less frequently
        
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
        # Initialize direct framebuffer writer (faster and lower latency than ffmpeg)
        try:
            self._init_fbdev()
            self._fbdev_writer_thread = threading.Thread(target=self._fbdev_writer_loop, daemon=True)
            self._fbdev_writer_thread.start()
            self._fbdev_active = True
            self.get_logger().info('FBDev writer thread started (direct framebuffer output)')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize framebuffer writer: {e}')
            self._fbdev_active = False

    def _init_fbdev(self):
        """Read fb0 properties (resolution, bpp) and compute layout for direct writes."""
        try:
            with open('/sys/class/graphics/fb0/virtual_size', 'r') as f:
                vs = f.read().strip()
            fb_w, fb_h = [int(x) for x in vs.split(',')]
        except Exception:
            fb_w, fb_h = 1920, 1080

        try:
            with open('/sys/class/graphics/fb0/bits_per_pixel', 'r') as f:
                bpp = int(f.read().strip())
        except Exception:
            bpp = 16

        self._fb_width = fb_w
        self._fb_height = fb_h
        self._fb_bpp = bpp
        self._fb_bpp_bytes = max(1, bpp // 8)
        self.get_logger().debug(f'FB dev init: {fb_w}x{fb_h} {bpp}bpp')

    def _fbdev_writer_loop(self):
        """Consume frames from the queue (BGR uint8 arrays) and write to /dev/fb0 in rgb565le."""
        try:
            fd = open('/dev/fb0', 'r+b', buffering=0)
        except Exception as e:
            self.get_logger().error(f'Could not open /dev/fb0 for writing: {e}')
            return

        while self.streaming_active:
            try:
                frame = self.frame_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if frame is None:
                continue

            try:
                img = frame
                # Scale the image to fit within the framebuffer while preserving aspect
                scale = min(self._fb_width / img.shape[1], self._fb_height / img.shape[0])
                inner_w = max(1, int(img.shape[1] * scale))
                inner_h = max(1, int(img.shape[0] * scale))
                img_scaled = cv2.resize(img, (inner_w, inner_h), interpolation=cv2.INTER_LINEAR)

                # Convert BGR -> RGB565 little endian
                r = (img_scaled[:, :, 2] >> 3).astype(np.uint16)
                g = (img_scaled[:, :, 1] >> 2).astype(np.uint16)
                b = (img_scaled[:, :, 0] >> 3).astype(np.uint16)
                rgb565 = (r << 11) | (g << 5) | b

                # Create full framebuffer buffer and place rgb565 into center
                fb_buf = np.zeros((self._fb_height, self._fb_width), dtype=np.uint16)
                x0 = max(0, (self._fb_width - inner_w) // 2)
                y0 = max(0, (self._fb_height - inner_h) // 2)
                fb_buf[y0:y0+inner_h, x0:x0+inner_w] = rgb565

                out_bytes = fb_buf.astype('<u2').tobytes()
                try:
                    fd.seek(0)
                    fd.write(out_bytes)
                except Exception as e:
                    self.get_logger().error(f'Failed writing to /dev/fb0: {e}')
            except Exception as e:
                self.get_logger().error(f'Error in fbdev writer loop: {e}')
                continue

        try:
            fd.close()
        except Exception:
            pass

    def _read_ffmpeg_stderr(self):
        """Read FFmpeg stderr and log lines for debugging."""
        if self.ffmpeg_process is None or self.ffmpeg_process.stderr is None:
            return
        try:
            for line in iter(self.ffmpeg_process.stderr.readline, b''):
                try:
                    text = line.decode('utf-8', errors='replace').strip()
                except Exception:
                    text = str(line)
                if text:
                    # Log ffmpeg output at debug level to avoid spamming
                    self.get_logger().debug(f'FFmpeg: {text}')
        except Exception as e:
            self.get_logger().error(f'Error reading FFmpeg stderr: {e}')

    def _ffmpeg_writer_loop(self):
        """Consume frames from the queue and write to ffmpeg stdin."""
        while self.streaming_active:
            try:
                data = self.frame_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            # ignore empty sentinel
            if not data:
                continue

            if self.ffmpeg_process is None or self.ffmpeg_process.stdin is None:
                continue

            try:
                self.ffmpeg_process.stdin.write(data)
            except BrokenPipeError:
                self.get_logger().error('FFmpeg pipe broken in writer')
                self.ffmpeg_process = None
                break
            except Exception as e:
                self.get_logger().error(f'Error writing to ffmpeg stdin: {e}')
                # if writing fails, drop frame and continue
                continue

        # try to flush
        try:
            if self.ffmpeg_process and self.ffmpeg_process.stdin:
                try:
                    self.ffmpeg_process.stdin.flush()
                except Exception:
                    pass
        except Exception:
            pass
    
    def _streaming_loop(self):
        """Main loop for capturing and processing thermal frames"""
        self.get_logger().info('Starting thermal streaming loop')
        
        while self.streaming_active and rclpy.ok():
            try:
                # Capture radiometric frame from camera using sensor abstraction
                radiometric_frame = self.camera.read()
                
                if radiometric_frame is None:
                    self.get_logger().warn('Failed to grab frame from camera')
                    continue


                undistorted = self.camera.get_undistorted_img(radiometric_frame)
                
                # Convert to Celsius (Lepton outputs in Kelvin * 100)
                temp_celsius = (undistorted / 100.0) - 273.15
                
                # Create display frame with colormap and overlays
                display_frame = self._create_display_frame(temp_celsius)
                
                # Optionally skip expensive overlays every N frames
                if self.overlay_every_n_frames > 1:
                    # use a modulo counter stored on the node
                    if not hasattr(self, '_overlay_counter'):
                        self._overlay_counter = 0
                    self._overlay_counter = (self._overlay_counter + 1) % self.overlay_every_n_frames
                    if self._overlay_counter != 0:
                        # regenerate a lightweight display frame (just colormap + resize)
                        # this avoids repeated contour/findContours work
                        temp_normalized = np.clip(
                            (temp_celsius - self.min_temp) / (self.max_temp - self.min_temp) * 255,
                            0,
                            255
                        ).astype(np.uint8)
                        display_frame = cv2.applyColorMap(temp_normalized, self.colormap)
                        display_frame = cv2.resize(display_frame, (self.display_width, self.display_height), interpolation=cv2.INTER_LINEAR)

                # Send to framebuffer writer via queue or display with OpenCV
                if getattr(self, '_fbdev_active', False):
                    try:
                        # Non-blocking enqueue; drop oldest frame if queue is full to preserve low latency
                        try:
                            self.frame_queue.put_nowait(display_frame)
                        except queue.Full:
                            try:
                                # drop oldest
                                _ = self.frame_queue.get_nowait()
                                self.frame_queue.put_nowait(display_frame)
                            except Exception:
                                # if we still can't enqueue, just drop this frame
                                pass
                    except Exception:
                        self.get_logger().error('Frame queue error for fbdev writer')
                else:
                    # Fallback: display with OpenCV (useful for debugging)
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
        # Adaptive normalize temperature to 0-255 range for colormap using percentiles
        try:
            lo, hi = np.percentile(temp_celsius, (2, 98))
            lo = max(lo, self.min_temp)
            hi = min(hi, self.max_temp)
            if hi - lo < 0.1:
                lo = self.min_temp
                hi = self.max_temp
            temp_normalized = np.clip((temp_celsius - lo) / (hi - lo) * 255, 0, 255).astype(np.uint8)
        except Exception:
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
        
        # Find and annotate hot spots above threshold (efficient, on radiometric data)
        hot_mask = temp_celsius > self.temp_threshold

        if np.any(hot_mask):
            # Create uint8 mask for contours
            hot_mask_uint8 = (hot_mask.astype(np.uint8) * 255)
            contours, _ = cv2.findContours(
                hot_mask_uint8,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            # Scale factor for contours (from thermal resolution to display resolution)
            scale_x = self.display_width / temp_celsius.shape[1]
            scale_y = self.display_height / temp_celsius.shape[0]

            for contour in contours:
                # Skip tiny contours to avoid noise
                if cv2.contourArea(contour) < 3:
                    continue

                # Calculate temperature statistics for this region
                mask_region = np.zeros_like(temp_celsius, dtype=np.uint8)
                cv2.drawContours(mask_region, [contour], -1, 1, -1)
                region_temps = temp_celsius[mask_region == 1]

                if region_temps.size == 0:
                    continue

                max_temp = float(np.max(region_temps))

                # Scale contour to display size and draw a bounding box
                scaled_contour = contour.astype(np.float32)
                scaled_contour[:, 0, 0] *= scale_x
                scaled_contour[:, 0, 1] *= scale_y
                scaled_contour = scaled_contour.astype(np.int32)

                x, y, w_box, h_box = cv2.boundingRect(scaled_contour)
                # Draw a thin rectangle around the hot region
                cv2.rectangle(display_frame, (x, y), (x + w_box, y + h_box), (0, 255, 255), 2)

                # Prepare temperature label and draw above the box
                temp_text = f"{max_temp:.1f}C"
                text_x = x
                text_y = max(10, y - 6)

                # Draw black outline for readability then white text
                cv2.putText(display_frame, temp_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(display_frame, temp_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        return display_frame    
   
    def _capture_and_publish(self):
        """Capture once and publish temperature array"""
        try:
            # Capture fresh radiometric frame using sensor abstraction
            radiometric_frame = self.camera.read()
            
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

            self.get_logger().info('Published radiometric array (uint16, Kx100) (velocity = 0)')
            
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
        
        # Close camera using sensor abstraction
        try:
            if hasattr(self, 'camera'):
                self.camera.stop()
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
