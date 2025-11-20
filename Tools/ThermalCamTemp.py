"""
Thermal Hotspot GPS Locator - ROS Integration Version
Works with FLIR Lepton 2.5 or 3.1R on PureThermal 3

Two modes:
- SIMULATION: Testing on Windows with fake data
- ROS: Production on RPi with GPS data from ROS topics
"""

import cv2
import numpy as np
import math
import time
from datetime import datetime
import platform
import os
from flirpy.camera.lepton import Lepton

# Operating mode configuration
IS_WINDOWS = platform.system() == "Windows"
SIMULATION_MODE = False  # Set to False when running in ROS on rover
CALIBRATION_MODE = True  # Set to True for testing with real camera + fake GPS
USE_ROS = not SIMULATION_MODE and not CALIBRATION_MODE  # Use ROS only in production

# ROS imports (only when USE_ROS is True)
if USE_ROS:
    try:
        import rospy
        from sensor_msgs.msg import NavSatFix, Imu
        from std_msgs.msg import Float64
        from geometry_msgs.msg import PoseStamped
        from tf.transformations import euler_from_quaternion
        print("ROS imports successful")
    except ImportError:
        print("ERROR: ROS not found. Install ROS or set SIMULATION_MODE = True")
        exit(1)

# Always needed
try:
    from geopy.distance import distance as geopy_distance
    from geopy.point import Point
except ImportError:
    print("ERROR: geopy required. Install with: pip install geopy")
    exit(1)


class ThermalGPSLocator:
    def __init__(self, config):
        """
        Initialize thermal GPS locator
        
        Args:
            config: Dictionary with configuration parameters
        """
        self.config = config
        self.altitude = config['altitude_m']
        self.pitch = math.radians(config['pitch_deg'])
        
        # FLIR Lepton specs - Different for 2.5 vs 3.1R
        if config['lepton_model'] == '2.5':
            self.img_width = 80
            self.img_height = 60
            self.hfov = math.radians(51)   # Horizontal FOV
            self.vfov = math.radians(38)   # Vertical FOV
        elif config['lepton_model'] == '3.1R':
            # Lepton 3.1R has higher resolution and wider FOV
            self.img_width = 160
            self.img_height = 120
            self.hfov = math.radians(57)
            self.vfov = math.radians(44)
        else:
            raise ValueError("lepton_model must be '2.5' or '3.1R'")
        
        print(f"Initialized for Lepton {config['lepton_model']}: {self.img_width}x{self.img_height}")
        
        # State variables
        self.current_gps = None
        self.current_heading = 0
        self.current_ground_speed = 0  # m/s
        
        # ROS-specific state
        self.ros_gps_received = False
        self.ros_heading_received = False
        self.last_gps_time = None
        
        # Initialize ROS if enabled
        if USE_ROS:
            self._init_ros()
    
    def _init_ros(self):
        """Initialize ROS node and subscribers"""
        print("Initializing ROS interface...")
        
        # Initialize node if not already initialized
        try:
            rospy.init_node('thermal_gps_locator', anonymous=True)
            print("ROS node initialized")
        except rospy.exceptions.ROSException:
            print("ROS node already initialized")
        
        # Subscribe to GPS topic
        # Expects: sensor_msgs/NavSatFix with latitude, longitude, altitude
        # IMPORTANT: Change 'ros_gps_topic' in config to match your rover's topic
        rospy.Subscriber(
            self.config['ros_gps_topic'],
            NavSatFix,
            self._gps_callback,
            queue_size=1
        )
        print(f"Subscribed to GPS topic: {self.config['ros_gps_topic']}")
        
        # Subscribe to IMU/heading topic
        # Expects: sensor_msgs/Imu with orientation quaternion
        # IMPORTANT: Change 'ros_imu_topic' in config to match your rover's topic
        if self.config['ros_imu_topic']:
            rospy.Subscriber(
                self.config['ros_imu_topic'],
                Imu,
                self._imu_callback,
                queue_size=1
            )
            print(f"Subscribed to IMU topic: {self.config['ros_imu_topic']}")
        
        # Publisher for hotspot results
        # Other ROS nodes can subscribe to this for navigation/logging
        self.hotspot_pub = rospy.Publisher(
            '/thermal/hotspot_gps',
            PoseStamped,
            queue_size=10
        )
        print("Publisher created: /thermal/hotspot_gps")
        
        # Wait for initial GPS message
        print("Waiting for GPS data...")
        timeout = rospy.Time.now() + rospy.Duration(10.0)
        while not self.ros_gps_received and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        
        if self.ros_gps_received:
            print("GPS data received!")
        else:
            print("Warning: No GPS data received yet")
    
    def _gps_callback(self, msg):
        """
        ROS callback for GPS data from rover
        
        Args:
            msg: sensor_msgs/NavSatFix
                - latitude (float64): Latitude in degrees
                - longitude (float64): Longitude in degrees  
                - altitude (float64): Altitude in meters
        """
        self.current_gps = (msg.latitude, msg.longitude)
        
        # Optionally use altitude from GPS instead of fixed camera height
        # Uncomment if you want dynamic altitude:
        # self.altitude = msg.altitude + self.config['camera_height_above_base']
        
        self.ros_gps_received = True
        self.last_gps_time = rospy.Time.now()
        
        if self.config.get('verbose', False):
            print(f"GPS update: {msg.latitude:.6f}, {msg.longitude:.6f}")
    
    def _imu_callback(self, msg):
        """
        ROS callback for IMU data to get heading
        
        Args:
            msg: sensor_msgs/Imu
                - orientation: Quaternion (x, y, z, w)
        """
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        
        _, _, yaw = euler_from_quaternion(orientation)
        
        # Convert yaw from radians to degrees (0-360)
        # Yaw is typically 0 = East, 90 = North in ROS
        # Convert to compass heading: 0 = North, 90 = East
        self.current_heading = (90 - math.degrees(yaw)) % 360
        
        self.ros_heading_received = True
        
        if self.config.get('verbose', False):
            print(f"Heading update: {self.current_heading:.1f}°")
    
    def get_gps_position(self):
        """Get current GPS position (mode-dependent)"""
        if SIMULATION_MODE:
            # Simulation: Return fake GPS coordinates
            self.current_gps = self.config['sim_gps']
            return self.current_gps
        
        if CALIBRATION_MODE:
            # Calibration: Use test GPS position (you'll set this to match your test location)
            self.current_gps = self.config['test_gps']
            print(f"Using test GPS: {self.current_gps[0]:.6f}, {self.current_gps[1]:.6f}")
            return self.current_gps
        
        # ROS mode: Data comes from callback
        if not self.ros_gps_received:
            print("Warning: No GPS data received from ROS topic yet")
            return None
        
        # Check if data is recent (within last 2 seconds)
        if self.last_gps_time:
            age = (rospy.Time.now() - self.last_gps_time).to_sec()
            if age > 2.0:
                print(f"Warning: GPS data is {age:.1f}s old")
        
        return self.current_gps
    
    def get_heading(self):
        """Get current compass heading in degrees (0-360)"""
        if SIMULATION_MODE:
            # Simulation: Return fake heading
            self.current_heading = self.config['sim_heading']
            return self.current_heading
        
        if CALIBRATION_MODE:
            # Calibration: Use test heading (direction camera is pointing)
            self.current_heading = self.config['test_heading']
            print(f"Using test heading: {self.current_heading:.1f}°")
            return self.current_heading
        
        # ROS mode: Data comes from IMU callback
        if not self.ros_heading_received:
            print("Warning: No heading data received from ROS IMU topic")
            # Could implement fallback: calculate heading from consecutive GPS points
            # if rover is moving fast enough
        
        return self.current_heading
    
    def capture_thermal_image(self):
        """Capture thermal image from FLIR Lepton"""
        if SIMULATION_MODE:
            # Generate simulated thermal image with hotspots
            # Units: centikelvin (Kelvin * 100)
            # Room temp ~20°C = 29315 centikelvin
            img = np.random.randint(29000, 30000, 
                                   (self.img_height, self.img_width), 
                                   dtype=np.uint16)
            
            # Add simulated hotspots at different temperatures
            # Hotspot 1: 32°C (30515 centikelvin) - center-right
            cv2.circle(img, (self.img_width//2 + 10, self.img_height//2 - 5), 8, 30515, -1)
            
            # Hotspot 2: 35°C (30815 centikelvin) - upper-left
            cv2.circle(img, (self.img_width//3, self.img_height//3), 5, 30815, -1)
            
            # Hotspot 3: 38°C (31115 centikelvin) - lower-center
            cv2.circle(img, (self.img_width//2, int(self.img_height*0.7)), 6, 31115, -1)
            
            print(f"Simulated thermal image: {img.shape}, range: {img.min()}-{img.max()} centikelvin")
            return img
        
              
        # Fallback: OpenCV capture
        if IS_WINDOWS or platform.system() == "Linux":
            try:
                with Lepton() as camera:
                    # Get raw thermal data
                    # Print camera diagnostic fields (safe access)
                    thermal = camera.grab()
                    
                    if thermal is None:
                        raise RuntimeError("Failed to capture frame from Lepton")
                    
                    print(thermal.min(), thermal.max())
                    
                    # Convert from raw values to Celsius
                    temp_celsius = (thermal.astype(np.float32) / 100.0) - 273.15

                    temp_celsius = cv2.flip(temp_celsius, 0)  # Flip vertically
                    
                    print(f"Captured {thermal.shape} thermal frame")
                    print(f"Temperature range: {temp_celsius.min():.1f}°C to {temp_celsius.max():.1f}°C")

                    # Save both raw and visualization images
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    
                    # Create colored visualization
                    display_img = ((temp_celsius - temp_celsius.min()) / 
                                 (temp_celsius.max() - temp_celsius.min()) * 255).astype(np.uint8)
                    display_color = cv2.applyColorMap(display_img, cv2.COLORMAP_INFERNO)
                    
                    # Add temperature overlay
                    cv2.putText(display_color, f"Min: {temp_celsius.min():.1f}°C", (10, 20),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(display_color, f"Max: {temp_celsius.max():.1f}°C", (10, 40),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    # Save the visualization with temperature overlay
                    vis_filename = f"thermal_{timestamp}_viz.png"
                    cv2.imwrite(vis_filename, display_color)
                    
                    # Save the raw thermal data for analysis
                    raw_filename = f"thermal_{timestamp}_raw.npy"
                    np.save(raw_filename, thermal)
                    
                    print(f"Saved images:")
                    print(f"- Visualization: {vis_filename}")
                    print(f"- Raw data: {raw_filename}")
                    
                    camera.close()
                                    
                # Return the temperature data in Celsius
                return temp_celsius
                
            except Exception as e:
                print(f"Error capturing thermal image: {e}")
                print("\nDiagnostic information:")
                print("1. Check if the FLIR Lepton is properly connected")
                print("2. Verify /dev/video1 permissions (try: ls -l /dev/video1)")
                print("3. Make sure no other process is using the camera")
                print("\nTrying to reset USB device...")
                
                try:
                    import subprocess
                    # Try to reset the USB device
                    subprocess.run(['sudo', 'usb_modeswitch', '-R', '-v', '0x1e4e', '-p', '0x0100'])
                    print("USB device reset attempted")
                except Exception as reset_error:
                    print(f"Could not reset USB device: {reset_error}")
                
                import traceback
                traceback.print_exc()
                
                return None
    
    def find_hotspots(self, thermal_img, temp_threshold):
        """
        Find hotspot blobs (groups of hot pixels) above temperature threshold
        
        Uses connected component analysis to group adjacent hot pixels into blobs,
        then returns the centroid and max temperature of each blob.
        
        Args:
            thermal_img: Raw thermal image in celcius
            temp_threshold: Temperature threshold in celcius
            
        Returns:
            List of (centroid_x, centroid_y, max_temp, blob_size) tuples
            sorted by max temperature (hottest first)
        """
        # Create binary mask of pixels above threshold
        hot_mask = (thermal_img > temp_threshold).astype(np.uint8)
        
        # Find connected components (blobs of hot pixels)
        # connectivity=8 means diagonal pixels are considered connected
        num_blobs, labels, stats, centroids = cv2.connectedComponentsWithStats(
            hot_mask, connectivity=8
        )
        
        hotspots = []
        
        # Skip label 0 (background)
        for i in range(1, num_blobs):
            # Get blob statistics
            centroid_x = centroids[i][0]
            centroid_y = centroids[i][1]
            blob_size = stats[i, cv2.CC_STAT_AREA]  # Number of pixels in blob
            
            # Find maximum temperature within this blob
            blob_mask = (labels == i)
            max_temp = thermal_img[blob_mask].max()
            avg_temp = thermal_img[blob_mask].mean()
            
            hotspots.append({
                'centroid_x': float(centroid_x),
                'centroid_y': float(centroid_y),
                'max_temp': float(max_temp),
                'avg_temp': float(avg_temp),
                'size_pixels': int(blob_size)
            })
        
        # Sort by maximum temperature (hottest first)
        hotspots.sort(key=lambda h: h['max_temp'], reverse=True)
        
        print(f"Found {len(hotspots)} hotspot blobs above {temp_threshold:.1f}°C")
        if hotspots:
            print(f"  Largest blob: {hotspots[0]['size_pixels']} pixels, "
                  f"Max temp: {hotspots[0]['max_temp']:.1f}°C")
        
        return hotspots
    
    def pixel_to_angle(self, x, y):
        """
        Convert pixel coordinates to angular offsets from camera center
        
        Args:
            x, y: Pixel coordinates in image
            
        Returns:
            (azimuth_offset, elevation_offset) in radians
            - azimuth_offset: Horizontal angle (positive = right)
            - elevation_offset: Vertical angle (positive = up)
        """
        # Normalize to -0.5 to 0.5 range
        norm_x = (x - self.img_width / 2) / self.img_width
        norm_y = (y - self.img_height / 2) / self.img_height
        
        # Convert to angles using field of view
        azimuth_offset = norm_x * self.hfov
        elevation_offset = norm_y * self.vfov
        
        return azimuth_offset, elevation_offset
    
    def estimate_ground_distance(self, elevation_offset):
        """
        Estimate horizontal ground distance to hotspot
        
        Uses camera altitude and pitch angle to estimate where the
        pixel's line of sight intersects the ground plane.
        
        Args:
            elevation_offset: Vertical angle offset from camera center (radians)
            
        Returns:
            Distance in meters (horizontal ground distance) or None if invalid
        """
        # Calculate total elevation angle to hotspot
        # Positive elevation_offset means looking up in image
        # We want positive to mean looking down at ground
        elevation_angle = self.pitch - elevation_offset
        
        # Check if looking above horizontal or straight down
        if elevation_angle <= 0 or elevation_angle >= math.pi / 2:
            return None
        
        # Calculate ground distance using trigonometry
        # distance = height / tan(angle)
        ground_distance = self.altitude / math.tan(elevation_angle)
        
        # Sanity check: reject unreasonable distances
        if ground_distance < 0 or ground_distance > 1000:
            return None
        
        return ground_distance
    
    def calculate_hotspot_gps(self, pixel_x, pixel_y):
        """
        Calculate GPS coordinates of a hotspot from its pixel location
        
        Args:
            pixel_x, pixel_y: Pixel coordinates in thermal image
            
        Returns:
            Tuple of (latitude, longitude, bearing, distance_m) or None
        """
        if self.current_gps is None:
            print("Warning: No GPS position available")
            return None
        
        # Get angular offsets from image center
        azimuth_offset, elevation_offset = self.pixel_to_angle(pixel_x, pixel_y)
        
        # Calculate absolute compass bearing to hotspot
        bearing = (self.current_heading + math.degrees(azimuth_offset)) % 360
        
        # Estimate ground distance
        ground_dist = self.estimate_ground_distance(elevation_offset)
        if ground_dist is None or ground_dist < 0:
            return None
        
        # Calculate GPS coordinates using bearing and distance
        start = Point(self.current_gps[0], self.current_gps[1])
        destination = geopy_distance(meters=ground_dist).destination(start, bearing)
        
        return (destination.latitude, destination.longitude, bearing, ground_dist)
    
    def publish_hotspot_ros(self, hotspot_data):
        """
        Publish hotspot GPS coordinates to ROS topic
        
        Args:
            hotspot_data: Dictionary with hotspot information
        """
        if not USE_ROS:
            return
        
        # Create PoseStamped message
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"  # Adjust if using different coordinate frame
        
        # Position: Use lat/lon as x/y (or convert to local coordinates)
        msg.pose.position.x = hotspot_data['longitude']
        msg.pose.position.y = hotspot_data['latitude']
        msg.pose.position.z = 0  # Ground level
        
        # Orientation: Not meaningful for a point, but required
        msg.pose.orientation.w = 1.0
        
        self.hotspot_pub.publish(msg)
        
        if self.config.get('verbose', False):
            print(f"Published hotspot to /thermal/hotspot_gps")
    
    def find_nearest_hotspot_gps(self, temp_threshold_celsius):
        """
        Main function: Find GPS coordinates of nearest thermal hotspot
        
        Args:
            temp_threshold_celsius: Temperature threshold in Celsius
            
        Returns:
            Dictionary with hotspot information or None if no hotspots found
        """
        print("\n" + "="*60)
        print("THERMAL HOTSPOT GPS LOCATOR")
        print(f"Mode: {'SIMULATION' if SIMULATION_MODE else 'ROS'}")
        print("="*60)
        
        # Step 1: Get current GPS position
        print("\n1. Getting GPS position...")
        self.get_gps_position()
        if self.current_gps is None:
            print("ERROR: No GPS fix available")
            return None
        print(f"   Current GPS: {self.current_gps[0]:.6f}, {self.current_gps[1]:.6f}")
        
        # Step 2: Get compass heading
        print("\n2. Getting compass heading...")
        self.get_heading()
        print(f"   Current heading: {self.current_heading:.1f}°")
        
        # Step 3: Capture thermal image
        print("\n3. Capturing thermal image...")
        thermal_img = self.capture_thermal_image()
        if thermal_img is None:
            print("ERROR: Failed to capture thermal image")
            return None
        
        # Step 4: Find hotspots above threshold
        print(f"\n4. Finding hotspots above {temp_threshold_celsius:.1f}°C...")
        hotspots = self.find_hotspots(thermal_img, temp_threshold_celsius)
        
        if not hotspots:
            print("   No hotspots found above threshold")
            return None
        
        # Step 5: Calculate GPS coordinates for each hotspot blob
        print("\n5. Calculating GPS coordinates for hotspots...")
        nearest = None
        min_distance = float('inf')
        valid_hotspots = []
        
        # Process top 10 hottest blobs
        for i, blob in enumerate(hotspots[:10]):
            x = blob['centroid_x']
            y = blob['centroid_y']
            max_temp = blob['max_temp']
            avg_temp = blob['avg_temp']
            size = blob['size_pixels']
            
            result = self.calculate_hotspot_gps(x, y)
            if result:
                lat, lon, bearing, dist = result
                temp_c = max_temp
                avg_temp_c = avg_temp
                
                hotspot_info = {
                    'centroid_x': x,
                    'centroid_y': y,
                    'max_temperature_c': temp_c,
                    'avg_temperature_c': avg_temp_c,
                    'size_pixels': size,
                    'latitude': lat,
                    'longitude': lon,
                    'bearing': bearing,
                    'distance_m': dist
                }
                valid_hotspots.append(hotspot_info)
                
                print(f"   Blob {i+1}: {size} pixels, max {temp_c:.1f}°C, avg {avg_temp_c:.1f}°C")
                print(f"           -> {lat:.6f},{lon:.6f} (bearing: {bearing:.1f}°, dist: {dist:.1f}m)")
                
                # Track nearest hotspot
                if dist < min_distance:
                    min_distance = dist
                    nearest = hotspot_info
        
        if nearest is None:
            print("   Could not calculate GPS for any hotspots")
            return None
        
        # Publish to ROS if enabled
        if USE_ROS:
            self.publish_hotspot_ros(nearest)
        
        # Print summary
        print("\n" + "="*60)
        print("NEAREST HOTSPOT FOUND:")
        print("="*60)
        print(f"Blob Size:    {nearest['size_pixels']} pixels")
        print(f"Max Temp:     {nearest['max_temperature_c']:.1f}°C")
        print(f"Avg Temp:     {nearest['avg_temperature_c']:.1f}°C")
        print(f"GPS:          {nearest['latitude']:.6f}, {nearest['longitude']:.6f}")
        print(f"Bearing:      {nearest['bearing']:.1f}° from North")
        print(f"Distance:     {nearest['distance_m']:.1f} meters")
        print(f"Google Maps:  https://www.google.com/maps?q={nearest['latitude']},{nearest['longitude']}")
        print("="*60 + "\n")
        
        return nearest


def main():
    """Main function with configuration"""
    
    # ==================== CONFIGURATION ====================
    config = {
        # Hardware settings
        'lepton_model': '2.5',  # Change to '3.1R' when you upgrade
        'camera_index': 0,       # USB camera index (try 0, 1, 2 if not working)
        
        # ROS settings (ROS mode only)
        # IMPORTANT: Update these to match your rover's actual topic names
        'ros_gps_topic': '/gps/fix',      # Your rover's GPS topic
        'ros_imu_topic': '/imu/data',     # Your rover's IMU topic (or None if no IMU)
        
        # Camera mounting geometry
        # CALIBRATION MODE: Measure these values carefully!
        'altitude_m': 0,       # Camera height above ground (meters) - MEASURE THIS
        'pitch_deg': 0,       # Camera pitch angle (0=horizontal, 90=down) - MEASURE THIS
        
        # Detection parameters
        'temp_threshold_celsius': 25.0,  # Only detect above this temperature
        
        # Simulation settings (SIMULATION_MODE only)
        'sim_gps': (49.8880, -119.4960),  # Example: Kelowna, BC
        'sim_heading': 307.0,               # Simulated compass heading (45° = NE)
        
        # Calibration/Testing settings (CALIBRATION_MODE only)
        # Set these to match your test setup location
        'test_gps': (49.8880, -119.4960),      # Your actual test location GPS
        'test_heading': 292.0,                    # Direction camera is pointing (0=North, 90=East, etc.)
        'save_images': True,                    # Save thermal images during testing
        
        # Debug options
        'verbose': False,  # Set True for detailed GPS/IMU update messages
    }
    
    print("="*60)
    print("THERMAL HOTSPOT GPS LOCATOR - ROS INTEGRATION")
    print("="*60)
    
    mode_str = "SIMULATION" if SIMULATION_MODE else ("CALIBRATION" if CALIBRATION_MODE else "ROS")
    print(f"Mode:          {mode_str}")
    print(f"Platform:      {platform.system()}")
    print(f"Lepton Model:  {config['lepton_model']}")
    
    if CALIBRATION_MODE:
        print("\n" + "="*60)
        print("CALIBRATION MODE - Testing Instructions")
        print("="*60)
        print("1. Set up your camera at a measured height and angle")
        print("2. Update config values:")
        print(f"   - 'test_gps': Your camera's GPS position")
        print(f"   - 'test_heading': Direction camera points (0=N, 90=E, 180=S, 270=W)")
        print(f"   - 'altitude_m': Measured camera height = {config['altitude_m']:.2f}m")
        print(f"   - 'pitch_deg': Measured camera angle = {config['pitch_deg']:.1f}°")
        print("3. Place a hot object (person, heater, etc.) at a known distance")
        print("4. Measure actual distance with tape measure")
        print("5. Run this script and compare predicted vs actual GPS/distance")
        print("6. Thermal images will be saved for review")
        print("="*60 + "\n")
    
    if USE_ROS:
        print(f"GPS Topic:     {config['ros_gps_topic']}")
        print(f"IMU Topic:     {config['ros_imu_topic']}")
    
    print("="*60 + "\n")
    
    # ==================== RUN ====================
    try:
        locator = ThermalGPSLocator(config)
        
        if USE_ROS:
            # ROS mode: Run continuously at set rate
            rate = rospy.Rate(1)  # 1 Hz - scan once per second
            
            print("Running in ROS mode. Press Ctrl+C to stop.\n")
            
            scan_count = 0
            while not rospy.is_shutdown():
                scan_count += 1
                print(f"\n[Scan #{scan_count}]")
                
                result = locator.find_nearest_hotspot_gps(
                    config['temp_threshold_celsius']
                )
                
                if result:
                    print(f"✓ Hotspot detected and published")
                else:
                    print("○ No hotspots detected")
                
                rate.sleep()
        else:
            # Simulation mode: Single scan for testing
            result = locator.find_nearest_hotspot_gps(
                config['temp_threshold_celsius']
            )
            
            if result:
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                print(f"\n✓ Success! Scan completed at {timestamp}")
            else:
                print("\n○ No hotspots found")
            
    except KeyboardInterrupt:
        print("\n\n" + "="*60)
        print("SHUTDOWN: Interrupted by user")
        print("="*60)
        if USE_ROS:
            rospy.signal_shutdown("User interrupt")
    except Exception as e:
        print(f"\n\nERROR: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()