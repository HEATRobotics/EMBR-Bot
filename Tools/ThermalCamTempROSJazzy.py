"""
Thermal Hotspot GPS Locator - ROS2 Integration Version (Minimal Changes)
Works with FLIR Lepton 2.5 or 3.1R on PureThermal 3

Two modes:
- SIMULATION: Testing on Windows with fake data
- ROS: Production on RPi with GPS data from ROS topics
"""

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import math
from datetime import datetime
import platform

# Operating mode configuration
IS_WINDOWS = platform.system() == "Windows"
SIMULATION_MODE = False  # Set to False when running in ROS on rover
CALIBRATION_MODE = False  # Set to True for testing with real camera + fake GPS
USE_ROS = not SIMULATION_MODE and not CALIBRATION_MODE  # Use ROS only in production


# ROS2 message imports (only when USE_ROS is True)
if USE_ROS:
    try:
        from sensor_msgs.msg import NavSatFix, Imu
        from geometry_msgs.msg import PoseStamped
        print("ROS2 imports successful")
    except ImportError as e:
        print(f"ERROR: ROS2 message imports failed: {e}")
        exit(1)

# Always needed
try:
    from geopy.distance import distance as geopy_distance
    from geopy.point import Point
except ImportError:
    print("ERROR: geopy required. Install with: pip install geopy")
    exit(1)


def yaw_from_quaternion(x, y, z, w):
    """
    Minimal replacement for tf.transformations.euler_from_quaternion.
    Returns yaw (rad).
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ThermalGPSLocator:
    def __init__(self, config):
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

        # ROS2 node handle (created in _init_ros)
        self.node = None
        self.hotspot_pub = None

        if USE_ROS:
            self._init_ros()

    def _init_ros(self):
        """Initialize ROS2 node and subscribers (minimal changes)"""
        print("Initializing ROS2 interface...")

        # Create ROS2 node (REQUIRES rclpy.init() already called)
        self.node = Node('thermal_gps_locator')

        # Subscribe to GPS topic
        self.node.create_subscription(
            NavSatFix,
            self.config['ros_gps_topic'],
            self._gps_callback,
            10
        )
        print(f"Subscribed to GPS topic: {self.config['ros_gps_topic']}")

        # Subscribe to IMU topic
        if self.config['ros_imu_topic']:
            self.node.create_subscription(
                Imu,
                self.config['ros_imu_topic'],
                self._imu_callback,
                10
            )
            print(f"Subscribed to IMU topic: {self.config['ros_imu_topic']}")

        # Publisher for hotspot results
        self.hotspot_pub = self.node.create_publisher(
            PoseStamped,
            '/thermal/hotspot_gps',
            10
        )
        print("Publisher created: /thermal/hotspot_gps")

        print("ROS2 interface ready (GPS/IMU will arrive asynchronously).")

    def _gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        self.ros_gps_received = True
        self.last_gps_time = self.node.get_clock().now()

        if self.config.get('verbose', False):
            print(f"GPS update: {msg.latitude:.6f}, {msg.longitude:.6f}")

    def _imu_callback(self, msg):
        yaw = yaw_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        # Convert ROS yaw (0=East, 90=North) to compass heading (0=North, 90=East)
        self.current_heading = (90 - math.degrees(yaw)) % 360
        self.ros_heading_received = True

        if self.config.get('verbose', False):
            print(f"Heading update: {self.current_heading:.1f}°")

    def get_gps_position(self):
        if SIMULATION_MODE:
            self.current_gps = self.config['sim_gps']
            return self.current_gps

        if CALIBRATION_MODE:
            self.current_gps = self.config['test_gps']
            print(f"Using test GPS: {self.current_gps[0]:.6f}, {self.current_gps[1]:.6f}")
            return self.current_gps

        if not self.ros_gps_received:
            print("Warning: No GPS data received from ROS topic yet")
            return None

        if self.last_gps_time:
            age_ns = (self.node.get_clock().now() - self.last_gps_time).nanoseconds
            age = age_ns / 1e9
            if age > 2.0:
                print(f"Warning: GPS data is {age:.1f}s old")

        return self.current_gps

    def get_heading(self):
        if SIMULATION_MODE:
            self.current_heading = self.config['sim_heading']
            return self.current_heading

        if CALIBRATION_MODE:
            self.current_heading = self.config['test_heading']
            print(f"Using test heading: {self.current_heading:.1f}°")
            return self.current_heading

        if not self.ros_heading_received:
            print("Warning: No heading data received from ROS IMU topic")

        return self.current_heading

    def capture_thermal_image(self):
        if SIMULATION_MODE:
            img = np.random.randint(29000, 30000, (self.img_height, self.img_width), dtype=np.uint16)
            cv2.circle(img, (self.img_width//2 + 10, self.img_height//2 - 5), 8, 30515, -1)
            cv2.circle(img, (self.img_width//3, self.img_height//3), 5, 30815, -1)
            cv2.circle(img, (self.img_width//2, int(self.img_height*0.7)), 6, 31115, -1)
            print(f"Simulated thermal image: {img.shape}, range: {img.min()}-{img.max()} centikelvin")
            temp_celsius = (img.astype(np.float32) / 100.0) - 273.15
            return temp_celsius

        try:
            from flirpy.camera.lepton import Lepton  
            with Lepton() as camera:
                thermal = camera.grab()
                if thermal is None:
                    raise RuntimeError("Failed to capture frame from Lepton")

                temp_celsius = (thermal.astype(np.float32) / 100.0) - 273.15
                temp_celsius = cv2.flip(temp_celsius, 0)

                print(f"Captured {thermal.shape} thermal frame")
                print(f"Temperature range: {temp_celsius.min():.1f}°C to {temp_celsius.max():.1f}°C")

                if self.config.get('save_images', False):
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

                    display_img = ((temp_celsius - temp_celsius.min()) /
                                   (temp_celsius.max() - temp_celsius.min() + 1e-6) * 255).astype(np.uint8)
                    display_color = cv2.applyColorMap(display_img, cv2.COLORMAP_INFERNO)

                    cv2.putText(display_color, f"Min: {temp_celsius.min():.1f}°C", (10, 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(display_color, f"Max: {temp_celsius.max():.1f}°C", (10, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                    vis_filename = f"thermal_{timestamp}_viz.png"
                    raw_filename = f"thermal_{timestamp}_raw.npy"
                    cv2.imwrite(vis_filename, display_color)
                    np.save(raw_filename, thermal)

                    print(f"Saved images:\n- {vis_filename}\n- {raw_filename}")

                return temp_celsius

        except Exception as e:
            print(f"Error capturing thermal image: {e}")
            return None

    def find_hotspots(self, thermal_img, temp_threshold):
        hot_mask = (thermal_img > temp_threshold).astype(np.uint8)

        num_blobs, labels, stats, centroids = cv2.connectedComponentsWithStats(
            hot_mask, connectivity=8
        )

        hotspots = []
        for i in range(1, num_blobs):
            blob_mask = (labels == i)
            max_temp = thermal_img[blob_mask].max()
            avg_temp = thermal_img[blob_mask].mean()

            hotspots.append({
                'centroid_x': float(centroids[i][0]),
                'centroid_y': float(centroids[i][1]),
                'max_temp': float(max_temp),
                'avg_temp': float(avg_temp),
                'size_pixels': int(stats[i, cv2.CC_STAT_AREA])
            })

        hotspots.sort(key=lambda h: h['max_temp'], reverse=True)
        print(f"Found {len(hotspots)} hotspot blobs above {temp_threshold:.1f}°C")
        return hotspots

    def pixel_to_angle(self, x, y):
        norm_x = (x - self.img_width / 2) / self.img_width
        norm_y = (y - self.img_height / 2) / self.img_height
        azimuth_offset = norm_x * self.hfov
        elevation_offset = norm_y * self.vfov
        return azimuth_offset, elevation_offset

    def estimate_ground_distance(self, elevation_offset):
        elevation_angle = self.pitch - elevation_offset
        if elevation_angle <= 0 or elevation_angle >= math.pi / 2:
            return None
        ground_distance = self.altitude / math.tan(elevation_angle)
        if ground_distance < 0 or ground_distance > 1000:
            return None
        return ground_distance

    def calculate_hotspot_gps(self, pixel_x, pixel_y):
        if self.current_gps is None:
            print("Warning: No GPS position available")
            return None

        azimuth_offset, elevation_offset = self.pixel_to_angle(pixel_x, pixel_y)
        bearing = (self.current_heading + math.degrees(azimuth_offset)) % 360

        ground_dist = self.estimate_ground_distance(elevation_offset)
        if ground_dist is None or ground_dist < 0:
            return None

        start = Point(self.current_gps[0], self.current_gps[1])
        destination = geopy_distance(meters=ground_dist).destination(start, bearing)
        return (destination.latitude, destination.longitude, bearing, ground_dist)

    def publish_hotspot_ros(self, hotspot_data):
        if not USE_ROS:
            return

        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Keep your original convention (lon->x, lat->y)
        msg.pose.position.x = hotspot_data['longitude']
        msg.pose.position.y = hotspot_data['latitude']
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        self.hotspot_pub.publish(msg)

        if self.config.get('verbose', False):
            print("Published hotspot to /thermal/hotspot_gps")

    def find_nearest_hotspot_gps(self, temp_threshold_celsius):
        print("\n" + "="*60)
        print("THERMAL HOTSPOT GPS LOCATOR")
        print(f"Mode: {'SIMULATION' if SIMULATION_MODE else ('CALIBRATION' if CALIBRATION_MODE else 'ROS2')}")
        print("="*60)

        print("\n1. Getting GPS position...")
        self.get_gps_position()
        if self.current_gps is None:
            print("ERROR: No GPS fix available")
            return None
        print(f"   Current GPS: {self.current_gps[0]:.6f}, {self.current_gps[1]:.6f}")

        print("\n2. Getting compass heading...")
        self.get_heading()
        print(f"   Current heading: {self.current_heading:.1f}°")

        print("\n3. Capturing thermal image...")
        thermal_img = self.capture_thermal_image()
        if thermal_img is None:
            print("ERROR: Failed to capture thermal image")
            return None

        print(f"\n4. Finding hotspots above {temp_threshold_celsius:.1f}°C...")
        hotspots = self.find_hotspots(thermal_img, temp_threshold_celsius)
        if not hotspots:
            print("   No hotspots found above threshold")
            return None

        print("\n5. Calculating GPS coordinates for hotspots...")
        nearest = None
        min_distance = float('inf')

        for i, blob in enumerate(hotspots[:10]):
            x = blob['centroid_x']
            y = blob['centroid_y']

            result = self.calculate_hotspot_gps(x, y)
            if result:
                lat, lon, bearing, dist = result

                hotspot_info = {
                    'centroid_x': x,
                    'centroid_y': y,
                    'max_temperature_c': blob['max_temp'],
                    'avg_temperature_c': blob['avg_temp'],
                    'size_pixels': blob['size_pixels'],
                    'latitude': lat,
                    'longitude': lon,
                    'bearing': bearing,
                    'distance_m': dist
                }

                print(f"   Blob {i+1}: {blob['size_pixels']} pixels, "
                      f"max {blob['max_temp']:.1f}°C, avg {blob['avg_temp']:.1f}°C")
                print(f"           -> {lat:.6f},{lon:.6f} (bearing: {bearing:.1f}°, dist: {dist:.1f}m)")

                if dist < min_distance:
                    min_distance = dist
                    nearest = hotspot_info

        if nearest is None:
            print("   Could not calculate GPS for any hotspots")
            return None

        if USE_ROS:
            self.publish_hotspot_ros(nearest)

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
    config = {
        'lepton_model': '2.5',
        'camera_index': 0,

        # ROS topics (ROS2 mode only)
        'ros_gps_topic': '/gps/fix',
        'ros_imu_topic': '/imu/data',

        # Camera mounting geometry
        'altitude_m': 0.0,
        'pitch_deg': 0.0,

        'temp_threshold_celsius': 25.0,

        # Simulation
        'sim_gps': (49.8880, -119.4960),
        'sim_heading': 307.0,

        # Calibration
        'test_gps': (49.8880, -119.4960),
        'test_heading': 292.0,
        'save_images': True,

        'verbose': False,
    }

    print("="*60)
    print("THERMAL HOTSPOT GPS LOCATOR - ROS2 (MINIMAL CHANGES)")
    print("="*60)

    # ✅ MANDATORY: init ROS2 before creating any Node()
    if USE_ROS:
        rclpy.init()

    locator = ThermalGPSLocator(config)

    if USE_ROS:
        scan_count = {"n": 0}

        def timer_cb():
            scan_count["n"] += 1
            print(f"\n[Scan #{scan_count['n']}]")
            result = locator.find_nearest_hotspot_gps(config['temp_threshold_celsius'])
            if result:
                print("✓ Hotspot detected and published")
            else:
                print("○ No hotspots detected")

        locator.node.create_timer(1.0, timer_cb)

        try:
            print("Running in ROS2 mode. Press Ctrl+C to stop.\n")
            rclpy.spin(locator.node)
        except KeyboardInterrupt:
            pass
        finally:
            locator.node.destroy_node()
            rclpy.shutdown()

    else:
        result = locator.find_nearest_hotspot_gps(config['temp_threshold_celsius'])
        if result:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            print(f"\n✓ Success! Scan completed at {timestamp}")
        else:
            print("\n○ No hotspots found")


if __name__ == "__main__":
    main()