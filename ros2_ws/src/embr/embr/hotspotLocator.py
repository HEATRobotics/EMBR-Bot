#!/usr/bin/env python3
"""
Thermal Hotspot GPS Locator (ROS2)

ROS2 node that subscribes to the radiometric 2D array
published by the thermal stream node and computes the nearest hotspot's
GPS coordinates based on camera geometry, GPS, and IMU heading.

Inputs:
- /thermal/radiometric_array (std_msgs/UInt16MultiArray) — raw radiometric data (Kelvin x 100)
- /gps_imu (msg_interface/GPSAndIMU) — GPS position and orientation

Outputs:
- /thermal/hotspot_gps (geometry_msgs/PoseStamped) — nearest hotspot lat/lon encoded as x/y

"""

import math
from typing import Optional, Tuple, List, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
from geometry_msgs.msg import PoseStamped
from msg_interface.msg import GPSAndIMU
from embr.sensors import SensorFactory, SensorConfig, create_sensor

import numpy as np
import cv2
from geopy.distance import distance as geopy_distance
from geopy.point import Point


class hotspotLocator(Node):
    def __init__(self):
        super().__init__('thermal_hotspot_locator')

        # Declare parameter for config file path
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value
        
        # Use default config path if not provided
        if not config_file:
            config_file = 'src/embr/config/sensors.json'
        
        # Try to load thermal camera config to get model information
        try:
            configs = SensorFactory.load_config(config_file)
            thermal_config = configs.get('thermal')
            
            if thermal_config is None:
                self.get_logger().warning(f'Thermal sensor not found in config file: {config_file}. Using defaults.')
                self.lepton_model = '3.1R'
                params = {}
            else:
                params = thermal_config.params
                self.lepton_model = params.get('model', '3.1R')
                self.get_logger().info(f'Loaded thermal camera config from {config_file}: model={self.lepton_model}')
        except Exception as e:
            self.get_logger().warning(f'Failed to load config file: {e}. Using defaults.')
            self.lepton_model = '3.1R'
            params = {}

        # Create thermal sensor object from config so we can access camera intrinsics
        self.thermal_sensor = create_sensor('thermal', thermal_config)

        # Configuration - load from config params with defaults
        # Camera geometry
        self.altitude_m = params.get('altitude_m', 1.0)      # Camera height above ground (meters)
        self.pitch_deg = params.get('pitch_deg', 45.0)      # Camera pitch (0=horizontal, 90=down)
        # Temp threshold will be set via radio commands, default here
        self.temp_threshold_c = 30.0  # Celsius - will be updated via radio

        # Topics (override here if needed)
        self.array_topic = 'thermal/radiometric_array'
        self.gps_imu_topic = 'gps'

        # Set Lepton model parameters based on config
        self._set_camera_model(self.lepton_model)      

        # State
        self.current_gps: Optional[Tuple[float, float]] = None  # (lat, lon)
        self.current_heading_deg: float = 0.0  # compass heading 0-360, 0=N, 90=E
        self.current_roll_deg: float = 0.0
        self.current_pitch_deg: float = 0.0
        self.current_yaw_deg: float = 0.0
        self.have_gps: bool = False
        # internal flag to avoid spamming the log while waiting for GPS
        self._warned_waiting_gps: bool = False

        # Subscribers
        self.create_subscription(GPSAndIMU, self.gps_imu_topic, self._gps_imu_cb, 10)
        self.create_subscription(UInt16MultiArray, self.array_topic, self._array_cb, 10)

        # Publisher
        self.hotspot_pub = self.create_publisher(PoseStamped, '/thermal/hotspot_gps', 10)

        self.get_logger().info(f'hotspotLocator started (camera model: {self.lepton_model})')

    def _set_camera_model(self, model: str) -> None:
        if model == '2.5':
            # Lepton 2.5
            self.hfov = math.radians(51)
            self.vfov = math.radians(38)
        elif model == '3.1R':
            self.hfov = math.radians(57)
            self.vfov = math.radians(44)
        else:
            self.get_logger().warning("Unknown lepton_model, defaulting to 2.5 FOV")
            self.hfov = math.radians(51)
            self.vfov = math.radians(38)

    # -------------------- Subscribers --------------------
    def _gps_imu_cb(self, msg: GPSAndIMU) -> None:
        # Extract GPS (lat/lon are floats in degrees)
        self.current_gps = (float(msg.lat), float(msg.lon))
        self.have_gps = True

        # Store IMU attitude (assume msg pitch/yaw/roll are in degrees from cube sensor)
        self.current_pitch_deg = float(msg.pitch)
        self.current_yaw_deg = float(msg.yaw)
        self.current_roll_deg = float(msg.roll)

        # Convert yaw to compass heading: 0 = North, 90 = East
        # Assuming yaw is ENU convention: 0 = East, +pi/2 = North
        heading = (90.0 - math.degrees(msg.yaw)) % 360.0
        self.current_heading_deg = heading

    def _array_cb(self, msg: UInt16MultiArray) -> None:
        self.get_logger().info("Thermal Array received")
        if not self.have_gps:
            # warn once to avoid log spam while waiting for GPS
            if not getattr(self, '_warned_waiting_gps', False):
                self.get_logger().warning('Waiting for GPS fix...')
                self._warned_waiting_gps = True
            return

        # Determine image shape from layout
        h = msg.layout.dim[0].size
        w = msg.layout.dim[1].size

        rad = np.array(msg.data, dtype=np.uint16)
        rad = rad.reshape((h, w))

        # Convert to Celsius for analysis
        temp_c = (rad.astype(np.float32) / 100.0) - 273.15

        # Detect hotspots and publish nearest
        hotspots = self._find_hotspots(temp_c, self.temp_threshold_c)
        if not hotspots:
            self.get_logger().info("No hotspots found")
            return

        nearest = self._compute_nearest_hotspot_gps(hotspots, w, h)
        if nearest is None:
            self.get_logger().info("Nearest hotspot not found")
            return

        self._publish_hotspot(nearest)

    # -------------------- Processing --------------------
    def _find_hotspots(self, temp_c: np.ndarray, threshold_c: float) -> List[Dict]:
        hot_mask = (temp_c > threshold_c).astype(np.uint8)
        num, labels, stats, centroids = cv2.connectedComponentsWithStats(hot_mask, connectivity=8)

        out: List[Dict] = []
        for i in range(1, num):  # skip background
            cx, cy = float(centroids[i][0]), float(centroids[i][1])
            size = int(stats[i, cv2.CC_STAT_AREA])
            blob_mask = (labels == i)
            max_temp = float(temp_c[blob_mask].max())
            avg_temp = float(temp_c[blob_mask].mean())
            out.append({
                'centroid_x': cx,
                'centroid_y': cy,
                'size_pixels': size,
                'max_temp': max_temp,
                'avg_temp': avg_temp,
            })

        out.sort(key=lambda h: h['max_temp'], reverse=True)
        return out

    def _pixel_to_angle(self, x: float, y: float, width: int, height: int) -> Tuple[float, float]:
        # Normalize coordinates to [-0.5, 0.5]
        nx = (x - width / 2.0) / width
        ny = (y - height / 2.0) / height
        az = nx * self.hfov
        el = ny * self.vfov
        return az, el

    # New camera-based helpers
    def _pixel_to_camera_ray(self, u: float, v: float, width: int, height: int) -> np.ndarray:
        """Return unit ray in camera coords for pixel (u,v). Uses intrinsics"""
        pts = np.array([[[u, v]]], dtype=np.float64)
        und = cv2.undistortPoints(pts, self.thermal_sensor.camera_matrix, self.thermal_sensor.distortion_coeff, P=None)
        x_norm = float(und[0, 0, 0])
        y_norm = float(und[0, 0, 1])
        ray = np.array([x_norm, y_norm, 1.0], dtype=np.float64)
        ray /= np.linalg.norm(ray)
        return ray

    def _camera_to_world_ray(self, ray_cam: np.ndarray, roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
        """Rotate ray from camera frame to world frame using roll/pitch/yaw (degrees)."""
        r = math.radians(roll_deg)
        p = math.radians(pitch_deg)
        y = math.radians(yaw_deg)
        Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]], dtype=np.float64)
        Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]], dtype=np.float64)
        Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]], dtype=np.float64)
        R_world_cam = Rz @ Ry @ Rx
        ray_world = R_world_cam @ ray_cam
        ray_world /= np.linalg.norm(ray_world)
        return ray_world

    def _intersect_ray_ground(self, camera_alt_m: float, ray_world: np.ndarray):
        """Intersect a world ray with z=0 ground plane. Returns (distance_m, ground_vec) or None."""
        dz = ray_world[2]
        if dz >= 0:
            return None
        t = - camera_alt_m / dz
        if t <= 0:
            return None
        horiz_vec = ray_world * t
        horiz_distance = math.hypot(horiz_vec[0], horiz_vec[1])
        return horiz_distance, horiz_vec

    def _estimate_ground_distance(self, el_offset: float) -> Optional[float]:
        pitch = math.radians(self.pitch_deg)
        elevation_angle = pitch - el_offset
        if elevation_angle <= 0.0 or elevation_angle >= math.pi / 2:
            return None
        dist = self.altitude_m / math.tan(elevation_angle)
        if dist < 0 or dist > 10000:
            return None
        return dist

    def _compute_nearest_hotspot_gps(self, hotspots: List[Dict], width: int, height: int) -> Optional[Dict]:
        if self.current_gps is None:
            return None

        nearest = None
        min_dist = float('inf')
        self.get_logger().info(f"hotspots found: {len(hotspots)}")

        for blob in hotspots[:10]:
            u = blob['centroid_x']
            v = blob['centroid_y']

            ray_cam = self._pixel_to_camera_ray(u, v, width, height)

            # Camera attitude: roll/yaw follow the vehicle; pitch adds the fixed camera tilt
            cam_roll = self.current_roll_deg
            cam_pitch = self.current_pitch_deg + float(self.pitch_deg)
            cam_yaw = self.current_yaw_deg

            # Convert Optical Frame (OpenCV) to Body Frame (ENU)
            ray_body = np.array([
                ray_cam[2],  # Body X (Forward) <- Optical Z
                -ray_cam[0], # Body Y (Left) <- Optical -X
                -ray_cam[1]  # Body Z (Up) <- Optical -Y
            ])

            ray_world = self._camera_to_world_ray(ray_body, cam_roll, cam_pitch, cam_yaw)
            dz = ray_world[2]
            self.get_logger().info(
                f"Hotspot centroid=({u:.2f},{v:.2f}), ray_world={ray_world}, dz={dz:.4f}, cam_pitch={cam_pitch:.2f}, cam_yaw={cam_yaw:.2f}, cam_roll={cam_roll:.2f}"
            )

            ground_res = self._intersect_ray_ground(self.altitude_m, ray_world)
            if ground_res is None:
                self.get_logger().info("Ray does not intersect ground (dz >= 0 or t <= 0)")
                continue

            ground_dist, ground_vec = ground_res

            # compute bearing from north clockwise
            east = ground_vec[0]
            north = ground_vec[1]
            bearing_rad = math.atan2(east, north)
            bearing_deg = (math.degrees(bearing_rad) + 360.0) % 360.0

            start = Point(self.current_gps[0], self.current_gps[1])
            dest = geopy_distance(meters=ground_dist).destination(start, bearing_deg)

            info = {
                'centroid_x': u,
                'centroid_y': v,
                'size_pixels': blob['size_pixels'],
                'max_temperature_c': blob['max_temp'],
                'avg_temperature_c': blob['avg_temp'],
                'latitude': dest.latitude,
                'longitude': dest.longitude,
                'bearing': bearing_deg,
                'distance_m': ground_dist,
            }

            if ground_dist < min_dist:
                min_dist = ground_dist
                nearest = info

        return nearest

    def _publish_hotspot(self, data: Dict) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        # Store lon/lat in x/y for a quick transport (matches original script)
        msg.pose.position.x = float(data['longitude'])
        msg.pose.position.y = float(data['latitude'])
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        self.hotspot_pub.publish(msg)
        self.get_logger().info(
            f"Hotspot -> lat={data['latitude']:.6f}, lon={data['longitude']:.6f}, "
            f"bearing={data['bearing']:.1f}°, dist={data['distance_m']:.5f}m, "
            f"max={data['max_temperature_c']:.1f}C, size={data['size_pixels']}px"
        )


def main(args=None):
    rclpy.init(args=args)
    node = hotspotLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
