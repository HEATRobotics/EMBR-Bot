#!/usr/bin/env python3
"""
Thermal Hotspot GPS Locator (ROS2)

Production ROS2 node that subscribes to the radiometric 2D array
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

import numpy as np
import cv2
from geopy.distance import distance as geopy_distance
from geopy.point import Point


class ThermalHotspotLocator(Node):
    def __init__(self):
        super().__init__('thermal_hotspot_locator')

        # Configuration (can be adjusted here)
        # Camera geometry
        self.lepton_model = '2.5'  # '2.5' or '3.1R'
        self.altitude_m = 1.0      # Camera height above ground (meters)
        self.pitch_deg = 45.0      # Camera pitch (0=horizontal, 90=down)
        self.temp_threshold_c = 30.0  # Celsius threshold for hotspot detection

        # Topics (override here if needed)
        self.array_topic = 'thermal/radiometric_array'
        self.gps_imu_topic = 'gps_imu'

        # Set Lepton model parameters
        self._set_camera_model(self.lepton_model)

        # State
        self.current_gps: Optional[Tuple[float, float]] = None  # (lat, lon)
        self.current_heading_deg: float = 0.0  # compass heading 0-360, 0=N, 90=E
        self.have_gps: bool = False

        # Subscribers
        self.create_subscription(GPSAndIMU, self.gps_imu_topic, self._gps_imu_cb, 10)
        self.create_subscription(UInt16MultiArray, self.array_topic, self._array_cb, 10)

        # Publisher
        self.hotspot_pub = self.create_publisher(PoseStamped, '/thermal/hotspot_gps', 10)

        self.get_logger().info('ThermalHotspotLocator started')

    def _set_camera_model(self, model: str) -> None:
        if model == '2.5':
            # Lepton 2.5
            self.hfov = math.radians(51)
            self.vfov = math.radians(38)
        elif model == '3.1R':
            self.hfov = math.radians(57)
            self.vfov = math.radians(44)
        else:
            self.get_logger().warn("Unknown lepton_model, defaulting to 2.5 FOV")
            self.hfov = math.radians(51)
            self.vfov = math.radians(38)

    # -------------------- Subscribers --------------------
    def _gps_imu_cb(self, msg: GPSAndIMU) -> None:
        # Extract GPS (lat/lon are int32 in 1e7 format, alt is int32 in mm)
        self.current_gps = (float(msg.lat) / 1e7, float(msg.lon) / 1e7)
        self.have_gps = True
        
        # Extract heading from yaw (yaw is in radians as int32, needs conversion)
        # The yaw field appears to be in radians based on getCube.py using attitude.yaw
        yaw_rad = float(msg.yaw)
        
        # Convert yaw to compass heading: 0 = North, 90 = East
        # Assuming yaw is ENU convention: 0 = East, +pi/2 = North
        heading = (90.0 - math.degrees(yaw_rad)) % 360.0
        self.current_heading_deg = heading

    def _array_cb(self, msg: UInt16MultiArray) -> None:
        if not self.have_gps:
            self.get_logger().warn_once('Waiting for GPS fix...')
            return

        # Determine image shape from layout
        if len(msg.layout.dim) >= 2:
            h = msg.layout.dim[0].size
            w = msg.layout.dim[1].size
        else:
            # Fallback: assume Lepton 2.5 default if no layout
            h, w = 60, 80

        rad = np.array(msg.data, dtype=np.uint16)
        if rad.size != h * w:
            self.get_logger().warn(f'Array size mismatch: {rad.size} vs {h}x{w}')
            return
        rad = rad.reshape((h, w))

        # Convert to Celsius for analysis
        temp_c = (rad.astype(np.float32) / 100.0) - 273.15

        # Detect hotspots and publish nearest
        hotspots = self._find_hotspots(temp_c, self.temp_threshold_c)
        if not hotspots:
            return

        nearest = self._compute_nearest_hotspot_gps(hotspots, w, h)
        if nearest is None:
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
        for blob in hotspots[:10]:
            az, el = self._pixel_to_angle(blob['centroid_x'], blob['centroid_y'], width, height)
            bearing = (self.current_heading_deg + math.degrees(az)) % 360.0
            ground_dist = self._estimate_ground_distance(el)
            if ground_dist is None:
                continue

            start = Point(self.current_gps[0], self.current_gps[1])
            dest = geopy_distance(meters=ground_dist).destination(start, bearing)
            dist_m = ground_dist

            info = {
                'centroid_x': blob['centroid_x'],
                'centroid_y': blob['centroid_y'],
                'size_pixels': blob['size_pixels'],
                'max_temperature_c': blob['max_temp'],
                'avg_temperature_c': blob['avg_temp'],
                'latitude': dest.latitude,
                'longitude': dest.longitude,
                'bearing': bearing,
                'distance_m': dist_m,
            }

            if dist_m < min_dist:
                min_dist = dist_m
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
            f"bearing={data['bearing']:.1f}°, dist={data['distance_m']:.1f}m, "
            f"max={data['max_temperature_c']:.1f}C, size={data['size_pixels']}px"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ThermalHotspotLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
