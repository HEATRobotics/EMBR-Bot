#!/usr/bin/env python3
"""
Fusion node for publishing custom GPSAndIMU messages on topic /gps.

msg_interface/GPSAndIMU fields:
- float32 lat
- float32 lon
- float32 alt
- float32 vel
- float32 pitch
- float32 yaw
- float32 roll

Intended consumers:
- ros2_ws/src/embr/embr/hotspotLocator.py
- ros2_ws/src/embr/embr/thermalStream.py
- ros2_ws/src/embr/embr/sensors/thermal.py
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, NavSatFix
from msg_interface.msg import GPSAndIMU

import message_filters


class GpsImuFusion(Node):
    def __init__(self):
        super().__init__('gps_imu_fusion')

        # --- Parameters ---
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        self.declare_parameter('output_topic', 'gps')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('gps_topic', '/gps/fix')

        self.declare_parameter('queue_size', 20)
        self.declare_parameter('slop', 0.05)

        self.declare_parameter('warn_on_frame_mismatch', True)
        self.declare_parameter('warn_on_zero_stamp', True)
        self.declare_parameter('log_every_n', 1)

        self.output_topic = str(self.get_parameter('output_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.gps_topic = str(self.get_parameter('gps_topic').value)

        self.queue_size = int(self.get_parameter('queue_size').value)
        self.slop = float(self.get_parameter('slop').value)

        self.warn_on_frame_mismatch = bool(self.get_parameter('warn_on_frame_mismatch').value)
        self.warn_on_zero_stamp = bool(self.get_parameter('warn_on_zero_stamp').value)
        self.log_every_n = max(1, int(self.get_parameter('log_every_n').value))

        # --- Publisher ---
        self.pub = self.create_publisher(GPSAndIMU, self.output_topic, 10)

        # --- Speed estimation state ---
        self._prev_lat = None
        self._prev_lon = None
        self._prev_alt = None
        self._prev_time = None

        # --- Debug state ---
        self._sync_count = 0
        self._last_sync_clock_ns = None

        # --- QoS ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(10, self.queue_size),
        )

        # --- Subscribers ---
        self.imu_sub = message_filters.Subscriber(
            self, Imu, self.imu_topic, qos_profile=sensor_qos
        )
        self.gps_sub = message_filters.Subscriber(
            self, NavSatFix, self.gps_topic, qos_profile=sensor_qos
        )

        # --- Approximate sync ---
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.imu_sub, self.gps_sub],
            queue_size=self.queue_size,
            slop=self.slop,
            allow_headerless=False,
        )
        self.sync.registerCallback(self.synced_cb)

        self.get_logger().info(
            f"GpsImuFusion started.\n"
            f"  IMU topic: {self.imu_topic}\n"
            f"  GPS topic: {self.gps_topic}\n"
            f"  Output topic: {self.output_topic}\n"
            f"  queue_size: {self.queue_size}\n"
            f"  slop: {self.slop:.3f}s\n"
            f"Waiting for synced message pairs..."
        )

    @staticmethod
    def _stamp_to_float(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _quat_to_euler(x, y, z, w):
        """
        Convert quaternion to roll, pitch, yaw in radians.
        """
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    @staticmethod
    def _haversine_distance(lat1, lon1, lat2, lon2):
        """
        Great-circle distance between two lat/lon points in meters.
        """
        R = 6378137.0

        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        a = (
            math.sin(dlat / 2.0) ** 2
            + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2.0) ** 2
        )
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        return R * c

    def _estimate_speed(self, lat, lon, alt, current_time):
        """
        Estimate scalar speed in m/s from consecutive GPS fixes.
        """
        if self._prev_lat is None or self._prev_time is None:
            self._prev_lat = lat
            self._prev_lon = lon
            self._prev_alt = alt
            self._prev_time = current_time
            return 0.0

        dt = current_time - self._prev_time
        if dt <= 1e-6:
            return 0.0

        horizontal_dist = self._haversine_distance(
            self._prev_lat, self._prev_lon, lat, lon
        )
        vertical_dist = alt - self._prev_alt
        total_dist = math.sqrt(horizontal_dist * horizontal_dist + vertical_dist * vertical_dist)

        speed = total_dist / dt

        self._prev_lat = lat
        self._prev_lon = lon
        self._prev_alt = alt
        self._prev_time = current_time

        return speed

    def synced_cb(self, imu_msg: Imu, gps_msg: NavSatFix):
        self._sync_count += 1

        imu_t = self._stamp_to_float(imu_msg.header.stamp)
        gps_t = self._stamp_to_float(gps_msg.header.stamp)
        dt = abs(imu_t - gps_t)

        # --- Timestamp sanity ---
        if self.warn_on_zero_stamp:
            if imu_msg.header.stamp.sec == 0 and imu_msg.header.stamp.nanosec == 0:
                self.get_logger().warn("IMU has zero timestamp.")
            if gps_msg.header.stamp.sec == 0 and gps_msg.header.stamp.nanosec == 0:
                self.get_logger().warn("GPS has zero timestamp.")

        # --- Frame sanity ---
        if self.warn_on_frame_mismatch:
            imu_frame = imu_msg.header.frame_id
            gps_frame = gps_msg.header.frame_id
            if imu_frame and gps_frame and imu_frame != gps_frame:
                self.get_logger().warn(
                    f"Frame mismatch: IMU frame='{imu_frame}' vs GPS frame='{gps_frame}'"
                )

        # --- Debug sync period ---
        now_ns = self.get_clock().now().nanoseconds
        sync_period_s = None
        if self._last_sync_clock_ns is not None:
            sync_period_s = (now_ns - self._last_sync_clock_ns) / 1e9
        self._last_sync_clock_ns = now_ns

        if (self._sync_count % self.log_every_n) == 0:
            msg = (
                f"[sync #{self._sync_count}] "
                f"IMU={imu_t:.6f}s GPS={gps_t:.6f}s | |Δt|={dt:.4f}s"
            )
            if sync_period_s is not None:
                msg += f" | period={sync_period_s:.3f}s"
            self.get_logger().info(msg)

        # Skip invalid GPS
        if gps_msg.status.status < 0:
            return

        lat = float(gps_msg.latitude)
        lon = float(gps_msg.longitude)
        alt = float(gps_msg.altitude)

        speed = self._estimate_speed(lat, lon, alt, gps_t)

        roll, pitch, yaw = self._quat_to_euler(
            float(imu_msg.orientation.x),
            float(imu_msg.orientation.y),
            float(imu_msg.orientation.z),
            float(imu_msg.orientation.w),
        )

        out = GPSAndIMU()
        out.lat = lat
        out.lon = lon
        out.alt = alt
        out.vel = float(speed)
        out.pitch = float(pitch)
        out.yaw = float(yaw)
        out.roll = float(roll)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GpsImuFusion()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt: shutting down gps_imu_fusion.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()