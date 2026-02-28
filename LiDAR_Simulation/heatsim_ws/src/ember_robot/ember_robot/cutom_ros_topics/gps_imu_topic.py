# Fusion script for creating the gps_imu node needed for custom ros topic /gps_imu (msg_interface/GPSAndIMU).
#   This is the GPS position and orientation
# Scripts thay may use this:
# |- ros2_ws/src/embr/embr/hotspotLocator.py
# |- ros2_ws/src/embr/embr/thermalStream.py
# |- ros2_ws/src/embr/embr/sensors/thermal.py

#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry

import message_filters


class GpsImuFusion(Node):
    def __init__(self):
        super().__init__('gps_imu_fusion')

        # --- Parameters ---
        # If you're in Gazebo, set use_sim_time true in launch or via param.
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        # --- Output params ---
        self.declare_parameter('output_topic', '/gps_imu')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')

        self.output_topic = str(self.get_parameter('output_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)

        # --- Publisher ---
        self.odom_pub = self.create_publisher(Odometry, '/gps_imu', 10)

        # --- Local ENU origin (set on first valid GPS fix) ---
        self._origin_set = False
        self._lat0 = 0.0
        self._lon0 = 0.0
        self._alt0 = 0.0
        self._cos_lat0 = 1.0

        # WGS84-ish earth radius (meters)
        self._R = 6378137.0

        # Approx sync parameters
        self.declare_parameter('queue_size', 20)
        self.declare_parameter('slop', 0.05)  # seconds

        # Topic names as parameters (so you can remap without editing code)
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('gps_topic', '/gps/fix')

        # Debug options
        self.declare_parameter('warn_on_frame_mismatch', True)
        self.declare_parameter('warn_on_zero_stamp', True)
        self.declare_parameter('log_every_n', 1)  # set to e.g. 10 to reduce spam

        self.queue_size = int(self.get_parameter('queue_size').value)
        self.slop = float(self.get_parameter('slop').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.gps_topic = str(self.get_parameter('gps_topic').value)

        self.warn_on_frame_mismatch = bool(self.get_parameter('warn_on_frame_mismatch').value)
        self.warn_on_zero_stamp = bool(self.get_parameter('warn_on_zero_stamp').value)
        self.log_every_n = max(1, int(self.get_parameter('log_every_n').value))

        # --- QoS ---
        # Many IMU/GPS drivers publish BEST_EFFORT. If you subscribe RELIABLE, you may get nothing.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(10, self.queue_size),
        )

        # --- Subscribers (message_filters) ---
        self.imu_sub = message_filters.Subscriber(
            self, Imu, self.imu_topic, qos_profile=sensor_qos
        )
        self.gps_sub = message_filters.Subscriber(
            self, NavSatFix, self.gps_topic, qos_profile=sensor_qos
        )

        # --- Time synchronizer ---
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.imu_sub, self.gps_sub],
            queue_size=self.queue_size,
            slop=self.slop,
            allow_headerless=False,  # keep strict; better to fix publishers than guess time
        )
        self.sync.registerCallback(self.synced_cb)

        # --- Debug state ---
        self._sync_count = 0
        self._last_sync_clock_ns = None

        self.get_logger().info(
            f"GpsImuFusion started.\n"
            f"  IMU topic: {self.imu_topic}\n"
            f"  GPS topic: {self.gps_topic}\n"
            f"  queue_size: {self.queue_size}\n"
            f"  slop: {self.slop:.3f}s\n"
            f"  log_every_n: {self.log_every_n}\n"
            f"Waiting for synced message pairs..."
        )

    def _gps_to_local_xy(self, lat_deg: float, lon_deg: float, alt_m: float):
        """Convert lat/lon (deg) to local ENU meters relative to first fix (equirectangular approx)."""
        if not self._origin_set:
            self._lat0 = lat_deg
            self._lon0 = lon_deg
            self._alt0 = alt_m
            self._cos_lat0 = math.cos(math.radians(self._lat0))
            self._origin_set = True
            self.get_logger().info(
                f"Set GPS origin: lat0={self._lat0:.8f}, lon0={self._lon0:.8f}, alt0={self._alt0:.3f}"
            )

        dlat = math.radians(lat_deg - self._lat0)
        dlon = math.radians(lon_deg - self._lon0)

        x_east  = self._R * dlon * self._cos_lat0
        y_north = self._R * dlat
        z_up    = alt_m - self._alt0
        return x_east, y_north, z_up

    @staticmethod
    def _stamp_to_float(stamp) -> float:
        """Convert builtin_interfaces/Time to seconds as float."""
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def synced_cb(self, imu_msg: Imu, gps_msg: NavSatFix):
        """Called when a matched IMU + GPS pair is found within slop window."""
        self._sync_count += 1

        # --- Timestamp sanity ---
        imu_t = self._stamp_to_float(imu_msg.header.stamp)
        gps_t = self._stamp_to_float(gps_msg.header.stamp)
        dt = abs(imu_t - gps_t)

        if self.warn_on_zero_stamp:
            if imu_msg.header.stamp.sec == 0 and imu_msg.header.stamp.nanosec == 0:
                self.get_logger().warn("IMU has zero timestamp (sec=0,nanosec=0).")
            if gps_msg.header.stamp.sec == 0 and gps_msg.header.stamp.nanosec == 0:
                self.get_logger().warn("GPS has zero timestamp (sec=0,nanosec=0).")

        # --- Frame sanity ---
        if self.warn_on_frame_mismatch:
            imu_frame = imu_msg.header.frame_id
            gps_frame = gps_msg.header.frame_id
            # Some GPS drivers leave frame_id empty; only warn if both are set and mismatch.
            if imu_frame and gps_frame and imu_frame != gps_frame:
                self.get_logger().warn(
                    f"Frame mismatch: IMU frame='{imu_frame}' vs GPS frame='{gps_frame}'"
                )

        # --- Debug: sync rate ---
        now_ns = self.get_clock().now().nanoseconds
        sync_period_s = None
        if self._last_sync_clock_ns is not None:
            sync_period_s = (now_ns - self._last_sync_clock_ns) / 1e9
        self._last_sync_clock_ns = now_ns

        # --- Logging (rate-limited by log_every_n) ---
        if (self._sync_count % self.log_every_n) == 0:
            msg = (
                f"[sync #{self._sync_count}] "
                f"IMU={imu_t:.6f}s GPS={gps_t:.6f}s | |Δt|={dt:.4f}s"
            )
            if sync_period_s is not None:
                msg += f" | period={sync_period_s:.3f}s"
            self.get_logger().info(msg)

        # --- Build + publish fused Odometry (local meters ENU) ---
        # Skip if GPS has no fix
        if gps_msg.status.status < 0:
            return

        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.child_frame_id = self.child_frame_id

        # Position: GPS -> local ENU meters
        x, y, z = self._gps_to_local_xy(float(gps_msg.latitude),
                                    float(gps_msg.longitude),
                                    float(gps_msg.altitude))
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = z

        # Orientation: IMU quaternion
        out.pose.pose.orientation = imu_msg.orientation

        # Twist: use IMU angular velocity (optional but better than zeros)
        out.twist.twist.angular = imu_msg.angular_velocity

        # --- Covariance mapping ---
        # Odometry pose.covariance is 6x6 row-major:
        # [x y z roll pitch yaw]
        # GPS gives 3x3 position covariance. Map it into x/y/z block.
        pc = gps_msg.position_covariance
        if len(pc) == 9:
            out.pose.covariance[0]  = pc[0]  # xx
            out.pose.covariance[1]  = pc[1]  # xy
            out.pose.covariance[2]  = pc[2]  # xz
            out.pose.covariance[6]  = pc[3]  # yx
            out.pose.covariance[7]  = pc[4]  # yy
            out.pose.covariance[8]  = pc[5]  # yz
            out.pose.covariance[12] = pc[6]  # zx
            out.pose.covariance[13] = pc[7]  # zy
            out.pose.covariance[14] = pc[8]  # zz

        # IMU orientation covariance is 3x3 for roll/pitch/yaw (if provided).
        oc = imu_msg.orientation_covariance
        if len(oc) == 9 and oc[0] >= 0.0:  # ROS uses -1 to mean "unknown"
            out.pose.covariance[21] = oc[0]  # roll-roll
            out.pose.covariance[22] = oc[1]
            out.pose.covariance[23] = oc[2]
            out.pose.covariance[27] = oc[3]
            out.pose.covariance[28] = oc[4]  # pitch-pitch
            out.pose.covariance[29] = oc[5]
            out.pose.covariance[33] = oc[6]
            out.pose.covariance[34] = oc[7]
            out.pose.covariance[35] = oc[8]  # yaw-yaw

        self.odom_pub.publish(out)


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