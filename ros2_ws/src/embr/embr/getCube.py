import rclpy
from rclpy.node import Node
import time
from dronekit import connect
from msg_interface.msg import GPSAndIMU

class AttitudePublisher(Node):
    def __init__(self):
        super().__init__('attitude_publisher')
        self.publisher_ = self.create_publisher(GPSAndIMU, 'gps_imu', 10)
        self.vehicle = connect("/dev/ttyAMA0", wait_ready=False, baud=57600)
        self.get_logger().info('Attitude Publisher node initialized')

    def publish_attitude(self):
        while True:
            location = self.vehicle.location.global_frame
            attitude = self.vehicle.attitude
            gps_imu_msg = GPSAndIMU()
            gps_imu_msg.lat = int(location.lat * 1e7)
            gps_imu_msg.lon = int(location.lon * 1e7)
            gps_imu_msg.alt = int(location.alt * 1000)
            gps_imu_msg.vel = self.vehicle.groundspeed
            gps_imu_msg.yaw = attitude.yaw
            gps_imu_msg.pitch = attitude.pitch
            gps_imu_msg.roll = attitude.roll
            self.get_logger().info(f'Published Telem Data: Lat: {gps_imu_msg.lat} Lon: {gps_imu_msg.lon} Alt: {gps_imu_msg.alt} Velocity: {gps_imu_msg.vel} Yaw: {gps_imu_msg.yaw} Pitch: {gps_imu_msg.pitch} Roll: {gps_imu_msg.roll}')
            self.publisher_.publish(gps_imu_msg)
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    attitude_publisher = AttitudePublisher()
    attitude_publisher.publish_attitude()
    rclpy.spin(attitude_publisher)
    attitude_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
