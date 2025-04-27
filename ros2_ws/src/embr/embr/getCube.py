import rclpy
from rclpy.node import Node
import time
from dronekit import connect
from msg_interface.msg import Gps

class AttitudePublisher(Node):
    def __init__(self):
        super().__init__('attitude_publisher')
        self.publisher_ = self.create_publisher(Gps, 'gps', 10)
        self.vehicle = connect("/dev/ttyAMA0", wait_ready=False, baud=57600)
        self.get_logger().info('Attitude Publisher node initialized')

    def publish_attitude(self):
        while True:
            location = self.vehicle.location.global_frame
            gps_msg = Gps()
            gps_msg.lat = int(location.lat * 1e7)
            gps_msg.lon = int(location.lon * 1e7)
            gps_msg.alt = int(location.alt * 1000)
            gps_msg.vel = self.vehicle.groundspeed
            self.get_logger().info(f'Published Telem Data: Lat: {gps_msg.lat} Lon: {gps_msg.lon} Alt: {gps_msg.alt} Velocity: {gps_msg.vel}')
            self.publisher_.publish(gps_msg)
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
