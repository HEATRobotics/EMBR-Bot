from math import atan2, degrees, sqrt
import numpy as np
import rclpy
import time
from rclpy.node import Node
from msg_interface.msg import Gps
from pymavlink import mavutil
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32
from std_msgs.msg import String  # Example for publishing received data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class CommSubscriber(Node):
    def __init__(self):
        super().__init__('mavlink_subscriber')
        self.subscription = self.create_subscription(Gps, 'gps', self.cube_callback, 10)
        self.subscription_temperature = self.create_subscription(Temperature, 'temperature', self.temperature_callback, 10)
        self.subscription   # prevent unused variable warning
        self.mavlink_connection = mavutil.mavserial(device='/dev/serial0', baud=57600)
        self.get_logger().info('Mavlink Subscriber node initialized')

        # Lidar Subscription:
        self.subscription_pointcloud = self.create_subscription(
            PointCloud2,
            '/pointcloud',
            self.lidar_callback,
            10
        )


        # Create a publisher for any received data (example: String message)
        #self.received_data_publisher = self.create_publisher(String, 'mavlink_rx', 3)

        # Create a timer to periodically check for incoming messages
        #self.timer = self.create_timer(0.01, self.read_mavlink_messages) # Check every 100ms

    def temperature_callback(self, msg):
        timems = int((time.time() - time.mktime(time.gmtime(0))) * 1000) % 4294967296
        self.get_logger().info(f"Temperature: {msg.temperature}")
        self.mavlink_connection.mav.named_value_float_send(
            time_boot_ms = timems,
            name = b'temp',
            value = msg.temperature)

    def lidar_callback(self, msg: PointCloud2):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        distances = [0] * 72  # 72 bins over 360°
        counts = [0] * 72

        for (x, y, z) in points:
            angle = (degrees(atan2(y, x)) + 360) % 360  # [0, 360)
            index = int(angle // 5)  # 5° resolution
            distance = sqrt(x**2 + y**2)
            distance_cm = int(distance * 100)

            # Accumulate for averaging
            distances[index] += distance_cm
            counts[index] += 1

        # Average distances
        for i in range(72):
            if counts[i] > 0:
                distances[i] = distances[i] // counts[i]
            else:
                distances[i] = 0xFFFF  # Indicate "no data"

        timems = int((time.time() - time.mktime(time.gmtime(0))) * 1000) % 4294967296

        self.mavlink_connection.mav.obstacle_distance_send(
            time_boot_ms=timems,
            sensor_type=1,  # MAV_DISTANCE_SENSOR_LASER
            distances=distances,
            increment=5,  # degrees between measurements
            min_distance=30,  # e.g., 30 cm
            max_distance=4000,  # 40 meters
            increment_f=float(5.0),
            angle_offset=0.0,
            frame=8  # MAV_FRAME_BODY_FRD (forward-right-down), change as needed
        )

        self.get_logger().info(f"Published OBSTACLE_DISTANCE with {sum(c > 0 for c in counts)} populated bins")



    def cube_callback(self, msg):
        lat = msg.lat
        lon = msg.lon
        alt = msg.alt
        vel = int(msg.vel * 100)
        timems = int((time.time() - time.mktime(time.gmtime(0))) * 1000) % 4294967296
        self.mavlink_connection.mav.global_position_int_send(timems, lat, lon, alt, 0, vel, 0, 0, 0)

    def read_mavlink_messages(self):
        msg = self.mavlink_connection.recv_match(blocking=False)
        if msg:
            self.get_logger().info(f"Received MAVLink message: {msg}")
            # Example: If you expect to receive a STATUSTEXT message
            if msg.get_type() == "STATUSTEXT":
                text = msg.text.decode().strip('\x00')
                received_msg = String()
                received_msg.data = f"Received STATUSTEXT: {text}"
                self.received_data_publisher.publish(received_msg)
            elif msg.get_type() == "NAMED_VALUE_FLOAT":
                print(f"Received {message.name}: {message.value}")
                name = msg.name.decode().strip('\x00')
                value = msg.value
                received_msg = String()
                received_msg.data = f"Received {name}: {value}"
                self.received_data_publisher.publish(received_msg)
            # Add more conditions here to handle other expected message types
            # elif msg.get_type() == "YOUR_MESSAGE_TYPE":
            #     # Extract data from the message and publish it as a ROS2 message

def main(args=None):
    rclpy.init(args=args)
    comm_subscriber = CommSubscriber()
    rclpy.spin(comm_subscriber)
    comm_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
