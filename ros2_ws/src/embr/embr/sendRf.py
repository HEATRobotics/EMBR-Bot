from math import atan2, degrees, sqrt, pi
import numpy as np
import rclpy
import time
from rclpy.node import Node
from msg_interface.msg import Gps
from pymavlink.dialects.v20 import common as mavlink2
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
        
        self.mavlink_connection.mav = mavlink2.MAVLink(self.mavlink_connection)
        
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

    
    def lidar_callback(self, msg):
        self.get_logger().info('Lidar Callback Called')
        
        # Create a set to track which sectors are updated
        updated_sectors = set()

        # Process the incoming point cloud
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        
        for (x, y, z) in points:
            distance = math.sqrt(x**2 + y**2)
            angle = math.atan2(y, x)
            if angle < 0:
                angle += 2 * math.pi
            sector_idx = int(angle / self.sector_size_rad)
            distance_cm = int(distance * 100)

            if 0 <= sector_idx < self.num_sectors:
                # Only update if not max distance
                if distance_cm < 10000: 
                    self.previous_sector_distances[sector_idx] = distance_cm
                    updated_sectors.add(sector_idx)

        # Use current time since boot
        boot_time_sec = time.monotonic()
        time_usec = int(boot_time_sec * 1e6)

        # Send MAVLink message with updated distances
        self.mavlink_connection.mav.obstacle_distance_send(
            time_usec=time_usec,
            sensor_type=mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            distances=self.previous_sector_distances,
            increment=5,  # degrees per sector
            min_distance=20,
            max_distance=10000,
            increment_f=5.0 * math.pi/180.0,
            angle_offset=0.0,
            frame=mavutil.mavlink.MAV_FRAME_BODY_FRD
        )

        self.log_obstacle_distance(
            time_usec=time_usec,
            sensor_type=mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            distances=self.previous_sector_distances,
            increment=5,
            min_distance=20,
            max_distance=10000,
            increment_f=5.0 * math.pi/180.0,
            angle_offset=0.0,
            frame=mavutil.mavlink.MAV_FRAME_BODY_FRD
        )

        self.get_logger().info(f"Updated sectors: {sorted(list(updated_sectors))}")



    def log_obstacle_distance(self, time_usec, sensor_type, distances, increment, min_distance, max_distance, increment_f, angle_offset, frame):
        distance_summary = distances[:10]  # Only print first 10 values to keep log readable
        distance_summary_str = ', '.join(str(d) for d in distance_summary)
        if len(distances) > 10:
            distance_summary_str += ", ..."

        self.get_logger().info(
            "\n--- OBSTACLE_DISTANCE MAVLink Packet ---\n"
            f"Time (boot, usec): {time_usec}\n"
            f"Sensor Type: {sensor_type} ({self.sensor_type_to_string(sensor_type)})\n"
            f"Distances (cm): [{distance_summary_str}]\n"
            f"Number of Distance Values: {len(distances)}\n"
            f"Increment: {increment} deg\n"
            f"Increment_f: {increment_f:.5f} rad\n"
            f"Min Distance: {min_distance} cm\n"
            f"Max Distance: {max_distance} cm\n"
            f"Angle Offset: {angle_offset} rad\n"
            f"Frame: {frame} ({self.frame_to_string(frame)})\n"
            "----------------------------------------"
        )

    def sensor_type_to_string(self, sensor_type):
        types = {
            0: "LASER",
            1: "ULTRASOUND",
            2: "INFRARED",
            3: "RADAR",
            4: "UNKNOWN"
        }
        return types.get(sensor_type, "UNDEFINED")

    def frame_to_string(self, frame_id):
        frame_names = {
            0: "GLOBAL",
            1: "LOCAL_NED",
            2: "LOCAL_ENU",
            3: "GLOBAL_RELATIVE_ALT",
            4: "LOCAL_OFFSET_NED",
            5: "BODY_NED",
            6: "BODY_OFFSET_NED",
            7: "GLOBAL_TERRAIN_ALT",
            8: "BODY_FRD",
            12: "BODY_FRD",  # Confirmed for MAV_FRAME_BODY_FRD
        }
        return frame_names.get(frame_id, f"Unknown ({frame_id})")



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
