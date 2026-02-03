"""
Radio communication node with sensor abstraction.
Supports both real and simulated connections.
"""

from math import atan2, degrees, sqrt, pi
import numpy as np
import rclpy
import time
from rclpy.node import Node
from msg_interface.msg import Gps
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from embr.sensors import create_sensor, SensorConfig


class CommSubscriber(Node):
    def __init__(self):
        super().__init__('radio_subscriber')
        
        # --- Add a flag to track if initial message with 4 lat/long points is received ---

        # Declare parameter for config file path
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value
        
        # Create Radio connection
        try:
            self.radio_connection = create_sensor('radio', config_file)
            self.radio_connection.start()
            
            mode_type = 'simulated' if 'Sim' in self.radio_connection.__class__.__name__ else 'real'
            self.get_logger().info(f'Radio connection initialized in {mode_type} mode (using {mode_type} connection)')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Radio: {e}')
            raise

        # --- Wait for initial message with 4 lat/long points before subscribing or sending ---

        # --- Publish lat/long points to new topic

        # --- Once initial message is received, create subscriptions and allow sending ---
        self.subscription = self.create_subscription(Gps, 'gps', self.cube_callback, 10)
        self.subscription_temperature = self.create_subscription(
            Temperature, 'temperature', self.temperature_callback, 10
        )

        self.get_logger().info('Radio Subscriber node initialized')
    
    def temperature_callback(self, msg):
        """Handle temperature messages."""
        try:
            self.get_logger().info(f"Sending temperature: {msg.temperature}")
            self.radio_connection.send_temperature(msg.temperature)
        except Exception as e:
            self.get_logger().error(f'Error sending temperature: {e}')
    
    def cube_callback(self, msg):
        """Handle GPS messages."""
        try:
            lat = msg.lat
            lon = msg.lon
            alt = msg.alt
            vel = int(msg.vel * 100)
            
            self.get_logger().info(f"Sending GPS: Lat: {lat}, Lon: {lon}, Alt: {alt}, Vel: {vel}")
            self.radio_connection.send_gps(lat, lon, alt, vel)
        except Exception as e:
            self.get_logger().error(f'Error sending GPS: {e}')
    
    def destroy_node(self):
        """Cleanup connection on shutdown."""
        if hasattr(self, 'radio_connection'):
            self.radio_connection.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    comm_subscriber = CommSubscriber()
    try:
        rclpy.spin(comm_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        comm_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
