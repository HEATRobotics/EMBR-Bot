"""
MAVLink communication node with sensor abstraction.
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

from embr.sensors import create_sensor, SensorConfig, SensorFactory


class CommSubscriber(Node):
    def __init__(self):
        super().__init__('mavlink_subscriber')
        
        # Load sensor configuration
        self.declare_parameter('sensor_mode', 'auto')
        self.declare_parameter('sensor_device', '/dev/ttyAMA1')
        self.declare_parameter('sensor_baud', 57600)
        self.declare_parameter('config_file', 'config/sensors.json')
        
        mode = self.get_parameter('sensor_mode').value
        device = self.get_parameter('sensor_device').value
        baud = self.get_parameter('sensor_baud').value
        config_file = self.get_parameter('config_file').value
        
        # Try to load from config file first
        try:
            configs = SensorFactory.load_config(config_file)
            config = configs.get('mavlink', SensorConfig(mode=mode, device=device, baud=baud))
        except Exception as e:
            self.get_logger().warn(f'Could not load config file: {e}. Using parameters.')
            config = SensorConfig(mode=mode, device=device, baud=baud)
        
        # Create MAVLink connection
        try:
            self.mavlink_connection = create_sensor('mavlink', config)
            self.mavlink_connection.start()
            
            conn_type = 'simulated' if 'Sim' in self.mavlink_connection.__class__.__name__ else 'real'
            self.get_logger().info(f'MAVLink connection initialized in {config.mode} mode (using {conn_type} connection)')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MAVLink: {e}')
            raise
        
        # Create subscriptions
        self.subscription = self.create_subscription(Gps, 'gps', self.cube_callback, 10)
        self.subscription_temperature = self.create_subscription(
            Temperature, 'temperature', self.temperature_callback, 10
        )
        
        # Create publisher for received data (optional)
        self.received_data_publisher = self.create_publisher(String, 'mavlink_received', 10)
        
        # Timer to read MAVLink messages
        self.timer = self.create_timer(0.1, self.read_mavlink_messages)
        
        self.get_logger().info('Mavlink Subscriber node initialized')
    
    def temperature_callback(self, msg):
        """Handle temperature messages."""
        try:
            self.get_logger().info(f"Sending temperature: {msg.temperature}")
            self.mavlink_connection.send_temperature(msg.temperature)
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
            self.mavlink_connection.send_gps(lat, lon, alt, vel)
        except Exception as e:
            self.get_logger().error(f'Error sending GPS: {e}')
    
    def read_mavlink_messages(self):
        """Read incoming MAVLink messages."""
        try:
            msg = self.mavlink_connection.read()
            if msg:
                self.get_logger().info(f"Received MAVLink message: {msg}")
                
                # Handle different message types
                if isinstance(msg, dict):
                    msg_type = msg.get('type', 'UNKNOWN')
                    
                    if msg_type == "STATUSTEXT":
                        text = msg.get('text', '')
                        received_msg = String()
                        received_msg.data = f"Received STATUSTEXT: {text}"
                        self.received_data_publisher.publish(received_msg)
                    
                    elif msg_type == "NAMED_VALUE_FLOAT":
                        name = msg.get('name', '')
                        value = msg.get('value', 0.0)
                        received_msg = String()
                        received_msg.data = f"Received {name}: {value}"
                        self.received_data_publisher.publish(received_msg)
                
                # Handle real MAVLink messages (from pymavlink)
                elif hasattr(msg, 'get_type'):
                    msg_type = msg.get_type()
                    
                    if msg_type == "STATUSTEXT":
                        text = msg.text.decode().strip('\x00')
                        received_msg = String()
                        received_msg.data = f"Received STATUSTEXT: {text}"
                        self.received_data_publisher.publish(received_msg)
                    
                    elif msg_type == "NAMED_VALUE_FLOAT":
                        name = msg.name.decode().strip('\x00')
                        value = msg.value
                        received_msg = String()
                        received_msg.data = f"Received {name}: {value}"
                        self.received_data_publisher.publish(received_msg)
        
        except Exception as e:
            self.get_logger().error(f'Error reading MAVLink: {e}')
    
    def destroy_node(self):
        """Cleanup connection on shutdown."""
        if hasattr(self, 'mavlink_connection'):
            self.mavlink_connection.stop()
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
