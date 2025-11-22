"""
Cube Orange GPS publisher node with sensor abstraction.
Supports both real and simulated sensors.
"""

import rclpy
from rclpy.node import Node
import time
from msg_interface.msg import Gps
from embr.sensors import create_sensor, SensorConfig, SensorFactory


class AttitudePublisher(Node):
    def __init__(self):
        super().__init__('attitude_publisher')
        
        # Load sensor configuration
        self.declare_parameter('sensor_mode', 'auto')
        self.declare_parameter('sensor_device', '/dev/ttyAMA0')
        self.declare_parameter('sensor_baud', 57600)
        self.declare_parameter('config_file', 'config/sensors.json')
        
        mode = self.get_parameter('sensor_mode').value
        device = self.get_parameter('sensor_device').value
        baud = self.get_parameter('sensor_baud').value
        config_file = self.get_parameter('config_file').value
        
        # Try to load from config file first
        try:
            configs = SensorFactory.load_config(config_file)
            config = configs.get('cube', SensorConfig(mode=mode, device=device, baud=baud))
        except Exception as e:
            self.get_logger().warn(f'Could not load config file: {e}. Using parameters.')
            config = SensorConfig(mode=mode, device=device, baud=baud)
        
        # Create sensor
        try:
            self.sensor = create_sensor('cube', config)
            self.sensor.start()
            
            sensor_type = 'simulated' if 'Sim' in self.sensor.__class__.__name__ else 'real'
            self.get_logger().info(f'Cube sensor initialized in {config.mode} mode (using {sensor_type} sensor)')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize sensor: {e}')
            raise
        
        # Create publisher and timer
        self.publisher_ = self.create_publisher(Gps, 'gps', 10)
        self.timer = self.create_timer(1.0, self.publish_attitude)
    
    def publish_attitude(self):
        """Read GPS data and publish."""
        try:
            gps_data = self.sensor.read()
            
            gps_msg = Gps()
            gps_msg.lat = gps_data.lat
            gps_msg.lon = gps_data.lon
            gps_msg.alt = gps_data.alt
            gps_msg.vel = gps_data.vel
            
            self.get_logger().info(
                f'GPS: Lat: {gps_msg.lat} Lon: {gps_msg.lon} '
                f'Alt: {gps_msg.alt} Vel: {gps_msg.vel:.2f}'
            )
            
            self.publisher_.publish(gps_msg)
        except Exception as e:
            self.get_logger().error(f'Error reading GPS: {e}')
    
    def destroy_node(self):
        """Cleanup sensor on shutdown."""
        if hasattr(self, 'sensor'):
            self.sensor.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    attitude_publisher = AttitudePublisher()
    try:
        rclpy.spin(attitude_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        attitude_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
