"""
Temperature publisher node with sensor abstraction.
Supports both real and simulated sensors.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from embr.sensors import create_sensor, SensorConfig, SensorFactory


class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        
        # Load sensor configuration
        self.declare_parameter('sensor_mode', 'auto')
        self.declare_parameter('sensor_device', '/dev/ttyACM0')
        self.declare_parameter('sensor_baud', 9600)
        self.declare_parameter('config_file', 'config/sensors.json')
        
        mode = self.get_parameter('sensor_mode').value
        device = self.get_parameter('sensor_device').value
        baud = self.get_parameter('sensor_baud').value
        config_file = self.get_parameter('config_file').value
        
        # Try to load from config file first
        try:
            configs = SensorFactory.load_config(config_file)
            config = configs.get('temperature', SensorConfig(mode=mode, device=device, baud=baud))
        except Exception as e:
            self.get_logger().warn(f'Could not load config file: {e}. Using parameters.')
            config = SensorConfig(mode=mode, device=device, baud=baud)
        
        # Create sensor
        try:
            self.sensor = create_sensor('temperature', config)
            self.sensor.start()
            
            sensor_type = 'simulated' if isinstance(self.sensor.__class__.__name__, str) and 'Sim' in self.sensor.__class__.__name__ else 'real'
            self.get_logger().info(f'Temperature sensor initialized in {config.mode} mode (using {sensor_type} sensor)')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize sensor: {e}')
            raise
        
        # Create publisher and timer
        self.publisher_ = self.create_publisher(Temperature, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.read_and_publish)
    
    def read_and_publish(self):
        """Read temperature and publish."""
        try:
            temperature = self.sensor.read()
            self.get_logger().info(f"Temperature: {temperature:.2f}°C")
            
            temperature_msg = Temperature()
            temperature_msg.header.stamp = self.get_clock().now().to_msg()
            temperature_msg.temperature = temperature
            temperature_msg.variance = 0.0
            
            self.publisher_.publish(temperature_msg)
        except Exception as e:
            self.get_logger().error(f'Error reading temperature: {e}')
    
    def destroy_node(self):
        """Cleanup sensor on shutdown."""
        if hasattr(self, 'sensor'):
            self.sensor.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    temperature_publisher = TemperaturePublisher()
    try:
        rclpy.spin(temperature_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        temperature_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
