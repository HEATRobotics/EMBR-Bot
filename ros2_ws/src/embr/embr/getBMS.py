"""
BMS BLE (Battery Management System Bluetooth Low Energy) 
sensor implementation for EMBR Bot.
Supports both real hardware and simulated data for testing.
"""

import asyncio
import inspect

import rclpy
from rclpy.node import Node

from embr.sensors import create_sensor
# import msg_interface.msg as ____ # Undecided if custom message is needed yet


class BMSPublisher(Node):
    def __init__(self):
        super().__init__('bms_publisher')
        
        # Declare parameter for config file path
        self.declare_parameter('config_file', '')
        config_path = self.get_parameter('config_file').value
        
        # Create sensor
        try:
            self.sensor = create_sensor('bms', config_path)
            self._run_sensor_method(self.sensor.start)
            
            mode_type = 'simulated' if 'Sim' in self.sensor.__class__.__name__ else 'real'
            self.get_logger().info(f'BMS sensor initialized in {mode_type} mode (using {mode_type} sensor)')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize sensor: {e}')
            raise
        
        # Create publisher and timer
        # self.publisher_ = self.create_publisher(____, 'bms_data', 10)  # Undecided on message type
        self.timer = self.create_timer(1.0, self.publish_bms_data)

    def publish_bms_data(self):
        """Read BMS data and publish."""
        try:
            bms_data = self.sensor.read()
            
            # Convert bms_data to ROS message format if needed
            # bms_msg = ____  # Undecided on message type
            
            # self.publisher_.publish(bms_msg)
            self.get_logger().info(f'BMS Data: {bms_data}')  # Placeholder logging
        except Exception as e:
            self.get_logger().error(f'Error reading BMS data: {e}')

    def _run_sensor_method(self, method):
        result = method()
        if inspect.isawaitable(result):
            return asyncio.run(result)
        return result

    def destroy_node(self):
        """Cleanup sensor on shutdown."""
        if hasattr(self, 'sensor'):
            self._run_sensor_method(self.sensor.stop)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bms_publisher = BMSPublisher()
    try:
        rclpy.spin(bms_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        bms_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
