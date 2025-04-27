import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import time
import re
import serial

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Temperature, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.read_serial_data)
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        time.sleep(2)

    def read_serial_data(self):
        raw = self.ser.readline().decode('utf-8', errors='ignore').strip()

        match = re.search(r"[-+]?\d*\.\d+|\d+", raw)  # Match a float or int
        if match:
            temperature = float(match.group())
            self.get_logger().info(f"Received: {temperature:.2f}")
            temperature_msg = Temperature()
            temperature_msg.header.stamp = self.get_clock().now().to_msg()
            temperature_msg.temperature = temperature
            temperature_msg.variance = 0.0
            
            self.publisher_.publish(temperature_msg)
  

def main(args=None):
    rclpy.init(args=args)
    temperature_publisher = TemperaturePublisher()
    rclpy.spin(temperature_publisher)
    temperature_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
