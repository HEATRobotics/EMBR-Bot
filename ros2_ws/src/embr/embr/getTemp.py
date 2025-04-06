"""
CURRENTLY ONLY SENDS MOCK DATA
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import smbus
import time
import math

# Constants for Steinhart-Hart / Beta parameter equation
T0 = 298.15  # Reference temperature (25°C in Kelvin)
R0 = 10000   # Reference resistance at T0
B = 3950     # Beta parameter

I2C_ADDRESS = 0x48
ANALOG_CHANNEL = 0x40

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Temperature, 'temperature', 10)
        self.timer = self.create_timer(1, self.publish_temperature)

        # busNum = self.find_device()
        # if busNum != None:
        #     self.bus = smbus.SMBus(busNum)
        # else:
        #     self.get_logger().error("No device found")
        
    
    
    def find_device(self):
        if self.check_device(8):
           return 8
        if self.check_device(7):
            return 7
        return None            

    def check_device(self, busNum):
        try:
            bus = smbus.SMBus(busNum)
            bus.read_byte(I2C_ADDRESS)
            bus.close()
            self.get_logger().info(f"Device found on I2C bus {busNum}")
            return True
        except FileNotFoundError:
            self.get_logger().error(f"I2C bus {busNum} not found")
        except IOError:
            self.get_logger().info(f"No device found at address {hex(I2C_ADDRESS)} on I2C bus {busNum}")
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
        finally:
            if 'bus' in locals():
                bus.close()
        return False

    def read_adc(self, channel):
        self.bus.write_byte(I2C_ADDRESS, channel)
        adc_value = self.bus.read_byte(I2C_ADDRESS)
        return adc_value

    def resistance_to_temperature(self, R):
        temperature = 1 / ((1/T0) + (1/B) * math.log(R / R0)) - 273.15
        return temperature

    def publish_temperature(self):
        # adc_value = self.read_adc(ANALOG_CHANNEL)
        # self.get_logger().info(f"ADC: {adc_value}")

        # voltage = adc_value * 3.3 / 255
        # self.get_logger().info(f"Voltage: {voltage} V")

        # resistance = (voltage * R0) / (3.3 - voltage)
        # self.get_logger().info(f"Resistance: {resistance} Ohms")
        
        # temperature = self.resistance_to_temperature(resistance)
        temperature = 36.10
        self.get_logger().info(f"Temperature: {temperature:.2f} °C")

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
