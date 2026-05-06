"""Console entry point for a basic BMS Bluetooth scan."""
import asyncio
import rclpy
from rclpy.node import Node
from embr.sensors.bms import RealBMSSensor, SimBMSSensor
from embr.sensors import create_sensor, SensorConfig
from msg_interface.msg import BMS


class BMSCommunication(Node):
    def __init__(self):
        super().__init__("bms_publisher")

        # Declare parameter for config file
        self.declare_parameter('config_file', '')
        config_path = self.get_parameter('config_file').value

        # Create Sensor — do NOT call async start() here; call it later
        try:
            self.sensor = create_sensor('bms', config_path)
            mode_type = 'simulated' if 'Sim' in self.sensor.__class__.__name__ else 'real'
            self.get_logger().info(
                f'Cube sensor initialized in {mode_type} mode (using {mode_type} sensor)'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to initialize sensor: {e}')
            raise

        # Create publisher
        self.publisher_ = self.create_publisher(BMS, 'bms', 10)
        self.timer = self.create_timer(1.0, self.publish_bms_data)

    def publish_bms_data(self):
        """Timer callback — publish latest BMS data."""
        try:
            data = self.sensor.read()
            msg = BMS()
            # TODO: populate msg fields from data
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to read BMS data: {e}')


async def start_sensor_and_spin(node: BMSCommunication) -> None:
    """Start the async sensor, then hand off to ROS spinning."""
    try:
        await node.sensor.start()
    except RuntimeError as e:
        node.get_logger().error(f'Bluetooth scan failed: {e}')
        # Continue anyway — node will keep trying via timer or you can raise here


def main(args=None) -> None:
    """ROS console-script entry point."""
    rclpy.init(args=args)
    node = BMSCommunication()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        # Run the async sensor start first
        loop.run_until_complete(start_sensor_and_spin(node))

        # Now spin the ROS node normally
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        loop.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()