"""
BMS BLE (Battery Management System Bluetooth Low Energy) 
sensor implementation for EMBR Bot.
Supports both real hardware and simulated data for testing.
"""

import asyncio
import inspect

import rclpy
from rclpy.node import Node

from embr.sensors import create_sensor, SensorConfig
# import msg_interface.msg as ____ # Undecided if custom message is needed yet


class BMSPublisher(Node):
    def __init__(self):
        super().__init__('bms_publisher')

        # Declare parameter for config file path
        self.declare_parameter('config_file', '')
        config_path = self.get_parameter('config_file').value

        self.sensor = None

        # Create sensor
        try:
            self.sensor = create_sensor('bms', config_path)

            mode_type = 'simulated' if 'Sim' in self.sensor.__class__.__name__ else 'real'
            self.get_logger().info(
                f'BMS sensor created in {mode_type} mode using {self.sensor.__class__.__name__}'
            )

        except Exception as e:
            self.get_logger().error(f'Failed to create sensor: {e}')
            raise

        # Create publisher and timer
        # self.publisher_ = self.create_publisher(____, 'bms_data', 10)
        self.timer = self.create_timer(1.0, self.publish_bms_data)

    async def start_sensor(self):
        """
        Start the BMS sensor.

        RealBMSSensor.start() is async because Bleak scanning is async.
        SimBMSSensor.start() may be normal sync code.
        This supports both.
        """
        if not self.sensor:
            raise RuntimeError("Sensor was not created.")

        try:
            result = self.sensor.start()

            if inspect.isawaitable(result):
                await result

            self.get_logger().info("BMS sensor started.")

        except Exception as e:
            self.get_logger().error(f"Failed to start BMS sensor: {e}")
            raise

    async def connect_bms(self):
        """
        Optional direct BLE scan test.
        You may not need this if RealBMSSensor.start() already scans.
        """
        from bleak import BleakScanner

        print("Scanning for BLE devices...")
        devices = await BleakScanner.discover(timeout=10, adapter="hci0")

        if not devices:
            print("No BLE devices found.")
            return

        for d in devices:
            print(f"Address: {d.address}")
            print(f"Name: {d.name}")
            print(f"Details: {d}")
            print("-" * 40)

    def publish_bms_data(self):
        """Read BMS data and publish."""
        pass

    def destroy_node(self):
        """Cleanup sensor on shutdown."""
        if hasattr(self, 'sensor') and self.sensor:
            try:
                result = self.sensor.stop()

                # Do not await here because destroy_node is sync.
                # For now, ignore async stop during early BLE scanning tests.
                if inspect.isawaitable(result):
                    self.get_logger().warn(
                        "Sensor stop() is async and was not awaited during shutdown."
                    )

            except Exception as e:
                self.get_logger().warn(f"Failed to stop BMS sensor cleanly: {e}")

        super().destroy_node()


async def main(args=None):
    rclpy.init(args=args)

    bms_publisher = BMSPublisher()

    try:
        await bms_publisher.start_sensor()
        rclpy.spin(bms_publisher)

    except KeyboardInterrupt:
        pass

    finally:
        bms_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())