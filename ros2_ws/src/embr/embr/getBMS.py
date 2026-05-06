"""Console entry point for a basic BMS Bluetooth scan."""

import asyncio

from embr.sensors.bms import RealBMSSensor


async def scan_bluetooth_devices() -> None:
    """Run one Bluetooth scan and print discovered devices."""
    sensor = RealBMSSensor()
    try:
        print("Scanning for Bluetooth devices with BlueZ/Bleak...")
        await sensor.start()
    finally:
        await sensor.stop()


def main(args=None) -> None:
    """ROS console-script entry point."""
    asyncio.run(scan_bluetooth_devices())


if __name__ == "__main__":
    main()
