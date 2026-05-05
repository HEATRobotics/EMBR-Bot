import asyncio
from typing import Optional
from .base import Sensor, SensorConfig


class BMS_Handler(Sensor):
    """Abstract BMS Handler Interface."""
    pass


class RealBMSSensor(BMS_Handler):
    """Real BMS sensor implementation using BLE."""

    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        self.connection = None
        self.adapter = "hci0"

    async def find_bms(self):
        """Scan for nearby BLE devices."""
        from bleak import BleakScanner

        devices = await BleakScanner.discover(
            timeout=10,
            adapter=self.adapter
        )

        return devices

    async def start(self) -> None:
        """Scan for nearby BLE devices so we can identify the BMS."""
        devices = await self.find_bms()

        if not devices:
            raise RuntimeError(
                "No BLE devices found. Make sure the BMS is awake and not connected to another app."
            )

        print("Found BLE devices:")
        for d in devices:
            print(f"Address: {d.address} | Name: {d.name} | Details: {d}")

    async def stop(self):
        """Stop real BMS connection."""
        if self.connection and self.connection.is_connected:
            await self.connection.disconnect()