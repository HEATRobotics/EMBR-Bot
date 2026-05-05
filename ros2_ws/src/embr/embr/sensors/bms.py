from typing import Optional
from .base import Sensor, SensorConfig


class BMS_Handler(Sensor):
    """Abstract BMS Handler Interface."""
    pass


class RealBMSSensor(BMS_Handler):
    """Real BMS sensor implementation using BLE."""

    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        self.device = config.device if config else None
        self.devices = []
        self.matched_devices = []

    async def start(self) -> None:
        """Scan for nearby BLE devices and report likely BMS matches."""
        if self._running:
            return

        try:
            from bleak import BleakScanner

            self.devices = await BleakScanner.discover(timeout=10.0)
            self.matched_devices = [
                device for device in self.devices
                if device.name and "BMS" in device.name.upper()
            ]

            print("Found BLE devices:")
            for device in self.devices:
                print(f"Address: {device.address} | Name: {device.name} | Details: {device}")

            if self.matched_devices:
                print("Likely BMS devices:")
                for device in self.matched_devices:
                    print(f"Address: {device.address} | Name: {device.name}")
            else:
                print("No BLE device with 'BMS' in its name was found.")
        except Exception as e:
            raise RuntimeError(f"Failed to scan for BMS: {e}")

        self._running = True

    def read(self) -> dict:
        """Return the latest BLE scan result."""
        if not self._running:
            raise RuntimeError("Sensor not started")

        return {
            "device_count": len(self.devices),
            "matched_bms_count": len(self.matched_devices),
            "matched_bms_devices": [
                {"address": device.address, "name": device.name}
                for device in self.matched_devices
            ],
        }

    def stop(self) -> None:
        """Stop BLE scan state."""
        self._running = False


class SimBMSSensor(BMS_Handler):
    """Minimal simulated BMS sensor so package imports still work."""

    def start(self) -> None:
        self._running = True

    def read(self) -> dict:
        if not self._running:
            raise RuntimeError("Sensor not started")

        return {
            "device_count": 1,
            "matched_bms_count": 1,
            "matched_bms_devices": [
                {"address": "SIMULATED", "name": "Sim BMS"}
            ],
        }

    def stop(self) -> None:
        self._running = False
