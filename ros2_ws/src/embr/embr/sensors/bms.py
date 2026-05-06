"""Basic BMS Bluetooth discovery using BlueZ through Bleak."""

from typing import Optional

from .base import Sensor, SensorConfig


class BMS_Handler(Sensor):
    """Base BMS sensor interface."""
    pass


class RealBMSSensor(BMS_Handler):
    """Scan for nearby Bluetooth Low Energy devices."""

    def __init__(self, config: Optional[SensorConfig] = None):
        config = config or SensorConfig(
            mode="real",
            device=None,
            baud=0,
            params={},
        )
        super().__init__(config)
        params = config.params if config and isinstance(config.params, dict) else {}
        self.scan_timeout = float(params.get("scan_timeout", 5.0))
        self.adapter = params.get("adapter")
        self.devices = []

    async def start(self) -> None:
        """Scan for Bluetooth devices and print the result."""
        if self._running:
            return

        self._running = True
        self.devices = await self.scan()
        self.print_devices(self.devices)

    async def scan(self) -> list:
        """Return nearby BLE devices discovered by BlueZ/Bleak."""
        try:
            from bleak import BleakScanner
        except ImportError as exc:
            raise RuntimeError(
                "Bleak is not installed. Install it with `pip install bleak`."
            ) from exc

        scan_kwargs = {"timeout": self.scan_timeout}
        if self.adapter:
            scan_kwargs["adapter"] = self.adapter

        try:
            return await BleakScanner.discover(**scan_kwargs)
        except Exception as exc:
            raise RuntimeError(
                "Bluetooth scan failed. Check that BlueZ is running and the USB "
                "Bluetooth dongle is available, powered on, and not blocked."
            ) from exc

    def read(self) -> dict:
        """Return the most recent scan result in a simple serializable format."""
        if not self._running:
            raise RuntimeError("Sensor not started")

        return {
            "devices": [
                {
                    "name": device.name or "Unknown",
                    "address": device.address,
                }
                for device in self.devices
            ]
        }

    async def stop(self) -> None:
        """Stop the BMS scanner."""
        self._running = False

    @staticmethod
    def print_devices(devices: list) -> None:
        """Print discovered devices to the console."""
        if not devices:
            print("No Bluetooth devices found.")
            return

        print(f"Found {len(devices)} Bluetooth device(s):")
        for index, device in enumerate(devices, start=1):
            name = device.name or "Unknown"
            print(f"{index}. {name} - {device.address}")


class SimBMSSensor(BMS_Handler):
    """Tiny simulated BMS scanner for factory compatibility."""

    def __init__(self, config: Optional[SensorConfig] = None):
        config = config or SensorConfig(
            mode="sim",
            device=None,
            baud=0,
            params={},
        )
        super().__init__(config)
        self.devices = []

    async def start(self) -> None:
        self._running = True
        self.print_devices(self.devices)

    def read(self) -> dict:
        if not self._running:
            raise RuntimeError("Sensor not started")
        return {"devices": []}

    async def stop(self) -> None:
        self._running = False

    @staticmethod
    def print_devices(devices: list) -> None:
        RealBMSSensor.print_devices(devices)
