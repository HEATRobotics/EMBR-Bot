"""Basic BMS Bluetooth discovery using BlueZ through Bleak."""
import argparse
import asyncio
import time
from typing import Optional

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from .base import Sensor, SensorConfig

class BMS_Handler(Sensor):
    """Base BMS sensor interface."""
    pass

class DeviceNotFoundError(Exception):
    pass

class Args(argparse.Namespace):
    """BMS Contract"""
    name: Optional[str]     # BLE Device Name to scan for 
    address: Optional[str]  # BLE MAC Addres
    characteristic: str     # UUID of the BLE Characteristic to read/write# BLE Device name to scan for 
    services: list[str]     # UUIDs of services to discover
    debug:bool              # Verbose BLE output

class RealBMSSensor(BMS_Handler):
    """Scan for nearby Bluetooth Low Energy devices."""

    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        params = config.params if config and isinstance(config.params, dict) else {}

        # Args Setup 
        self.args = Args()
        self.args.name = params.get("name", None)
        self.args.address = params.get("address", "A4:C1:37:03:1E:15")
        self.args.characteristic = params.get("characteristic")
        self.args.services = params.get("services", []])
        self.args.debug = params.get("debug", False)

        self.device = None

    async def start(self) -> None:
        """Connect to Device and Print Name"""
        
        if self._running:
            return

        self._running = True

        queue = asyncio.Queue()
        await self.connect(self.args, queue)

    async def connect(self, args: Args, queue: asyncio.Queue[tuple[float, Optional[bytearray]]]):
        """Async function that takes BLE settings and 'conveyor belt', 
        connects to the BMS, and drops timestamped data onto the belt as it arrives"""

        # Find Device With MAC Address
        if args.address:
            self.device = await BleakScanner.find_device_by_address(
                args.address
            )
            if self.device is None:
                raise DeviceNotFoundError
        elif args.name:
            self.device = await BleakScanner.find_device_by_name(
                args.name
            )
            if self.device is None:
                raise DeviceNotFoundError
        
        async def callback_handler(_: BleakGATTCharacteristic, data: bytearray) -> None:
            await queue.put((time.time(), data))

        # May need to find and log characterisitics if there is multiple
        async def get_characteristic() -> str:
            async with BleakClient(self.device) as client:
                # Find the first characterisitc that supports notify
                notify_char = None
                for service in client.services:
                    self.args.services.append(service)
                    for char in service.characteristics:
                        if "notify" in char.properties:
                            notify_char = char.uuid
                            break
                    if notify_char:
                        break
            if notify_char is None:
                raise ValueError("No notifiable characterisitic found on device")
            return notify_char

        async with BleakClient(self.device) as client:
            notify_char = await get_characteristic()
            await client.start_notify(char_specifier=notify_char, callback=callback_handler)
            await asyncio.sleep(10.0)
            await client.stop_notify(notify_char)
            await queue.put((time.time(),None))

    async def run_queue_consumer(self, queue: asyncio.Queue[tuple[float, Optional[bytearray]]]):
        """Consumer side of the 'conveyor belt', waits for data from run_ble_client"""
        # TODO: Mostly here for debugging, refactor as needed
        while True:
            epoch, data = await queue.get()
            if data is None:
                # Got Disconnection Message from client, disconnecting
                break

    async def stop(self) -> None:
        """Stop the BMS scanner."""
        self._running = False

    @staticmethod
    def print_device(device) -> None:
        """Print discovered devices to the console."""
        # TODO
        pass

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
