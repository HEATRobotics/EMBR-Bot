# Tools

Utilities for setup and running EMBR.

## Start scripts
- `start-embr.sh` – launches EMBR 

Run from the `Tools/` directory:

```
./start-embr.sh
```

## Setup scripts (Raspberry Pi)

Located in `Tools/Setup-Scripts/`. Use these on a fresh Pi:

- `setup-all` – installs ROS 2 Humble, system deps, Python deps, configures thermal camera, patches DroneKit, and installs raspi-config
- `install-ROS2-Humble` – install only ROS 2
- `install-dependencies` – system + Python deps
- `setup-thermal-camera` – PureThermal 3 config (udev, etc.)

Example:

```
cd Tools/Setup-Scripts
./setup-all
```

For wiring and UART setup, see `Documentation/2025/Serial-and-pins.md`.

## Client-side radio

Experimental code for ground-side interaction with the radio and MAVLink under `Client-Side-Radio/`.

- `mavlink-client-interface.py`, `serial-client-interface.py`, `send.py`

Use these for development and testing only; the canonical runtime is in the ROS 2 nodes under `ros2_ws/`.