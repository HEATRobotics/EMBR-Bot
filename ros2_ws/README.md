# ROS 2 workspace (ros2_ws)

ROS 2 workspace containing:
- `src/embr` – main EMBR package with runtime nodes and launch files
- `src/msg_interface` – custom message definitions

## Prerequisites

- ROS 2 Humble installed (use `Tools/Setup-Scripts/setup-all` on Raspberry Pi)
- Python dependencies:
  ```
  pip install -r ./requirements.txt
  ```
  Note: `pyserial` is imported as `serial`.

## Build

`colcon` builds all packages in the workspace.

1. Ensure new nodes are registered in `./src/embr/setup.py` and launches in `./src/embr/launch/embr_launch.py`.
2. Build:
	```
	colcon build
	```
3. Source the overlay:
	```
	source install/setup.bash
	```

## Run

### Using Default Configuration (Real Hardware)

Launch all EMBR nodes with the default config (real hardware sensors):

```
ros2 launch embr embr_launch.py
```

Run a specific node:

```
ros2 run embr getTemp
ros2 run embr getCube
ros2 run embr radio
```

### Using Custom Configuration

For simulation or custom sensor configurations, pass a different config file:

```bash
# Full simulation mode
ros2 launch embr embr_launch.py config_file:=config/sensors_sim.json

# Mixed mode (some real, some sim)
ros2 launch embr embr_launch.py config_file:=config/sensors_mixed.json

# Your own custom config
ros2 launch embr embr_launch.py config_file:=/path/to/custom_config.json
```

Run individual nodes with custom configs:

```bash
ros2 run embr getTemp --ros-args -p config_file:=config/sensors_sim.json
```

### Sensor Configuration

All sensors (temperature, GPS/IMU, radio, thermal camera) use a unified sensor abstraction that supports both real hardware and simulated modes.

Sensors are configured via JSON files in `src/embr/config/`:
- `sensors.json` - Default config for real hardware
- `sensors_sim.json` - Full simulation mode (no hardware required)
- `sensors_mixed.json` - Mixed real/sim example

See [Configuration Guide](src/embr/config/CONFIG.md) for detailed documentation.

### Legacy Nodes

The original nodes are still available:

```bash
ros2 launch embr embr_launch.py
ros2 run embr radio
```

If you run with `sudo`, manually source ROS 2 in that shell:

```
source /opt/ros/humble/setup.bash
```

## Package layout

```
src/
├── embr/
│   ├── embr/            # nodes: getCube.py, getTemp.py, radio.py
│   ├── launch/          # embr_launch.py
│   └── setup.py         # entry points
└── msg_interface/
	 ├── msg/             # custom msgs (e.g., GPSAndIMU.msg)
	 └── package.xml
```

## Nodes

### getCube
Interfaces with Cube Orange flight controller via MAVLink.

### getTemp
Publishes temperature data from serial sensor.

### sendRf
Handles RF communication for telemetry.

### thermalStream
Thermal camera streaming node with HDMI output and intelligent frame publishing.
- Streams color-mapped thermal video over HDMI with direct framebuffer access
- Highlights and annotates hotspots above configurable temperature threshold
- Publishes raw radiometric frames (uint16, Kelvin×100) when vehicle velocity is zero
- All parameters (camera mounting, thresholds, display settings) configurable via JSON
- See [Thermal Camera Documentation](../Documentation/2025/Thermal Camera.md) for details

### hotspotLocator
GPS locator for thermal hotspots using camera geometry and vehicle position.
- Subscribes to thermal radiometric arrays and GPS/IMU data
- Computes GPS coordinates of detected hotspots using camera FOV and mounting angle
- Publishes hotspot locations as PoseStamped messages
- Camera mounting parameters (altitude, pitch) configurable via JSON

## Also see

- Top-level quick start: `../README.md`
- Docs index: `../Documentation/README.md`
- Serial and wiring: `../Documentation/2025/Serial-and-pins.md`
- Start scripts and utilities: `../Tools/Readme.md`
