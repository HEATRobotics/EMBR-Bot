# EMBR-Bot

A ROS2-based autonomous robot system for wildfire cold trailing operations, running on Raspberry Pi 4 with flight controller integration.

## Overview

EMBR is designed to assist in wildfire response by mapping hot spots that may reignite into fires after a wildfire burns through an area, a procedure known as cold trailing. The system integrates a Cube Orange flight controller, RFD 900x radio telemetry, thermal camera, temperature sensors, and LIDAR to detect and map thermal anomalies and environmental conditions in post-wildfire areas.

### Hardware Platform
- **Main Computer**: Raspberry Pi 4B
- **Operating System**: Ubuntu 22.04.05 LTS 
- **ROS Version**: ROS2 Humble
- **Flight Controller**: Cube Orange
- **Radio**: RFD 900x-US
- **Thermal Camera**: FLIR Lepton 3.1R with PureThermal 3
- **Temperature Sensor**: Arduino-based (via serial)

## Table of Contents
- [Quick Start](#quick-start)
  - [Raspberry Pi Setup](#quick-start)
  - [Docker Quick Start](#docker-setup-for-development-off-raspberry-pi)
- [Simulation](#simulation)
  - [Sensor Simulation Modes](#sensor-simulation-modes)
  - [Running in Simulation](#running-in-simulation)
  - [Configuration Options](#configuration-options)
  - [Simulation Features](#simulation-features)
- [Docker Simulation](#docker-simulation)
  - [Quick Start](#docker-setup-for-development-off-raspberry-pi)
  - [Docker Commands](#docker-documentation)
- [Installation](#installation)
  - [Prerequisites](#prerequisites)
  - [Automated Installation](#automated-installation)
  - [Manual Installation](#manual-installation)
  - [Serial Port Configuration](#serial-port-configuration)
- [Hardware Setup & Wiring](#hardware-setup--wiring)
  - [UART Configuration](#uart-pin-configuration)
  - [RFD 900x Wiring](#raspberry-pi-to-rfd-900x-wiring)
  - [Cube Orange Wiring](#raspberry-pi-to-cube-orange-wiring)
- [Running the System](#running-the-system)
  - [Startup Script](#starting-all-components)
  - [Manual ROS2 Workflow](#manual-ros2-workflow)
- [System Architecture](#system-architecture)
  - [ROS2 Nodes](#system-architecture)
  - [Data Flow](#data-flow)
- [Testing](#testing)
  - [Running Tests](#running-tests)
  - [Test Types](#test-types)
- [Documentation](#documentation)
- [Development](#development)
  - [Project Structure](#project-structure)
  - [Adding New Nodes](#adding-new-nodes)
  - [DroneKit Modification](#cube-orange-dronekit-modification)
- [Contributing](#contributing)
- [Troubleshooting](#troubleshooting)

## Quick Start

For a complete system setup on a fresh Raspberry Pi:

```bash
cd Tools/Setup-Scripts
./setup-all
```

This will install ROS2 Humble, system dependencies, Python dependencies, patch DroneKit for Python 3.10+, configure the thermal camera, and install raspi-config.

**Important:** After installation, you must configure serial ports (see [Serial Port Configuration](#serial-port-configuration) below) before starting the system.

To start all system components:

```bash
cd Tools
./start-embr.sh
```

## Simulation

EMBR-Bot includes a comprehensive sensor abstraction layer that allows you to run the system with **simulated sensors**, enabling development and testing without any physical hardware.

### Sensor Simulation Modes

The system supports three modes for each sensor:

1. **`real`** - Use actual hardware sensors
2. **`sim`** - Use simulated sensors (no hardware required)
3. **`auto`** - Automatically detect hardware; fall back to simulation if unavailable

### Running in Simulation

**Option 1: Full Simulation (All Sensors)**
```bash
export EMBR_SENSOR_MODE=sim
ros2 launch embr embr_launch_v2.py
```

**Option 2: Individual Sensor Control**
```bash
export EMBR_TEMPERATURE_MODE=sim
export EMBR_CUBE_MODE=sim
export EMBR_MAVLINK_MODE=sim
ros2 launch embr embr_launch_v2.py
```

**Option 3: Configuration File**
```bash
ros2 launch embr embr_launch_v2.py config_file:=config/sensors_sim.json
```

**Option 4: Mixed Mode (Some Real, Some Simulated)**
```bash
# Edit config/sensors_mixed.json
export EMBR_TEMPERATURE_MODE=real  # Real Arduino sensor
export EMBR_CUBE_MODE=sim           # Simulated GPS
ros2 launch embr embr_launch_v2.py
```

### Configuration Options

Create custom sensor configurations in JSON format:

**All Simulated (`config/sensors_sim.json`)**
```json
{
  "temperature": {
    "mode": "sim",
    "params": {
      "base_temp": 22.0,
      "variation": 2.0,
      "noise": 0.1
    }
  },
  "cube": {
    "mode": "sim",
    "params": {
      "pattern": "circle",
      "velocity": 5.0,
      "start_lat": 37.7749,
      "start_lon": -122.4194
    }
  },
  "thermal": {"mode": "sim"},
  "mavlink": {"mode": "sim"}
}
```

**Mixed Mode (`config/sensors_mixed.json`)**
```json
{
  "temperature": {
    "mode": "real",
    "device": "/dev/ttyACM0",
    "baud": 9600
  },
  "cube": {"mode": "sim"},
  "thermal": {"mode": "sim"},
  "mavlink": {"mode": "sim"}
}
```

### Simulation Features

**Temperature Sensor**
- Realistic sine wave pattern with noise
- Configurable base temperature, variation, and noise levels
- Default: 22°C ± 2°C with ±0.1°C random noise

**GPS (Cube Orange)**
- Three movement patterns:
  - `circle` - Circular flight pattern (~11m radius)
  - `line` - Linear northward movement
  - `hover` - Stationary with small random variations
- Configurable starting position, velocity, and altitude
- Realistic GPS coordinate updates (latitude/longitude in degrees × 10^7)

**MAVLink Radio**
- Records all transmitted messages for verification
- Supports message injection for testing
- Compatible with real MAVLink message types

## Docker Setup for development off raspberry pi
To start docker and run embr inside container:
```bash
# Build and start
docker compose up -d

# Access container
docker compose exec embr-sim /bin/bash

# Source ROS
source install/setup.bash

# Inside container: Launch all nodes
ros2 launch embr embr_launch_v2.py
```

### Docker Documentation
- **[DOCKER_CHEATSHEET.md](DOCKER_CHEATSHEET.md)** - Quick command reference

## Installation

### Prerequisites
- Raspberry Pi 4B running Ubuntu 22.04 LTS
- Internet connection for downloading dependencies

### Automated Installation

Run the master setup script to install all components:

```bash
cd Tools/Setup-Scripts
./setup-all
```

The setup script will:
1. Install ROS2 Humble
2. Install system dependencies (python3-pip, libuvc-dev)
3. Install Python dependencies (ros2_ws/requirements.txt)
4. Patch DroneKit for Python 3.10+ compatibility
5. Configure thermal camera (PureThermal 3) with udev rules
6. Install raspi-config for serial configuration

### Manual Installation

If you prefer to install components individually:

**1. Install ROS2 Humble:**
```bash
cd Tools/Setup-Scripts
./install-ROS2-Humble
```

**2. Install Dependencies:**
```bash
cd Tools/Setup-Scripts
./install-dependencies
```

**3. Setup Thermal Camera:**
```bash
cd Tools/Setup-Scripts
./setup-thermal-camera
```

**4. Install Python Requirements:**
```bash
cd ros2_ws
pip install -r requirements.txt
```

**5. Patch DroneKit for Python 3.10+:**

Follow the steps in [Cube Orange DroneKit Modification](#cube-orange-dronekit-modification) below to update the `dronekit` package for Python 3.10+ compatibility.

### Serial Port Configuration

**This step is required for both automated and manual installation.**

The system requires two UART connections. Follow these steps to enable them:

**1. Run the configuration tool:**

```bash
sudo raspi-config
```

**2. Navigate to:**

**Interface Options → Serial Port**

- Disable login shell over serial
- Enable serial interface

**3. Enable second UART manually:**

```bash
sudo su
cd /boot/firmware
nano config.txt
```

Add or confirm:

```
# Enable the serial pins
enable_uart=1
dtoverlay=uart2
dtoverlay=disable-bt
```

Save and exit, then reboot.

**4. Verify configuration:**

After rebooting, verify with `ls -l /dev/serial*` - you should see two devices.

## Hardware Setup & Wiring

### UART Pin Configuration

| UART Interface | TXD Pin | RXD Pin | CTS Pin | RTS Pin | Device         |
|----------------|---------|---------|---------|---------|----------------|
| UART0/ttyAMA0  | GPIO14  | GPIO15  | GPIO16  | GPIO17  | Cube Orange    |
| UART2/ttyAMA1  | GPIO0   | GPIO1   | N/A     | N/A     | RFD 900x Radio |

### Raspberry Pi to RFD 900x Wiring

| Raspberry Pi Pin   | RFD 900x Pin  |
|--------------------|---------------|
| Pin 4 (5V)         | Pin 4 (5V)    |
| Pin 6 (GND)        | Pin 1 (GND)   |
| Pin 27 (GPIO0/TX)  | Pin 7 (RX)    |
| Pin 28 (GPIO1/RX)  | Pin 9 (TX)    |

### Raspberry Pi to Cube Orange Wiring

| Cube Orange TELEM Pin | Signal Name    | Raspberry Pi GPIO | RPi Physical Pin |
|:---------------------:|:--------------:|:-----------------:|:----------------:|
| 1                     | 5V (VCC)       | 5V Power          | Pin 2            |
| 2                     | TX (from Cube) | RXD (GPIO15)      | Pin 10           |
| 3                     | RX (to Cube)   | TXD (GPIO14)      | Pin 8            |
| 4                     | CTS (from Cube)| RTS (GPIO17)      | Pin 11           |
| 5                     | RTS (to Cube)  | CTS (GPIO16)      | Pin 36           |
| 6                     | GND            | Ground            | Pin 14           |

### Additional Connections
- **Temperature Sensor**: `/dev/ttyACM0` (Arduino via USB)
- **Thermal Camera**: USB connection with udev rule creating `/dev/pt3` symlink

For detailed pinout diagrams, see [Serial and Pins Documentation](Documentation/2025/Serial-and-pins.md).

## Running the System

### Starting All Components

Use the provided startup script to launch all ROS2 nodes:

```bash
cd Tools
./start-embr.sh
```

### Manual ROS2 Workflow

If you need to build and run manually:

**1. Build the ROS2 workspace:**
```bash
cd ros2_ws
colcon build
source install/setup.bash
```

**2. Launch all nodes:**
```bash
ros2 launch embr embr_launch.py
```

**3. Run individual nodes:**
```bash
ros2 run embr getCube      # Cube Orange telemetry
ros2 run embr getTemp      # Temperature sensor
ros2 run embr sendRf       # Radio transmission
```

### Important Notes
- Some nodes require superuser permissions for serial access
- If running with `sudo`, manually source ROS2: `source /opt/ros/humble/setup.bash`
- The system automatically pulls latest changes on startup via `git pull`

## System Architecture

The EMBR-Bot system consists of three main ROS2 nodes:

### 1. getCube Node
- **Purpose**: Reads telemetry from Cube Orange flight controller
- **Device**: `/dev/ttyAMA0` (UART0)
- **Published Topic**: `gps_imu` (GPS location, altitude, velocity, yaw, pitch, roll)
- **Data**: Latitude, Longitude, Altitude, Ground Speed, Yaw, Pitch, Roll

### 2. getTemp Node
- **Purpose**: Reads temperature data from Arduino sensor
- **Device**: `/dev/ttyACM0` (USB Serial)
- **Published Topic**: `temperature` (Temperature readings)
- **Update Rate**: 1 Hz

### 3. sendRf Node
- **Purpose**: Transmits data via RFD 900x radio using MAVLink protocol
- **Device**: `/dev/ttyAMA1` (UART2)
- **Subscribed Topics**: `gps_imu`, `temperature`, `/pointcloud` (LIDAR)
- **Protocol**: MAVLink v2.0
- **Features**: 
  - Transmits GPS and temperature data
  - Processes LIDAR point cloud into 72 sectors
  - Sends distance obstacles over radio link

### Data Flow
```
Cube Orange → getCube → gps topic → sendRf → RFD 900x → Ground Station
Arduino → getTemp → temperature topic → sendRf → RFD 900x → Ground Station
LIDAR → /pointcloud topic → sendRf → RFD 900x → Ground Station
```

## Documentation

Comprehensive documentation is available in the `Documentation/2025/` directory:

- **[Overview and Basics](Documentation/2025/Overview-and-basics.md)** - System overview, OS information, and ROS2 setup
- **[Serial and Pins](Documentation/2025/Serial-and-pins.md)** - Complete wiring diagrams and serial configuration
- **[Cube Orange Setup](Documentation/2025/Cube%20Orange.md)** - Flight controller configuration and motor setup
- **[ROS2 Workspace Guide](ros2_ws/README.md)** - Detailed ROS2 build and run instructions

### Legacy Documentation
Previous year documentation is preserved in `Documentation/2023-24/` including KR260 FPGA development work.

## Development

### Project Structure

```
EMBR-Bot/
├── ros2_ws/                    # ROS2 workspace
│   ├── src/
│   │   ├── embr/              # Main EMBR package
│   │   │   ├── embr/          # Python nodes
│   │   │   │   ├── getCube.py
│   │   │   │   ├── getTemp.py
│   │   │   │   └── sendRf.py
│   │   │   ├── launch/        # Launch files
│   │   │   └── setup.py       # Package configuration
│   │   └── msg_interface/     # Custom message definitions
│   └── requirements.txt       # Python dependencies
├── Tools/
│   ├── Setup-Scripts/         # Installation scripts
│   └── start-embr.sh          # EMBR startup
├── Documentation/
│   └── 2025/                  # Current documentation
└── temp_sensor/               # Arduino temperature sensor code
```

### Adding New Nodes

1. Create your node in `ros2_ws/src/embr/embr/`
2. Add entry point to `ros2_ws/src/embr/setup.py`
3. Add node to `ros2_ws/src/embr/launch/embr_launch.py`
4. Rebuild: `colcon build`
5. Source: `source install/setup.bash`

### Cube Orange DroneKit Modification

DroneKit requires a patch for Python 3.10+. Edit the DroneKit `__init__.py` file:

Note: This patch is applied automatically when running `Tools/Setup-Scripts/setup-all`.

```python
# Change:
import collections
# To:
import collections.abc as collections
from collections.abc import MutableMapping

# Change:
class Parameters(collections.MutableMapping, HasObservers)
# To:
class Parameters(MutableMapping, HasObservers)
```

Location: `~/.local/lib/python3.10/site-packages/dronekit/__init__.py`

## Testing

### Running Tests

The EMBR-Bot project includes comprehensive tests for the sensor abstraction layer and ROS2 nodes.

**Run all tests:**
```bash
cd ros2_ws
source install/setup.bash
colcon test
```

**View test results:**
```bash
colcon test-result --verbose
```

**Run specific test file:**
```bash
pytest src/embr/test/test_sensors.py -v
```

### Test Types

**1. Unit Tests (`test/test_sensors.py`)**
- Tests sensor implementations (Temperature, GPS, Thermal, MAVLink)
- Tests simulated sensor behavior
- Tests sensor factory and configuration loading
- Run with: `colcon test` or `pytest`

**2. Example Code (`examples/sensor_testing_examples.py`)**
- Demonstrates sensor usage patterns
- Shows configuration options
- Not a test - educational examples
- Run with: `python3 src/embr/examples/sensor_testing_examples.py`

## Contributing

When contributing to this repository:
1. Test all changes on actual hardware when possible
2. Update relevant documentation in `Documentation/2025/`
3. Ensure ROS2 nodes follow the established patterns
4. Verify serial communications don't conflict
5. Run tests before submitting: `colcon test`

## License

Components of this project use various licenses:
- EMBR package: Apache-2.0
- See individual component LICENSE files for details

## Troubleshooting

**Serial Port Permissions:**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

**ROS2 Not Found:**
```bash
source /opt/ros/humble/setup.bash
# Or add to ~/.bashrc
```

**Multiple Serial Devices:**
- Verify with `ls -l /dev/serial*` and `ls -l /dev/tty*`
- Check udev rules in `/etc/udev/rules.d/`

**Build Failures:**
```bash
cd ros2_ws
rm -rf build/ install/ log/
colcon build
```

---

**HEAT Robotics** 
