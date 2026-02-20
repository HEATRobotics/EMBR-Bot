# EMBR Sensor Configuration

This directory contains sensor configuration files for the EMBR robot system.

## Configuration Files

### `sensors.json` (Default)
The default configuration file used by all nodes. This is configured for **real hardware** with the following settings:

- **Temperature Sensor**: `/dev/ttyACM0` @ 9600 baud
- **Cube Orange (GPS)**: `/dev/ttyAMA0` @ 57600 baud  
- **MAVLink Radio**: `/dev/ttyAMA1` @ 57600 baud
- **Thermal Camera**: Lepton 3.1R with default mounting and display settings

All sensors are set to `"mode": "real"` to use actual hardware connections.

### `sensors_sim.json` (Full Simulation)
Complete simulation configuration for testing without any hardware. All sensors generate synthetic data.

### `sensors_mixed.json` (Mixed Mode Example)
Example showing how to mix real and simulated sensors (e.g., real temperature sensor with simulated GPS).

## Configuration Structure

Each sensor configuration has the following structure:

```json
{
  "sensor_name": {
    "mode": "real" | "sim",
    "device": "/dev/ttyXXX",  // Required for real mode
    "baud": 9600,              // Required for real mode
    "params": {                // Optional, for simulation parameters
      "param_name": value
    }
  }
}
```

### Modes

- **`real`**: Use actual hardware sensor
- **`sim`**: Use simulated sensor that generates synthetic data

## Usage

### Using Default Configuration

Simply run nodes without any parameters to use `config/sensors.json`:

```bash
# Single node
ros2 run embr getTemp

# All nodes via launch file
ros2 launch embr embr_launch.py
```

### Using Custom Configuration

Pass a custom config file to override the default:

```bash
# Single node with simulation config
ros2 run embr getTemp --ros-args -p config_file:=src/embr/config/sensors_sim.json

# All nodes with simulation config via launch file
ros2 launch embr embr_launch.py config_file:=config/sensors_sim.json

# Use an external config file
ros2 launch embr embr_launch.py config_file:=/path/to/my/custom_sensors.json
```

## Troubleshooting

### Config File Not Found

If you see an error about config file not found, check:
1. The path is relative to where you run the command (usually `ros2_ws`)
2. Use absolute paths if needed: `/full/path/to/config.json`

### Sensor Configuration Missing

If a specific sensor is missing from your config file, you'll see an error like:
```
Temperature sensor not found in config file: config/my_config.json
```

Make sure your config file includes all required sensors: `temperature`, `cube`, `radio`, and `thermal`.

### Permission Denied on Serial Ports

If you get permission errors accessing `/dev/ttyXXX` devices, add your user to the `dialout` group:

```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```
