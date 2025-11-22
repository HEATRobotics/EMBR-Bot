# EMBR Sensor Testing and Simulation Guide

This document explains how to test and simulate sensors in the EMBR-Bot project.

## Overview

The EMBR sensor abstraction layer allows you to:
- **Test without hardware**: Run all nodes with simulated sensors
- **Mixed mode testing**: Use some real sensors and some simulated
- **Easy switching**: Change between real/sim via config files or environment variables
- **Reproducible tests**: Simulated sensors produce deterministic or scriptable outputs

## Quick Start

### 1. All Simulated Sensors (No Hardware)

Set the environment variable:
```bash
export EMBR_SENSOR_MODE=sim
ros2 launch embr embr_launch_v2.py
```

Or edit `config/sensors.json` and set all sensors to `"mode": "sim"`.

### 2. All Real Sensors (Full Hardware)

```bash
export EMBR_SENSOR_MODE=real
ros2 launch embr embr_launch_v2.py
```

### 3. Mixed Mode (Some Real, Some Simulated)

Edit `config/sensors.json`:
```json
{
  "temperature": {
    "mode": "real",
    "device": "/dev/ttyACM0"
  },
  "cube": {
    "mode": "sim",
    "params": {
      "pattern": "circle"
    }
  },
  "mavlink": {
    "mode": "sim"
  }
}
```

### 4. Auto Mode (Detects Available Hardware)

```bash
# Default: will auto-detect which sensors are connected
ros2 launch embr embr_launch_v2.py
```

## Configuration

### Config File: `config/sensors.json`

Each sensor can be configured individually:

```json
{
  "temperature": {
    "mode": "auto",           // auto, real, or sim
    "device": "/dev/ttyACM0", // serial device path
    "baud": 9600,             // baud rate
    "params": {               // sensor-specific parameters
      "base_temp": 22.0,
      "variation": 2.0,
      "noise": 0.1
    }
  }
}
```

### Environment Variables

- `EMBR_SENSOR_MODE`: Set global mode (auto/real/sim)
- `EMBR_TEMPERATURE_MODE`: Override mode for temperature sensor
- `EMBR_CUBE_MODE`: Override mode for Cube sensor
- `EMBR_THERMAL_MODE`: Override mode for thermal camera
- `EMBR_MAVLINK_MODE`: Override mode for MAVLink connection

Priority: Per-sensor env var > Global env var > Config file > Auto-detect

### Launch File Parameters

```bash
ros2 run embr getTemp_v2 --ros-args \
  -p sensor_mode:=sim \
  -p sensor_device:=/dev/ttyACM0 \
  -p config_file:=config/sensors.json
```

## Simulated Sensor Behaviors

### Temperature Sensor
- **Pattern**: Sine wave with configurable noise
- **Parameters**:
  - `base_temp`: Base temperature (default: 22.0°C)
  - `variation`: Amplitude of sine wave (default: 2.0°C)
  - `noise`: Random noise amplitude (default: 0.1°C)

### Cube Orange GPS
- **Patterns**: circle, line, hover
- **Parameters**:
  - `start_lat`, `start_lon`, `start_alt`: Starting position
  - `velocity`: Ground speed in m/s
  - `pattern`: Movement pattern

### Thermal Camera
- **Behavior**: Generates synthetic hotspots
- **Parameters**:
  - `width`, `height`: Frame dimensions
  - `hotspot_count`: Number of hotspots
  - `moving`: Whether hotspots move

### MAVLink Connection
- **Behavior**: Records sent messages, can inject received messages
- **Use**: Inspect `sent_messages` list for testing

## Testing Examples

### Unit Tests

Run the test suite:
```bash
cd ros2_ws
colcon test --packages-select embr
colcon test-result --verbose
```

### Manual Testing with Simulated Sensors

```python
from embr.sensors import create_sensor, SensorConfig

# Create simulated temperature sensor
config = SensorConfig(mode='sim', params={'base_temp': 25.0})
sensor = create_sensor('temperature', config)

with sensor:
    temp = sensor.read()
    print(f"Temperature: {temp}°C")
```

### Integration Testing

Create a test config `config/test_sensors.json`:
```json
{
  "temperature": {"mode": "sim", "params": {"base_temp": 20.0}},
  "cube": {"mode": "sim", "params": {"pattern": "hover"}},
  "mavlink": {"mode": "sim"}
}
```

Run nodes:
```bash
ros2 run embr getTemp_v2 --ros-args -p config_file:=config/test_sensors.json
ros2 run embr getCube_v2 --ros-args -p config_file:=config/test_sensors.json
ros2 run embr sendRf_v2 --ros-args -p config_file:=config/test_sensors.json
```

### Pytest Fixtures

Use the provided fixtures in your tests:

```python
import pytest
from embr.sensors import create_sensor, SensorConfig

@pytest.fixture
def sim_temperature():
    config = SensorConfig(mode='sim')
    sensor = create_sensor('temperature', config)
    sensor.start()
    yield sensor
    sensor.stop()

def test_my_feature(sim_temperature):
    temp = sim_temperature.read()
    assert 15.0 < temp < 30.0
```

## Architecture

### Base Classes

- `Sensor`: Abstract base class for all sensors
- `SensorConfig`: Configuration dataclass

### Implementations

Each sensor has:
- `Real<Sensor>`: Hardware implementation
- `Sim<Sensor>`: Simulated implementation

### Factory

- `SensorFactory.create()`: Creates appropriate sensor based on config
- `create_sensor()`: Convenience function

## Migration Guide

To use the new sensor abstraction in existing nodes:

1. **Import the sensors module**:
   ```python
   from embr.sensors import create_sensor, SensorConfig, SensorFactory
   ```

2. **Load configuration**:
   ```python
   configs = SensorFactory.load_config('config/sensors.json')
   config = configs.get('temperature', SensorConfig(mode='auto'))
   ```

3. **Create sensor**:
   ```python
   self.sensor = create_sensor('temperature', config)
   self.sensor.start()
   ```

4. **Use sensor**:
   ```python
   data = self.sensor.read()
   ```

5. **Cleanup**:
   ```python
   self.sensor.stop()
   ```

## Best Practices

1. **Always use context managers when possible**:
   ```python
   with sensor:
       data = sensor.read()
   ```

2. **Handle exceptions**:
   ```python
   try:
       sensor.start()
       data = sensor.read()
   except RuntimeError as e:
       logger.error(f"Sensor error: {e}")
   ```

3. **Use fixtures in tests** for automatic cleanup

4. **Configure via files** for reproducibility

5. **Use auto mode** in production for flexibility

## Troubleshooting

### Sensor fails to start in auto mode
- Check device permissions: `ls -l /dev/ttyACM0`
- Verify device exists: `ls /dev/tty*`
- Try explicit mode: Set `"mode": "sim"` temporarily

### Import errors
- Rebuild package: `colcon build --packages-select embr`
- Source workspace: `source install/setup.bash`

### Config file not found
- Check path: Config file should be relative to workspace root
- Use absolute path: `-p config_file:=/full/path/to/sensors.json`

## Examples

See `test/test_sensors.py` for comprehensive examples of:
- Unit tests for each sensor
- Integration tests between sensors
- Using pytest fixtures
- Testing with simulated data
