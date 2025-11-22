# EMBR Sensor Abstraction Layer

A comprehensive hardware abstraction layer for EMBR-Bot sensors that enables seamless switching between real hardware and simulated sensors for testing.

## Features

- ✅ **Zero Hardware Testing**: Run all nodes with simulated sensors
- ✅ **Mixed Mode**: Use some real sensors and some simulated simultaneously
- ✅ **Auto-Detection**: Automatically detect available hardware
- ✅ **Configuration-Driven**: Easy configuration via JSON files
- ✅ **Environment Variables**: Override behavior without code changes
- ✅ **Context Managers**: Clean resource management with Python's `with` statement
- ✅ **Unit Tests**: Comprehensive test suite with pytest fixtures
- ✅ **Deterministic**: Simulated sensors produce predictable, scriptable outputs

## Quick Start

### Installation

1. Build the package:
```bash
cd ros2_ws
colcon build --packages-select embr
source install/setup.bash
```

### Running with Simulation (No Hardware Required)

```bash
# Option 1: Environment variable
export EMBR_SENSOR_MODE=sim
ros2 launch embr embr_launch_v2.py

# Option 2: Launch argument
ros2 launch embr embr_launch_v2.py sensor_mode:=sim

# Option 3: Use simulation config file
ros2 launch embr embr_launch_v2.py config_file:=config/sensors_sim.json
```

### Running with Real Hardware

```bash
export EMBR_SENSOR_MODE=real
ros2 launch embr embr_launch_v2.py
```

### Mixed Mode (Some Real, Some Sim)

Edit `config/sensors_mixed.json` and launch:
```bash
ros2 launch embr embr_launch_v2.py config_file:=config/sensors_mixed.json
```

## Supported Sensors

| Sensor | Real Implementation | Simulated Behavior |
|--------|-------------------|-------------------|
| **Temperature** | Arduino via serial | Sine wave with noise |
| **Cube Orange** | DroneKit GPS | Configurable movement patterns |
| **Thermal Camera** | OpenCV camera | Synthetic hotspots |
| **MAVLink** | Serial connection | Message recording/injection |

## Configuration

### sensors.json Example

```json
{
  "temperature": {
    "mode": "auto",
    "device": "/dev/ttyACM0",
    "baud": 9600,
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
      "velocity": 5.0
    }
  }
}
```

### Mode Options

- `auto`: Automatically detect hardware (default)
- `real`: Force real hardware (fails if not available)
- `sim`: Force simulation

### Environment Variables

Override configuration at runtime:

```bash
# Global override
export EMBR_SENSOR_MODE=sim

# Per-sensor override
export EMBR_TEMPERATURE_MODE=real
export EMBR_CUBE_MODE=sim
export EMBR_THERMAL_MODE=sim
export EMBR_MAVLINK_MODE=sim
```

## Usage Examples

### Python API

```python
from embr.sensors import create_sensor, SensorConfig

# Create simulated temperature sensor
config = SensorConfig(mode='sim', params={'base_temp': 25.0})
sensor = create_sensor('temperature', config)

# Use with context manager
with sensor:
    temp = sensor.read()
    print(f"Temperature: {temp:.2f}°C")
```

### ROS2 Nodes (V2)

The `_v2` versions of nodes use the sensor abstraction:

```python
# In your node
from embr.sensors import create_sensor, SensorConfig, SensorFactory

# Load config
configs = SensorFactory.load_config('config/sensors.json')
config = configs.get('temperature', SensorConfig(mode='auto'))

# Create and use sensor
self.sensor = create_sensor('temperature', config)
self.sensor.start()
data = self.sensor.read()
self.sensor.stop()
```

### Testing with Pytest

```python
import pytest
from embr.sensors import create_sensor, SensorConfig

@pytest.fixture
def sim_temp_sensor():
    config = SensorConfig(mode='sim')
    sensor = create_sensor('temperature', config)
    sensor.start()
    yield sensor
    sensor.stop()

def test_temperature_reading(sim_temp_sensor):
    temp = sim_temp_sensor.read()
    assert 15.0 < temp < 30.0
```

## Testing

### Run Unit Tests

```bash
cd ros2_ws
colcon test --packages-select embr
colcon test-result --verbose
```

### Run Examples

```bash
cd ros2_ws/src/embr
python3 examples/sensor_testing_examples.py
```

## Project Structure

```
embr/
├── embr/
│   ├── sensors/              # Sensor abstraction layer
│   │   ├── __init__.py
│   │   ├── base.py          # Base classes
│   │   ├── factory.py       # Sensor factory
│   │   ├── temperature.py   # Temperature implementations
│   │   ├── cube.py          # Cube Orange implementations
│   │   ├── thermal.py       # Thermal camera implementations
│   │   └── mavlink.py       # MAVLink implementations
│   ├── getTemp_v2.py        # Temperature node with abstraction
│   ├── getCube_v2.py        # Cube node with abstraction
│   └── sendRf_v2.py         # MAVLink node with abstraction
├── config/
│   ├── sensors.json         # Default configuration
│   ├── sensors_sim.json     # Full simulation config
│   └── sensors_mixed.json   # Mixed mode config
├── examples/
│   └── sensor_testing_examples.py
├── test/
│   └── test_sensors.py      # Unit tests
├── launch/
│   ├── embr_launch.py       # Original launch file
│   └── embr_launch_v2.py    # New launch file with abstraction
└── TESTING_GUIDE.md         # Comprehensive guide
```

## Migration from Original Nodes

Your original nodes (`getTemp.py`, `getCube.py`, `sendRf.py`) remain unchanged. The new `_v2` versions demonstrate the sensor abstraction pattern. To migrate:

1. Import sensor modules
2. Load configuration
3. Create sensors via factory
4. Replace direct hardware access with sensor API
5. Add cleanup in `destroy_node()`

See `TESTING_GUIDE.md` for detailed migration instructions.

## Simulated Sensor Behaviors

### Temperature Sensor
- Base temperature with sine wave variation
- Configurable noise for realism
- Deterministic with time-based seed

### Cube Orange GPS
- **Circle**: Circular flight pattern
- **Line**: Linear movement
- **Hover**: Stationary with small variations

### Thermal Camera
- Configurable number of hotspots
- Moving or static patterns
- Realistic bounding box detection

### MAVLink Connection
- Records all sent messages for inspection
- Supports message injection for testing
- Compatible with real MAVLink message types

## Benefits

1. **Faster Development**: Test code without hardware setup
2. **CI/CD Friendly**: Run tests in CI without physical sensors
3. **Reproducible**: Deterministic simulated data
4. **Safe**: No risk of damaging hardware during testing
5. **Flexible**: Easy to switch between real/sim
6. **Maintainable**: Clean separation of concerns

## Troubleshooting

### Permission Errors
```bash
# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Import Errors
```bash
# Rebuild and source
colcon build --packages-select embr
source install/setup.bash
```

### Config Not Found
Use absolute paths or ensure working directory is correct:
```bash
ros2 run embr getTemp_v2 --ros-args -p config_file:=/full/path/to/config.json
```

## Documentation

- [TESTING_GUIDE.md](TESTING_GUIDE.md) - Comprehensive testing guide
- [examples/sensor_testing_examples.py](examples/sensor_testing_examples.py) - Usage examples
- [test/test_sensors.py](test/test_sensors.py) - Test examples

## License

Apache-2.0

## Contributing

When adding new sensors:
1. Create `Real<Sensor>` and `Sim<Sensor>` classes
2. Inherit from `Sensor` base class
3. Add to `SensorFactory.SENSOR_MAP`
4. Update configuration examples
5. Add unit tests
