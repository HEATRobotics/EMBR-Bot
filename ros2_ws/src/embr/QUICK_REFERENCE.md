# EMBR Sensor Testing - Quick Reference

## 🚀 Quick Commands

### All Simulated (No Hardware)
```bash
export EMBR_SENSOR_MODE=sim
ros2 launch embr embr_launch_v2.py
```

### All Real Hardware
```bash
export EMBR_SENSOR_MODE=real
ros2 launch embr embr_launch_v2.py
```

### Auto-Detect
```bash
ros2 launch embr embr_launch_v2.py
```

### Use Custom Config
```bash
ros2 launch embr embr_launch_v2.py config_file:=config/sensors_mixed.json
```

## 📝 Environment Variables

```bash
# Global
export EMBR_SENSOR_MODE=sim|real|auto

# Per-sensor
export EMBR_TEMPERATURE_MODE=sim
export EMBR_CUBE_MODE=real
export EMBR_THERMAL_MODE=sim
export EMBR_MAVLINK_MODE=sim
```

## 🔧 Individual Nodes

```bash
# Temperature
ros2 run embr getTemp_v2 --ros-args -p sensor_mode:=sim

# Cube Orange
ros2 run embr getCube_v2 --ros-args -p sensor_mode:=sim

# MAVLink
ros2 run embr sendRf_v2 --ros-args -p sensor_mode:=sim
```

## 📊 Testing

```bash
# Run all tests
colcon test --packages-select embr
colcon test-result --verbose

# Run examples
python3 ros2_ws/src/embr/examples/sensor_testing_examples.py
```

## 🐍 Python Quick Start

```python
from embr.sensors import create_sensor, SensorConfig

# Temperature
config = SensorConfig(mode='sim', params={'base_temp': 25.0})
sensor = create_sensor('temperature', config)
with sensor:
    temp = sensor.read()

# GPS
config = SensorConfig(mode='sim', params={'pattern': 'circle'})
sensor = create_sensor('cube', config)
with sensor:
    gps = sensor.read()
```

## 📁 Config File Templates

### All Simulation
```json
{
  "temperature": {"mode": "sim"},
  "cube": {"mode": "sim"},
  "thermal": {"mode": "sim"},
  "mavlink": {"mode": "sim"}
}
```

### Mixed Mode
```json
{
  "temperature": {"mode": "real", "device": "/dev/ttyACM0"},
  "cube": {"mode": "sim"},
  "thermal": {"mode": "sim"},
  "mavlink": {"mode": "sim"}
}
```

## 🎯 Common Scenarios

### Scenario 1: Test temperature node without Arduino
```bash
export EMBR_TEMPERATURE_MODE=sim
ros2 run embr getTemp_v2
```

### Scenario 2: Test GPS logic with simulated movement
```bash
ros2 run embr getCube_v2 --ros-args \
  -p sensor_mode:=sim \
  -p config_file:=config/sensors_sim.json
```

### Scenario 3: Mixed testing (real temp, sim GPS)
Edit `config/sensors_mixed.json`:
```json
{
  "temperature": {"mode": "real"},
  "cube": {"mode": "sim"}
}
```
Then:
```bash
ros2 launch embr embr_launch_v2.py config_file:=config/sensors_mixed.json
```

## 🔍 Debugging

### Check sensor mode in logs
Look for: `"initialized in X mode (using Y sensor)"`

### Verify config loading
```bash
ros2 run embr getTemp_v2 --ros-args -p config_file:=config/sensors.json
# Check logs for "Could not load config file" warnings
```

### Test sensor directly
```python
from embr.sensors import create_sensor, SensorConfig
config = SensorConfig(mode='sim')
sensor = create_sensor('temperature', config)
sensor.start()
print(sensor.read())
sensor.stop()
```

## 📈 Simulated Patterns

### Temperature
- Sine wave: `base_temp ± variation`
- Default: 22°C ± 2°C with 0.1°C noise

### GPS Patterns
- `circle`: Fly in circles
- `line`: Linear movement
- `hover`: Stay in place with small variations

### Thermal
- Fixed or moving hotspots
- Configurable count and dimensions

## ⚡ Tips

1. **Use auto mode in production** - Falls back to sim if hardware unavailable
2. **Use sim mode in CI/CD** - No hardware needed
3. **Use mixed mode for partial testing** - Test some components with hardware
4. **Check permissions** - `ls -l /dev/ttyACM0` should show your user in group
5. **Rebuild after changes** - `colcon build --packages-select embr`

## 📚 See Also

- [TESTING_GUIDE.md](TESTING_GUIDE.md) - Full documentation
- [SENSOR_ABSTRACTION_README.md](SENSOR_ABSTRACTION_README.md) - API reference
- [examples/sensor_testing_examples.py](examples/sensor_testing_examples.py) - Code examples
