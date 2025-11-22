# EMBR Sensor Abstraction Layer - Setup Complete! 🎉

## What Has Been Created

### 📁 Core Sensor Abstraction Layer
- **`embr/sensors/`** - Complete hardware abstraction layer
  - `base.py` - Base `Sensor` class and `SensorConfig`
  - `factory.py` - `SensorFactory` for creating sensors
  - `temperature.py` - Real and simulated temperature sensor
  - `cube.py` - Real and simulated Cube Orange GPS
  - `thermal.py` - Real and simulated thermal camera
  - `mavlink.py` - Real and simulated MAVLink connection

### 🚀 Updated ROS2 Nodes (V2)
- **`getTemp_v2.py`** - Temperature node with sensor abstraction
- **`getCube_v2.py`** - GPS node with sensor abstraction
- **`sendRf_v2.py`** - MAVLink node with sensor abstraction
- **`embr_launch_v2.py`** - Launch file supporting configuration

### ⚙️ Configuration Files
- **`config/sensors.json`** - Default configuration (auto-detect)
- **`config/sensors_sim.json`** - Full simulation configuration
- **`config/sensors_mixed.json`** - Mixed mode example (some real, some sim)

### 🧪 Testing Infrastructure
- **`test/test_sensors.py`** - Comprehensive unit tests with pytest fixtures
- **`pytest.ini`** - Pytest configuration
- **`test_sensor_setup.sh`** - Linux/Mac test script
- **`test_sensor_setup.bat`** - Windows test script

### 📚 Documentation
- **`SENSOR_ABSTRACTION_README.md`** - Complete API and usage guide
- **`TESTING_GUIDE.md`** - Comprehensive testing guide
- **`QUICK_REFERENCE.md`** - Quick command reference

### 💡 Examples
- **`examples/sensor_testing_examples.py`** - Runnable examples for all sensors

## 🎯 Key Features

### 1. **Three Operating Modes**
- **Auto**: Automatically detects available hardware, falls back to simulation
- **Real**: Force real hardware (fails if not available)
- **Sim**: Force simulation (no hardware required)

### 2. **Flexible Configuration**
Priority (highest to lowest):
1. Per-sensor environment variable (`EMBR_TEMPERATURE_MODE`)
2. Global environment variable (`EMBR_SENSOR_MODE`)
3. Configuration file (`sensors.json`)
4. Auto-detection

### 3. **Simulated Behaviors**
- **Temperature**: Sine wave with configurable noise
- **GPS**: Circle, line, or hover patterns
- **Thermal**: Configurable moving hotspots
- **MAVLink**: Message recording and injection

## 🚀 Getting Started

### Step 1: Build the Package
```bash
cd ros2_ws
colcon build --packages-select embr
source install/setup.bash  # or install/setup.bat on Windows
```

### Step 2: Run Tests (Optional)
```bash
# On Linux/Mac
./test_sensor_setup.sh

# On Windows
test_sensor_setup.bat

# Or manually
colcon test --packages-select embr
python3 src/embr/examples/sensor_testing_examples.py
```

### Step 3: Try Simulation Mode
```bash
# Set environment variable
export EMBR_SENSOR_MODE=sim  # or set EMBR_SENSOR_MODE=sim on Windows

# Launch all nodes
ros2 launch embr embr_launch_v2.py

# Or individual nodes
ros2 run embr getTemp_v2
ros2 run embr getCube_v2
ros2 run embr sendRf_v2
```

### Step 4: Monitor Output
```bash
# In another terminal, watch the topics
ros2 topic echo /temperature
ros2 topic echo /gps
```

## 📊 Usage Examples

### Example 1: Test Temperature Node Without Arduino
```bash
ros2 run embr getTemp_v2 --ros-args -p sensor_mode:=sim
```

### Example 2: Test GPS Logic with Circular Pattern
```bash
ros2 run embr getCube_v2 --ros-args \
  -p sensor_mode:=sim \
  -p config_file:=config/sensors_sim.json
```

### Example 3: Mixed Mode (Real Temp, Simulated GPS)
Edit `config/sensors_mixed.json`:
```json
{
  "temperature": {"mode": "real", "device": "/dev/ttyACM0"},
  "cube": {"mode": "sim", "params": {"pattern": "hover"}},
  "mavlink": {"mode": "sim"}
}
```

Launch:
```bash
ros2 launch embr embr_launch_v2.py config_file:=config/sensors_mixed.json
```

### Example 4: Python API Usage
```python
from embr.sensors import create_sensor, SensorConfig

# Create simulated temperature sensor
config = SensorConfig(mode='sim', params={'base_temp': 25.0})
with create_sensor('temperature', config) as sensor:
    for i in range(5):
        temp = sensor.read()
        print(f"Reading {i+1}: {temp:.2f}°C")
```

## 🧪 Testing Scenarios

### Scenario 1: Full Simulation (CI/CD)
```bash
export EMBR_SENSOR_MODE=sim
colcon test --packages-select embr
```

### Scenario 2: Hardware Integration Test
```bash
export EMBR_SENSOR_MODE=real
ros2 launch embr embr_launch_v2.py
```

### Scenario 3: Partial Hardware Test
```bash
export EMBR_TEMPERATURE_MODE=real
export EMBR_CUBE_MODE=sim
ros2 launch embr embr_launch_v2.py
```

## 📁 File Structure Summary

```
ros2_ws/src/embr/
├── embr/
│   ├── sensors/              # NEW: Sensor abstraction layer
│   │   ├── __init__.py
│   │   ├── base.py
│   │   ├── factory.py
│   │   ├── temperature.py
│   │   ├── cube.py
│   │   ├── thermal.py
│   │   └── mavlink.py
│   ├── getTemp.py            # Original (unchanged)
│   ├── getCube.py            # Original (unchanged)
│   ├── sendRf.py             # Original (unchanged)
│   ├── getTemp_v2.py         # NEW: With abstraction
│   ├── getCube_v2.py         # NEW: With abstraction
│   └── sendRf_v2.py          # NEW: With abstraction
├── config/                   # NEW: Configuration files
│   ├── sensors.json
│   ├── sensors_sim.json
│   └── sensors_mixed.json
├── examples/                 # NEW: Example code
│   └── sensor_testing_examples.py
├── launch/
│   ├── embr_launch.py        # Original (unchanged)
│   └── embr_launch_v2.py     # NEW: With configuration support
├── test/
│   ├── test_sensors.py       # NEW: Sensor tests
│   └── ...                   # Original tests
├── pytest.ini                # NEW: Pytest config
├── SENSOR_ABSTRACTION_README.md  # NEW: Full documentation
├── TESTING_GUIDE.md          # NEW: Testing guide
├── QUICK_REFERENCE.md        # NEW: Quick reference
└── setup.py                  # Updated with v2 executables
```

## 🎯 Common Use Cases

### Use Case 1: Developer Testing Without Hardware
**Problem**: Want to test code changes but don't have sensors connected  
**Solution**: `export EMBR_SENSOR_MODE=sim && ros2 launch embr embr_launch_v2.py`

### Use Case 2: CI/CD Pipeline
**Problem**: Need automated tests without physical hardware  
**Solution**: Set `EMBR_SENSOR_MODE=sim` in CI config, run tests

### Use Case 3: Debugging One Sensor
**Problem**: One sensor is misbehaving, want to isolate it  
**Solution**: Set that sensor to sim mode while keeping others real

### Use Case 4: Algorithm Development
**Problem**: Developing GPS navigation logic, need predictable test data  
**Solution**: Use sim mode with scripted patterns (circle/line/hover)

## 🔧 Next Steps

1. **Try the examples**: `python3 src/embr/examples/sensor_testing_examples.py`
2. **Run tests**: `colcon test --packages-select embr`
3. **Read documentation**: Start with `QUICK_REFERENCE.md`
4. **Experiment with configs**: Edit `config/sensors.json`
5. **Write your own tests**: Use fixtures in `test/test_sensors.py` as templates

## 📝 Important Notes

### Your Original Code is Safe
- All original nodes (`getTemp.py`, `getCube.py`, `sendRf.py`) are **unchanged**
- Original launch file (`embr_launch.py`) still works
- V2 versions are **additions**, not replacements
- You can use both side-by-side

### Backward Compatibility
- Original nodes work exactly as before
- New sensor abstraction is opt-in
- No breaking changes to existing code

### Migration Path
1. **Phase 1**: Use V2 nodes for testing (current state)
2. **Phase 2**: Gradually migrate original nodes to use abstraction
3. **Phase 3**: Deprecate original nodes once V2 is proven

## 🆘 Troubleshooting

### Import Errors
```bash
colcon build --packages-select embr
source install/setup.bash  # or .bat on Windows
```

### Permission Denied on Serial Ports
```bash
# Linux
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Config File Not Found
```bash
# Use absolute path
ros2 run embr getTemp_v2 --ros-args -p config_file:=/full/path/to/sensors.json

# Or check working directory
pwd
ls config/sensors.json
```

### Tests Fail
```bash
# Check Python path
python3 -c "import embr.sensors; print('OK')"

# Verbose test output
colcon test --packages-select embr
colcon test-result --verbose
```

## 📚 Documentation Reference

| Document | Purpose |
|----------|---------|
| **QUICK_REFERENCE.md** | Quick commands and examples |
| **TESTING_GUIDE.md** | Comprehensive testing documentation |
| **SENSOR_ABSTRACTION_README.md** | Full API reference and architecture |
| **examples/sensor_testing_examples.py** | Runnable code examples |

## ✅ Verification Checklist

- [ ] Package builds successfully: `colcon build --packages-select embr`
- [ ] Tests pass: `colcon test --packages-select embr`
- [ ] Can run in sim mode: `ros2 run embr getTemp_v2 --ros-args -p sensor_mode:=sim`
- [ ] Can import sensors: `python3 -c "from embr.sensors import create_sensor"`
- [ ] Launch file works: `ros2 launch embr embr_launch_v2.py sensor_mode:=sim`
- [ ] Examples run: `python3 src/embr/examples/sensor_testing_examples.py`

## 🎉 Success!

You now have a complete sensor abstraction layer that allows you to:
- ✅ Test without any sensors connected
- ✅ Mix real and simulated sensors
- ✅ Run automated tests in CI/CD
- ✅ Develop algorithms with predictable data
- ✅ Easily switch between modes via config/env vars

**Happy Testing! 🚀**
