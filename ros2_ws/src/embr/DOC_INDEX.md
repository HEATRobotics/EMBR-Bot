# EMBR Sensor Abstraction Layer - Documentation Index

## 📚 Start Here

**New to this?** Start with:
1. 👉 **[README_COMPLETE.md](README_COMPLETE.md)** - Overview and quick start
2. 👉 **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - Essential commands

**Ready to test?**
1. Build: `colcon build --packages-select embr`
2. Run: `export EMBR_SENSOR_MODE=sim && ros2 launch embr embr_launch_v2.py`

---

## 📖 Documentation Guide

### 🚀 Getting Started (Read in order)

| Document | Purpose | Time |
|----------|---------|------|
| **[README_COMPLETE.md](README_COMPLETE.md)** | Complete overview, quick start, verification | 10 min |
| **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** | Command cheat sheet | 5 min |
| **[SETUP_COMPLETE.md](SETUP_COMPLETE.md)** | What was created and how to verify | 10 min |

### 📘 In-Depth Documentation

| Document | Purpose | Audience |
|----------|---------|----------|
| **[TESTING_GUIDE.md](TESTING_GUIDE.md)** | Complete testing documentation | Developers testing code |
| **[SENSOR_ABSTRACTION_README.md](SENSOR_ABSTRACTION_README.md)** | Full API reference | Developers using the API |
| **[ARCHITECTURE.md](ARCHITECTURE.md)** | Visual diagrams and architecture | Developers extending the system |

### 💻 Code Examples

| File | Purpose |
|------|---------|
| **[examples/sensor_testing_examples.py](examples/sensor_testing_examples.py)** | Runnable examples for all sensors |
| **[test/test_sensors.py](test/test_sensors.py)** | Unit test examples with pytest |

### ⚙️ Configuration

| File | Purpose |
|------|---------|
| **[config/sensors.json](config/sensors.json)** | Default config (auto-detect) |
| **[config/sensors_sim.json](config/sensors_sim.json)** | Full simulation |
| **[config/sensors_mixed.json](config/sensors_mixed.json)** | Mixed real/sim example |

---

## 🎯 Find What You Need

### I want to...

#### ...understand what this is
→ Read **[README_COMPLETE.md](README_COMPLETE.md)**

#### ...run a quick test
→ Follow **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** commands

#### ...test without hardware
→ See **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** → "All Simulated"

#### ...test with some sensors
→ See **[TESTING_GUIDE.md](TESTING_GUIDE.md)** → "Mixed Mode"

#### ...write tests
→ Study **[test/test_sensors.py](test/test_sensors.py)**

#### ...use the API in my code
→ Read **[SENSOR_ABSTRACTION_README.md](SENSOR_ABSTRACTION_README.md)**

#### ...understand the design
→ See **[ARCHITECTURE.md](ARCHITECTURE.md)**

#### ...see examples
→ Run **[examples/sensor_testing_examples.py](examples/sensor_testing_examples.py)**

#### ...configure sensors
→ Edit **[config/sensors.json](config/sensors.json)**

#### ...verify my setup
→ Run `test_sensor_setup.sh` or `.bat`

---

## 📂 File Organization

### Source Code
```
embr/
├── sensors/              ← Sensor abstraction layer
│   ├── __init__.py
│   ├── base.py          ← Base classes
│   ├── factory.py       ← Sensor factory
│   ├── temperature.py   ← Temperature sensor
│   ├── cube.py          ← GPS sensor
│   ├── thermal.py       ← Thermal camera
│   └── mavlink.py       ← MAVLink connection
├── getTemp_v2.py        ← New nodes using abstraction
├── getCube_v2.py
└── sendRf_v2.py
```

### Configuration
```
config/
├── sensors.json         ← Start here
├── sensors_sim.json     ← Full simulation
└── sensors_mixed.json   ← Mixed mode example
```

### Documentation
```
docs/
├── README_COMPLETE.md           ← START HERE
├── QUICK_REFERENCE.md           ← Commands
├── SETUP_COMPLETE.md            ← What was built
├── TESTING_GUIDE.md             ← Testing
├── SENSOR_ABSTRACTION_README.md ← API docs
├── ARCHITECTURE.md              ← Design
└── DOC_INDEX.md                 ← This file
```

### Tests & Examples
```
test/
└── test_sensors.py      ← Unit tests

examples/
└── sensor_testing_examples.py  ← Runnable examples
```

---

## 🎓 Learning Paths

### Path 1: Quick User (30 minutes)
1. Read [README_COMPLETE.md](README_COMPLETE.md) (10 min)
2. Follow [QUICK_REFERENCE.md](QUICK_REFERENCE.md) commands (10 min)
3. Run `examples/sensor_testing_examples.py` (10 min)

**Result**: Can use the system

### Path 2: Developer (2 hours)
1. Read [README_COMPLETE.md](README_COMPLETE.md) (10 min)
2. Read [SENSOR_ABSTRACTION_README.md](SENSOR_ABSTRACTION_README.md) (30 min)
3. Read [TESTING_GUIDE.md](TESTING_GUIDE.md) (30 min)
4. Study [test/test_sensors.py](test/test_sensors.py) (20 min)
5. Experiment with configs (30 min)

**Result**: Can write code using the API

### Path 3: Contributor (4 hours)
1. Complete Developer path (2 hours)
2. Read [ARCHITECTURE.md](ARCHITECTURE.md) (30 min)
3. Study source code in `embr/sensors/` (1 hour)
4. Implement a new sensor type (30 min)

**Result**: Can extend the system

---

## 🔍 Quick Search

### Commands
→ **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)**

### API Reference
→ **[SENSOR_ABSTRACTION_README.md](SENSOR_ABSTRACTION_README.md)**

### Configuration Options
→ **[TESTING_GUIDE.md](TESTING_GUIDE.md)** → Configuration section

### Testing Examples
→ **[test/test_sensors.py](test/test_sensors.py)**

### Code Examples
→ **[examples/sensor_testing_examples.py](examples/sensor_testing_examples.py)**

### Troubleshooting
→ **[README_COMPLETE.md](README_COMPLETE.md)** → Troubleshooting section

### Architecture Diagrams
→ **[ARCHITECTURE.md](ARCHITECTURE.md)**

---

## 🆘 Help & Support

### Common Issues

**Build fails**
→ Check [README_COMPLETE.md](README_COMPLETE.md) → Troubleshooting

**Import errors**
→ Rebuild and source: `colcon build --packages-select embr && source install/setup.bash`

**Tests fail**
→ Run `colcon test-result --verbose` for details

**Config not found**
→ Use absolute path or check working directory

**Permission denied on serial**
→ Add user to dialout group: `sudo usermod -a -G dialout $USER`

---

## 📊 Documentation Statistics

- **7 markdown documents** (including this index)
- **2 example files** (runnable Python)
- **3 config files** (JSON)
- **2 test scripts** (Linux + Windows)
- **Total: ~5,000 lines** of documentation and examples

---

## 🎯 Essential Files (Top 5)

If you only read 5 files, read these:

1. **[README_COMPLETE.md](README_COMPLETE.md)** - Complete overview
2. **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - Commands you need
3. **[examples/sensor_testing_examples.py](examples/sensor_testing_examples.py)** - Working code
4. **[config/sensors.json](config/sensors.json)** - Configuration
5. **[test/test_sensors.py](test/test_sensors.py)** - Test examples

---

## 🚀 Quick Start (Copy-Paste)

```bash
# 1. Build
cd ros2_ws
colcon build --packages-select embr
source install/setup.bash

# 2. Test (optional)
colcon test --packages-select embr
python3 src/embr/examples/sensor_testing_examples.py

# 3. Run in simulation
export EMBR_SENSOR_MODE=sim
ros2 launch embr embr_launch_v2.py

# 4. Watch it work
ros2 topic echo /temperature
```

---

## 📝 Document Purposes at a Glance

| Document | Type | Length | Purpose |
|----------|------|--------|---------|
| README_COMPLETE | Overview | Long | Comprehensive introduction |
| QUICK_REFERENCE | Cheatsheet | Short | Quick commands |
| SETUP_COMPLETE | Summary | Medium | What was built |
| TESTING_GUIDE | Tutorial | Long | How to test |
| SENSOR_ABSTRACTION_README | Reference | Long | API documentation |
| ARCHITECTURE | Diagrams | Medium | System design |
| DOC_INDEX | Index | Short | Navigation (this file) |

---

## 🎨 Documentation Style Guide

- **Bold** for emphasis and file names
- `code` for commands and code snippets
- → for "see also" references
- ✅ for completed features
- 📚 📖 📘 for documentation sections
- 🚀 🎯 💡 for action items and tips

---

## 🔄 Document Update Frequency

- **README_COMPLETE.md**: Updated for major features
- **QUICK_REFERENCE.md**: Updated for new commands
- **TESTING_GUIDE.md**: Updated for new testing patterns
- **SENSOR_ABSTRACTION_README.md**: Updated for API changes
- **ARCHITECTURE.md**: Updated for design changes
- **Config files**: Updated for new defaults
- **Examples**: Updated for new features

---

## 📅 Version Info

- **Created**: November 2025
- **Framework Version**: 1.0
- **ROS2 Version**: Humble
- **Python Version**: 3.8+
- **Status**: Production Ready

---

## ✨ Summary

This documentation suite provides:
- ✅ Multiple entry points for different needs
- ✅ Progressive disclosure (start simple, go deep)
- ✅ Practical examples and runnable code
- ✅ Visual diagrams for understanding
- ✅ Quick reference for daily use
- ✅ Comprehensive guides for deep learning

**Choose your starting point above and begin exploring!**

---

**Made with ❤️ for the EMBR-Bot project**
