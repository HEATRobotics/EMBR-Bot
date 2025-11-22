# EMBR Sensor Abstraction - Architecture Diagram

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                       ROS2 Application Layer                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │  getTemp_v2  │  │  getCube_v2  │  │  sendRf_v2   │          │
│  │   (Node)     │  │   (Node)     │  │   (Node)     │          │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘          │
└─────────┼──────────────────┼──────────────────┼─────────────────┘
          │                  │                  │
          │ uses             │ uses             │ uses
          ▼                  ▼                  ▼
┌─────────────────────────────────────────────────────────────────┐
│               Sensor Abstraction Layer (sensors/)                │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    SensorFactory                          │   │
│  │  ┌─────────────────────────────────────────────────┐     │   │
│  │  │  create_sensor(type, config) → Sensor          │     │   │
│  │  │  • Reads config files                           │     │   │
│  │  │  • Checks environment variables                 │     │   │
│  │  │  • Auto-detects hardware                        │     │   │
│  │  │  • Returns appropriate implementation           │     │   │
│  │  └─────────────────────────────────────────────────┘     │   │
│  └──────────────────────────────────────────────────────────┘   │
│                              │                                   │
│                              │ creates                           │
│                              ▼                                   │
│  ┌────────────────────────────────────────────────────────┐     │
│  │              Abstract Sensor Interface                  │     │
│  │  ┌──────────────────────────────────────────────────┐  │     │
│  │  │  class Sensor(ABC):                              │  │     │
│  │  │    def start() → None                            │  │     │
│  │  │    def read() → Any                              │  │     │
│  │  │    def stop() → None                             │  │     │
│  │  │    def is_running() → bool                       │  │     │
│  │  └──────────────────────────────────────────────────┘  │     │
│  └────────────────────────────────────────────────────────┘     │
│                              │                                   │
│              ┌───────────────┴───────────────┐                  │
│              │                               │                  │
│              ▼                               ▼                  │
│  ┌───────────────────────┐     ┌───────────────────────┐       │
│  │  Real Implementation  │     │  Sim Implementation   │       │
│  ├───────────────────────┤     ├───────────────────────┤       │
│  │ RealTemperatureSensor │     │ SimTemperatureSensor  │       │
│  │ • Serial port I/O     │     │ • Sine wave + noise   │       │
│  │ • Arduino comms       │     │ • Configurable params │       │
│  ├───────────────────────┤     ├───────────────────────┤       │
│  │ RealCubeSensor        │     │ SimCubeSensor         │       │
│  │ • DroneKit API        │     │ • Circle/line/hover   │       │
│  │ • GPS/telemetry       │     │ • Realistic movement  │       │
│  ├───────────────────────┤     ├───────────────────────┤       │
│  │ RealThermalSensor     │     │ SimThermalSensor      │       │
│  │ • OpenCV camera       │     │ • Synthetic hotspots  │       │
│  │ • Image processing    │     │ • Moving patterns     │       │
│  ├───────────────────────┤     ├───────────────────────┤       │
│  │ RealMavlinkConnection │     │ SimMavlinkConnection  │       │
│  │ • Serial MAVLink      │     │ • Message recording   │       │
│  │ • pymavlink           │     │ • Message injection   │       │
│  └───────────────────────┘     └───────────────────────┘       │
└─────────────────────────────────────────────────────────────────┘
          │                               │
          │                               │
          ▼                               ▼
┌─────────────────────┐         ┌─────────────────────┐
│   Hardware Layer    │         │  Simulated Data     │
├─────────────────────┤         ├─────────────────────┤
│ • /dev/ttyACM0      │         │ • Math functions    │
│ • /dev/ttyAMA0      │         │ • Random numbers    │
│ • /dev/ttyAMA1      │         │ • Time-based values │
│ • Camera device     │         │ • Synthetic images  │
└─────────────────────┘         └─────────────────────┘
```

## Configuration Flow

```
┌──────────────────────────────────────────────────────────────────┐
│                    Configuration Sources                          │
│  (Priority: Top to Bottom)                                        │
└──────────────────────────────────────────────────────────────────┘
                              │
         ┌────────────────────┼────────────────────┐
         │                    │                    │
         ▼                    ▼                    ▼
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│  Per-sensor     │  │  Global         │  │  Config File    │
│  Environment    │  │  Environment    │  │  sensors.json   │
│  Variable       │  │  Variable       │  │                 │
│                 │  │                 │  │                 │
│ EMBR_           │  │ EMBR_SENSOR_    │  │ {               │
│ TEMPERATURE_    │  │ MODE=sim        │  │   "temperature" │
│ MODE=real       │  │                 │  │   : {           │
│                 │  │                 │  │     "mode":"auto│
└─────────────────┘  └─────────────────┘  └─────────────────┘
         │                    │                    │
         └────────────────────┼────────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │  Auto-Detection │
                    │                 │
                    │  • Check device │
                    │  • Try import   │
                    │  • Fallback sim │
                    └─────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │  Final Mode     │
                    │  (real or sim)  │
                    └─────────────────┘
```

## Testing Workflow

```
┌──────────────────────────────────────────────────────────────────┐
│                        Testing Scenarios                          │
└──────────────────────────────────────────────────────────────────┘

Scenario 1: Full Simulation (No Hardware)
┌─────────────────────────────────────────────────────────────┐
│ export EMBR_SENSOR_MODE=sim                                  │
│ ros2 launch embr embr_launch_v2.py                           │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────┐
        │ All Sensors: Simulated              │
        │ ✓ Temperature: Sine wave            │
        │ ✓ GPS: Circle pattern               │
        │ ✓ Thermal: Synthetic hotspots       │
        │ ✓ MAVLink: Message recording        │
        └─────────────────────────────────────┘

Scenario 2: Mixed Mode (Partial Hardware)
┌─────────────────────────────────────────────────────────────┐
│ config/sensors_mixed.json:                                   │
│ {                                                            │
│   "temperature": {"mode": "real"},  ← Real hardware          │
│   "cube": {"mode": "sim"},          ← Simulated              │
│   "mavlink": {"mode": "sim"}        ← Simulated              │
│ }                                                            │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────┐
        │ Mixed Sensors                       │
        │ ✓ Temperature: Arduino (/dev/ttyACM0│
        │ ✓ GPS: Simulated (circle pattern)   │
        │ ✓ MAVLink: Simulated (recording)    │
        └─────────────────────────────────────┘

Scenario 3: Auto-Detect (Production)
┌─────────────────────────────────────────────────────────────┐
│ ros2 launch embr embr_launch_v2.py  # No config specified    │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────┐
        │ Factory probes each sensor:         │
        │ • Temperature: Check /dev/ttyACM0   │
        │   → Found: Use Real                 │
        │ • Cube: Check /dev/ttyAMA0          │
        │   → Not found: Use Sim              │
        │ • Thermal: Try open camera          │
        │   → Found: Use Real                 │
        │ • MAVLink: Check /dev/ttyAMA1       │
        │   → Not found: Use Sim              │
        └─────────────────────────────────────┘
```

## Data Flow Example

```
Temperature Reading Example:

┌──────────────────┐
│  ROS2 Timer      │  1.0 Hz
│  (1 second)      │
└────────┬─────────┘
         │ tick
         ▼
┌────────────────────────────────┐
│  Node: getTemp_v2              │
│  read_and_publish()            │
└────────┬───────────────────────┘
         │ calls
         ▼
┌────────────────────────────────┐
│  Sensor: TemperatureSensor     │
│  read() → float                │
└────────┬───────────────────────┘
         │
    ┌────┴─────┐
    │          │
    ▼          ▼
┌─────────┐  ┌──────────┐
│  Real   │  │   Sim    │
│ Sensor  │  │  Sensor  │
└────┬────┘  └─────┬────┘
     │             │
     │             │
     ▼             ▼
┌──────────┐  ┌──────────────────┐
│ Serial   │  │ Math.sin(time)   │
│ Read     │  │ + random noise   │
│ Arduino  │  │ → 23.45°C        │
│ → 22.8°C │  └──────────────────┘
└──────────┘
     │             │
     └──────┬──────┘
            │
            ▼
┌────────────────────────────────┐
│  ROS2 Message                  │
│  Temperature()                 │
│  .temperature = 23.45          │
│  .header.stamp = now()         │
└────────┬───────────────────────┘
         │ publish
         ▼
┌────────────────────────────────┐
│  Topic: /temperature           │
└────────────────────────────────┘
```

## Class Hierarchy

```
                    ┌──────────────┐
                    │   Sensor     │
                    │   (ABC)      │
                    └──────┬───────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
┌───────▼────────┐  ┌──────▼───────┐  ┌──────▼────────┐
│ Temperature    │  │    Cube      │  │   Thermal     │
│ Sensor (ABC)   │  │ Sensor (ABC) │  │ Sensor (ABC)  │
└───────┬────────┘  └──────┬───────┘  └──────┬────────┘
        │                  │                  │
   ┌────┴─────┐       ┌────┴─────┐      ┌────┴─────┐
   │          │       │          │      │          │
   ▼          ▼       ▼          ▼      ▼          ▼
┌──────┐  ┌─────┐ ┌──────┐  ┌─────┐ ┌──────┐  ┌─────┐
│ Real │  │ Sim │ │ Real │  │ Sim │ │ Real │  │ Sim │
└──────┘  └─────┘ └──────┘  └─────┘ └──────┘  └─────┘

All implement:
  • start() → None
  • read() → T
  • stop() → None
  • is_running → bool
```

## Deployment Modes

```
┌────────────────────────────────────────────────────────────┐
│                  Development / Testing                      │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Laptop / Desktop                                     │  │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────┐     │  │
│  │  │   Node 1   │  │   Node 2   │  │   Node 3   │     │  │
│  │  │    SIM     │  │    SIM     │  │    SIM     │     │  │
│  │  └────────────┘  └────────────┘  └────────────┘     │  │
│  │                                                       │  │
│  │  ✓ No hardware required                              │  │
│  │  ✓ Fast iteration                                    │  │
│  │  ✓ CI/CD friendly                                    │  │
│  └──────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────┐
│                    Integration Testing                      │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Test Bench                                           │  │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────┐     │  │
│  │  │   Node 1   │  │   Node 2   │  │   Node 3   │     │  │
│  │  │    REAL    │  │    SIM     │  │    SIM     │     │  │
│  │  └─────┬──────┘  └────────────┘  └────────────┘     │  │
│  │        │                                             │  │
│  │        ▼                                             │  │
│  │  ┌──────────┐                                        │  │
│  │  │ Arduino  │  One sensor at a time                 │  │
│  │  └──────────┘                                        │  │
│  │                                                       │  │
│  │  ✓ Partial hardware validation                       │  │
│  │  ✓ Safer than full system                           │  │
│  └──────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────┐
│                     Production / Robot                      │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Robot Hardware                                       │  │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────┐     │  │
│  │  │   Node 1   │  │   Node 2   │  │   Node 3   │     │  │
│  │  │    AUTO    │  │    AUTO    │  │    AUTO    │     │  │
│  │  └─────┬──────┘  └─────┬──────┘  └─────┬──────┘     │  │
│  │        │               │               │             │  │
│  │        ▼               ▼               ▼             │  │
│  │  ┌──────────┐    ┌─────────┐    ┌──────────┐       │  │
│  │  │ Arduino  │    │  Cube   │    │  Camera  │       │  │
│  │  └──────────┘    └─────────┘    └──────────┘       │  │
│  │                                                       │  │
│  │  ✓ Auto-detects hardware                             │  │
│  │  ✓ Falls back to sim if needed                      │  │
│  │  ✓ Graceful degradation                              │  │
│  └──────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────┘
```
