# Rover Probe Motor Control System (EMBR)


---

This is a hardware-abstracted ROS2 interface for controlling a linear probe motor (LMDCE421 Stepper). It features a **Factory Pattern** that allows for seamless switching between real hardware and simulated environments via a JSON configuration.


## 1. System Architecture
![Text](2025/Images/Probe%20Motor%20Implementation%20Overview.png)

The system is divided into three layers:

1. **Hardware/Simulation Layer**: Handles Modbus TCP communication or simulated physics.
2. **Abstraction Layer**: The `StepperFactory` uses `sensors.json` to decide which driver to instantiate.
3. **ROS2 Layer**: The `probeStepper` node acts as the server, while `nav` acts as the mission controller.

---

## 2. Requirements

### Hardware

* **Motor**: LMDCE421 Stepper Motor.
* **Communication**: Ethernet/Modbus TCP (Default IP: `192.168.33.1`).

### Software

* **OS**: Ubuntu 22.04 LTS with ROS2 Humble.
* **Ethernet:** [static IP](2025/rpi-ip-setup-for-motor.md)
* **Python Dependencies**:
* [`pymodbus`](https://pymodbus.readthedocs.io/en/latest/index.html): For Modbus TCP communication.



---

## 3. Configuration (`sensors.json`)

All hardware parameters are stored in `config/sensors.json`. This allows for calibration without changing the source code.

| Parameter | Description |
| --- | --- |
| `mode` | Set to `"real"` for hardware or `"sim"` for simulation. |
| `steps_per_distance` | Conversion factor (Steps per cm). Default: `51200`. |
| `motion_range` | Maximum vertical displacement allowed (cm). |
| `max_velocity` | Speed limit in steps/second. |

---

## 4. ROS2 Interface

### Subscribed Topics

* **`/move_probe_motor`** (`msg_interface/MoveProbeMotor`): Receives target position commands (cm).
* **`/probe_motor_feedback`** (`msg_interface/ProbeMotorFeedback`): Receives position updates and movement confirmation.

### Message Definitions

* **MoveProbeMotor**: `int32 move_position` (Target depth in cm).
* **ProbeMotorFeedback**: `bool has_moved`, `int32 position` (Current depth in cm).

---

## 5. Setup and Execution

### Building the Package

```bash
cd ~/ros2_ws
colcon build --packages-select embr msg_interface
source install/setup.bash

```

### Running the System

To launch all nodes (Temperature, Cube, Radio, and Motor) with the default configuration:

```bash
ros2 launch embr embr_launch.py

```

To specify a custom configuration file:

```bash
ros2 launch embr embr_launch.py config_file:=/path/to/your_config.json

```

---

## 6. Logic Flow: Navigation & Survey

The `nav` node executes an autonomous survey mission:

1. **Search Phase**: Randomly waits for a "hotspot" detection.
2. **Approach Phase**: Moves the rover toward the target.
3. **Deployment**:
* Publishes to `/move_probe_motor`.
* Uses a `threading.Event` to block until `/probe_motor_feedback` confirms the move.


4. **Retrieval**: Returns the probe to position `0` before continuing the search.

---

## 7. Troubleshooting

* **Connection Errors**: Ensure the LMDCE421 IP (`192.168.33.1`) is reachable from your host machine (`ping 192.168.33.1`) [see](2025/rpi-ip-setup-for-motor.md).
