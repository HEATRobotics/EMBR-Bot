# Rover Probe Motor Control System

# Overview
This documentation provides setup guide and serves as a resource to understand the implementation of the LMDCE421(probe-motor) communication.

This document will give the reader sufficient knowledge to modify/rebuild the probe-motor codebase.

With these goals in mind I have indicated (opt) next to topics or sections that I deem to be irrelevant to goal 1.

# Table Of Content
Probe-Motor
  - [LMDCE421](#lmdce421)
  - [Function Support](#function-support)


Communication Medium
  - [MODBUS/TCP](#modbus/tcp)
  - [ROS2 Network](#ros2-network)
  - [pymodbus](#pymodbus)

Setup
  - [Static IP Setup](#static-ip-setup)

---

# Probe-Motor
The probe-motor's task is to provide linear motion to a temperature probe, allowing the probe to take readings closer to the surface of the hotspot. The chosen motor for this task is the **LMDCE421**, a stepper with high precision and high load capacity.

## LMDCE421
<img alt="image" src="https://www.novantaims.com/wp-content/uploads/2017/01/lmd17_right_facing_hirez-234x300-NL.jpg" />

[LMDCE421](https://www.novantaims.com/liberty-mdrives/lm42-nema-17-ethernet-tcpip-ip20-lmoe42/) is an integrated Lexium MDrive (LMD) motor combining a stepper motor, drive electronics, and a controller into a single package. 

* **Operating Principle:** It uses microstepping to achieve high resolution. For our application, the motor translates rotary motion into linear motion via a lead screw.
* **Power Requirements:** Typically operates on **12–48 VDC**. 
* **Networking:** The "E" in the model name signifies **Ethernet** connectivity, supporting industrial protocols like MODBUS/TCP.
* **Registers (opt):** Internally, the motor operates on a set of parameters (e.g., Velocity, Acceleration, Target Position) mapped to specific memory addresses.



## Function Support
To achieve the goals of the probe-motor assembly, the codebase supports the following primary functions:

1.  **Relative/Absolute Movement:** Moving the probe to specific depths (steps or mm) relative to the current or home position.
2.  **Status Monitoring:** Reading motor registers to check if the device is "Ready," "Moving," or in an "Error" state.
3.  **Emergency Stop:** Immediate deceleration and torque-off command for safety.

---

# Communication Medium
The LMDCE421 offers an ethernet and serial interface. We have used the ethernet interface communication because there is extensive library support for encoding/decoding data easily over this interface and communication can easily be reproduced from any computer with an ethernet port. **MODBUS/TCP** is the chosen communication protocol for this interface.

## MODBUS/TCP
[MODBUS/TCP](https://www.novantaims.com/application-note/modbustcp-vs-mcodetcp-overview/) is a variant of the Modbus family of vendor-neutral communication protocols intended for supervision and control of automation equipment. 

<img width="646" height="259" alt="image" src="https://github.com/user-attachments/assets/230d1731-48d0-426b-b00b-496c96136636" />


* **Structure:** It wraps a standard Modbus frame inside a TCP/IP packet (Port 502).
* **Client/Server Model:** In this setup, our control script acts as the **Client** (Master), and the LMDCE421 acts as the **Server** (Slave).
* **Protocol Data Unit (PDU):** This is the core of the message containing the instruction.

<img alt="image" src="https://www.novantaims.com/support/mdi_getting_started/ethernet/images/pdu.gif" />


### Protocol Data Unit Examples
The following table illustrates how the request parameters are structured within the PDU for common motor operations:

| Operation | Function Code | Data Address | Data Quantity/Value | Observation |
| :--- | :--- | :--- | :--- | :--- |
| **Read Position** | `03` (Read Holding) | `00 64` (100) | `00 02` | Reads two registers to get a 32-bit position value. |
| **Check Status** | `04` (Read Input) | `00 0A` (10) | `00 01` | Checks a single register for the "Ready" flag. |
| **Set Velocity** | `06` (Write Single) | `01 2C` (300) | `03 E8` | Writes a fixed speed (e.g., 1000) to the velocity register. |
| **Multi-Move** | `10` (Write Multiple) | `02 58` (600) | `00 04` | Updates acceleration, deceleration, and target at once. |



## pymodbus
[pymodbus](https://pymodbus.readthedocs.io/en/latest/) is a full Python implementation of the Modbus protocol. It abstracts the low-level socket handling into readable Python methods. 

### 1. read_holding_registers
Used to retrieve current settings or state data from the motor.
```python
# Read 2 registers starting at address 100
response = client.read_holding_registers(address=100, count=2)
if not response.isError():
    print(response.registers)
```

### 2. write_register
Used to write a single 16-bit value to the motor.
```python
# Write the value 1 to register address 50
client.write_register(address=50, value=1)
```

### 3. write_registers
Used for 32-bit values or simultaneous parameter updates.
```python
# Write a list of values to registers starting at address 200
values = [1000, 500] 
client.write_registers(address=200, values=values)
```

### 4. read_input_registers
Used to read hardware-level data that is typically read-only.
```python
# Read the current position from the input register
pos_data = client.read_input_registers(address=300, count=1)
```
---

This is a hardware-abstracted ROS2 interface for controlling a linear probe motor (LMDCE421 Stepper). It features a **Factory Pattern** that allows for seamless switching between real hardware and simulated environments via a JSON configuration.


## ROS2 Network
![Text](Images/Probe%20Motor%20Implementation%20Overview.png)

The system is divided into three layers:

1. **Hardware/Simulation Layer**: Handles Modbus TCP communication or simulated physics.
2. **Abstraction Layer**: The `StepperFactory` uses `sensors.json` to decide which driver to instantiate.
3. **ROS2 Layer**: The `probeStepper` node acts as the server, while `nav` acts as the mission controller.

---

### 1. Requirements

#### Hardware

* **Motor**: LMDCE421 Stepper Motor.
* **Communication**: Ethernet/Modbus TCP (Default IP: `192.168.33.1`).

#### Software

* **OS**: Ubuntu 22.04 LTS with ROS2 Humble.
* **Ethernet:** [static IP](rpi-ip-setup-for-motor.md)
* **Python Dependencies**:
* [`pymodbus`](https://pymodbus.readthedocs.io/en/latest/index.html): For Modbus TCP communication.



---

### 2. Configuration (`sensors.json`)

All hardware parameters are stored in `config/sensors.json`. This allows for calibration without changing the source code.

| Parameter | Description |
| --- | --- |
| `mode` | Set to `"real"` for hardware or `"sim"` for simulation. |
| `steps_per_distance` | Conversion factor (Steps per cm). Default: `51200`. |
| `motion_range` | Maximum vertical displacement allowed (cm). |
| `max_velocity` | Speed limit in steps/second. |

---

### 3. ROS2 Interface

#### Subscribed Topics

* **`/move_probe_motor`** (`msg_interface/MoveProbeMotor`): Receives target position commands (cm).
* **`/probe_motor_feedback`** (`msg_interface/ProbeMotorFeedback`): Receives position updates and movement confirmation.

#### Message Definitions

* **MoveProbeMotor**: `int32 move_position` (Target depth in cm).
* **ProbeMotorFeedback**: `bool has_moved`, `int32 position` (Current depth in cm).

---

### 4. Setup and Execution

#### Building the Package

```bash
cd ~/ros2_ws
colcon build --packages-select embr msg_interface
source install/setup.bash

```

#### Running the System

To launch all nodes (Temperature, Cube, Radio, and Motor) with the default configuration:

```bash
ros2 launch embr embr_launch.py

```

To specify a custom configuration file:

```bash
ros2 launch embr embr_launch.py config_file:=/path/to/your_config.json

```

---

### 5. Logic Flow: Navigation & Survey

The `nav` node executes an autonomous survey mission:

1. **Search Phase**: Randomly waits for a "hotspot" detection.
2. **Approach Phase**: Moves the rover toward the target.
3. **Deployment**:
* Publishes to `/move_probe_motor`.
* Uses a `threading.Event` to block until `/probe_motor_feedback` confirms the move.


4. **Retrieval**: Returns the probe to position `0` before continuing the search.

---

### 6. Troubleshooting

* **Connection Errors**: Ensure the LMDCE421 IP (`192.168.33.1`) is reachable from your host machine (`ping 192.168.33.1`) [see](rpi-ip-setup-for-motor.md).

---

# Setup

## Static IP Setup

This guide explains how to configure your Raspberry Pi to talk directly to an industrial LMD/Lexium drive using a static IPv4 address using the built-in Netplan.

---

### Prerequisite:
- [ ] rpi is connected to the LMD Drive via ethernet

LMD Drive is powered with sufficient power:
- [ ] Voltage: 12VDC - 48VDC
- [ ] Current: 0.0Amp - 2.0 Amp

### Step 1: Identify your Ethernet Interface
Before editing configurations, you need the system name for your Ethernet port.

```bash
ip link show

```

Usually, this is `eth0`, but on some Ubuntu builds it may appear as `enp1s0` or similar.

---

### Step 2: Locate the Netplan Configuration

Netplan stores its settings in `.yaml` files. List them to find the one your system is using:

```bash
ls /etc/netplan/

```

Common names: `01-netcfg.yaml`, `50-cloud-init.yaml`, or `00-installer-config.yaml`.

---

### Step 3: Edit the Configuration

Open the file with `nano`. **Note:** YAML files are strictly sensitive to spaces. Do not use tabs.

```bash
sudo nano /etc/netplan/01-netcfg.yaml

```

Replace the content with the following block. Adjust `eth0` to your interface name and `192.168.33.10` to your desired Pi IP.

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.33.10/24

```

---

### Step 4: Validate and Apply

Netplan has a built-in safety feature to prevent you from locking yourself out of the system.

1. **Test the configuration:**

```bash
sudo netplan try

```

If you don't press 'Enter' within 120 seconds, it will roll back the changes.

2. **Apply the changes permanently:**

```bash
sudo netplan apply

```

---

### Step 5: Verify the Link

1. **Check that the IP is assigned:**

```bash
ip addr show eth0

```

2. **Ping the LMD Drive:**
Replace `192.168.33.1` with the actual IP address of your drive.

```bash
ping 192.168.33.1

```

---

### Troubleshooting Tips

* **No Link:** If `ip link` shows `NO-CARRIER`, check your physical Ethernet cable.
* **YAML Errors:** If `netplan apply` throws an error, double-check your indentation. Every nested line must be indented by exactly two spaces.
