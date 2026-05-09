# Cube Orange setup

Configuration notes for using the Cube Orange with EMBR.

## DroneKit (Python 3.10+) compatibility on Raspberry Pi

Our setup scripts apply this patch automatically. If you installed manually and encounter a DroneKit import error on Python 3.10+, apply the following changes to `dronekit/__init__.py` in your Python site-packages directory (e.g., `~/.local/lib/python3.10/site-packages/dronekit/__init__.py`).

Changes:
1. Replace `import collections` with `import collections.abc as collections`
2. Add `from collections.abc import MutableMapping`
3. Change class definition:
	- From: `class Parameters(collections.MutableMapping, HasObservers)`
	- To:   `class Parameters(MutableMapping, HasObservers)`

Tip: To locate the file, you can run `python3 -c "import dronekit, inspect, os; print(os.path.dirname(inspect.getfile(dronekit)))"`.

## Cube Orange Provisioning
Ardupilot is the firmware that runs on the cube. Mission Planner, QGroundControle, etc are ground control stations (GCS) that are used to both set up the cube initially (running on a computer) and control the cube (running on the controller). The firmware can be installed using any GCS but it is recommended to use mission planner as it has better support for rovers.

### Flash the firmware
1. Connect the cube to a computer via micro USB
2. Open mission planner and connect to the cube
3. Go to setup and click install firmware
4. Choose Rover and set Platform to Cube Orange Plus
5. Click upload firmware and follow instructions on screen

### Cube Settings
Under config->full parameter list set the following settings. 
1. Telem 1 (Herelink): Maps to SERIAL1.
https://docs.cubepilot.org/user-guides/herelink/herelink-overview

SERIAL1_PROTOCOL = 2 (MAVLink 2)

SERIAL1_BAUD = 57 (57600 baud is standard for Herelink)


2. Telem 2 (Raspberry Pi): Maps to SERIAL2.

SERIAL2_PROTOCOL = 2 (MAVLink 2)

SERIAL2_BAUD = 921 (921600 baud is recommended for Pi companion computers, but ensure your Pi's UART is configured to match).

3. GPS 2 (Lightware SF45/b): Maps to SERIAL4.

SERIAL4_PROTOCOL = 11 (Lidar360)

SERIAL4_BAUD = 115 (115200 baud, or whatever the SF45/b is default configured to).

3. CAN Bus & GPS Configuration (Here4)
The Here4 uses DroneCAN, which requires enabling the CAN bus.

CAN_P1_DRIVER = 1 (Enables CAN 1)

CAN_D1_PROTOCOL = 1 (DroneCAN)

GPS1_TYPE = 9 (DroneCAN)=

Reboot the Cube after setting these for the CAN bus to initialize.

4. LiDAR Proximity Setup
To use the SF45/b for obstacle avoidance:

PRX1_TYPE = 8 (Lightware SF45B)

AVOID_ENABLE = 7 (Enables all avoidance types: Proximity, proximity-based fence, etc.).