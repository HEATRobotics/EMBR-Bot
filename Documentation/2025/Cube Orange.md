# Cube Orange setup

Configuration notes for using the Cube Orange with EMBR.

## DroneKit (Python 3.10+) compatibility

Our setup scripts apply this patch automatically. If you installed manually and encounter a DroneKit import error on Python 3.10+, apply the following changes to `dronekit/__init__.py` in your Python site-packages directory (e.g., `~/.local/lib/python3.10/site-packages/dronekit/__init__.py`).

Changes:
1. Replace `import collections` with `import collections.abc as collections`
2. Add `from collections.abc import MutableMapping`
3. Change class definition:
	- From: `class Parameters(collections.MutableMapping, HasObservers)`
	- To:   `class Parameters(MutableMapping, HasObservers)`

Tip: To locate the file, you can run `python3 -c "import dronekit, inspect, os; print(os.path.dirname(inspect.getfile(dronekit)))"`.

## Radio receiver and RC input (Not required for Herelink)

Connect your receiver to the Cube’s RC input:

1. Receiver PPM/CH1 header (bottom-up): GND, 5V, Signal
2. Cube RCIN header (bottom-up): Signal, 5V, GND
3. Provide battery power to the receiver and Cube (USB power is often insufficient for RC + ESCs)

Transmitter configuration (example):
1. Hold Ok until Menu 
2. Click ok and navigate to RX Setup -> Output mode
3. Choose output PPM and Serial i-BUS

Calibrate radio in your flight stack (Mission Planner) so channels map as expected.
This is done under Servo output in mission planner setup tab. 

## ESC and motor outputs

Connect ESC signal leads to Cube Main Out pins:

1. Cube Main Out pins are, bottom-up: Signal, 5V, GND
2. In the flight stack, map Servo Output N to the desired function
	- Servo Output index correlates to Main Out N
	- Set the Function to match your control output from Radio Calibration

**Note** You will need to "arm" the cube to be able to run the motors. This can be done by plugging in red led button that comes with the cube into the GPS port and holding it until it goes from flashing red to solid. 

## Lidar
https://www.youtube.com/watch?v=OCMjvF--N_E&t=112s