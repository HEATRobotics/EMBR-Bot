# Set up for Cube Orange

## Modify dronekit
1. `nano /home/aidan/.local/lib/python3.10/site-packages/dronekit/__init__.py
`
2. change `import collections` to `import collections.abc as collections`
3. add `from collections.abc import MutableMapping`
4. change `class Parameters(collections.MutableMapping, HasObservers)` to `class Parameters(MutableMapping, HasObservers)`

## Motor setup

### Connect Receiver to cube
1. Reciever bottom up ppm/ch1: ground, power, signal
2. cube bottom up rcin: signal, power, ground
3. requires battery power

### On controller
1. Hold Ok until Menu 
2. Click ok and navigate to RX Setup -> Output mode
3. Choose output PPM and Serial i-BUS
 
### Connect ESC to Cube
1. Cube bottom up Main Out: signal, power, ground
2. Setup -> Servo Output # correlates to Main Out # and Function correlates to the controller output from Radio Calibration