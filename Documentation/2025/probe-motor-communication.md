# Overview
This documentation serves as a resource to understand the implementation of the LMDCE421(probe-motor) communication.

This documentation hopes to achieve 2 goals:
1. Educate the reader with just enough knowledge to make incremental changes to the probe-motor codebase
2. Educate the reader with enough knowledge to rebuild the probe-motor codebase

With these goals in mind I have indicated (opt) next to topics or sections that I deem to be irrelevant to goal 1.

Focus here is on the communication medium and the interface.

# Table Of Content
Probe-Motor
  - [LMDCE421](#lmdce421)
  - [Function Support](#function-support)
  
Communication Medium
  - [MODBUS/TCP](#modbus/tcp)
  - [pymodbus](#pymodbus)

## Probe-Motor
The probe-motor's task is to provide linear motion to a temperature probe allowing the probe to take reading closer to the surface of the hotspot. The chosen
motor for this task is the LMDCE421, a stepper with high precision and high load capacity.

### LMDCE421

### Function Support


## Communication Medium
The LMDCE421 offers an ethernet and serial interface. We have used the ethernet interface communication because there is extensive library support for encoding/decoding data
easily over this interface and communication can easily be /reproduced from any computer with an ethernet port. MODBUS/TCP is the chosen communication protocol for this interface.

### MODBUS/TCP

### pymodbus
