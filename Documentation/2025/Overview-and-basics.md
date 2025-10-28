# Overview and basics

This project targets a Raspberry Pi 4B running Ubuntu 22.04 LTS and ROS 2 Humble.

## Platform

- Hardware: Raspberry Pi 4B (Rev 1.4)
- OS: Ubuntu 22.04.05 LTS (example kernel at time of writing: 5.15.0-1074-raspi)
- ROS: ROS 2 Humble
- Flight controller: Cube Orange
- Radio telemetry: RFD 900x-US
- Sensors: FLIR Lepton 3.1R (PureThermal 3), external temperature sensor (Arduino)

## Install baseline

Use the setup scripts on Ubuntu 22.04 (not Raspberry Pi OS):

- ROS 2 Humble: `Tools/Setup-Scripts/install-ROS2-Humble`
- Full setup (ROS + deps + thermal camera + DroneKit patch): `Tools/Setup-Scripts/setup-all`

For details on running and building, see `ros2_ws/README.md`. To start everything, see `Tools/Readme.md` for the launcher scripts.

## Notes

- The setup scripts are Linux/Ubuntu-oriented; they will not work on Raspberry Pi OS.
- Serial/UART configuration and wiring are covered in `Serial-and-pins.md`.

