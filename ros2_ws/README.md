# ROS 2 workspace (ros2_ws)

ROS 2 workspace containing:
- `src/embr` – main EMBR package with runtime nodes and launch files
- `src/msg_interface` – custom message definitions

## Prerequisites

- ROS 2 Humble installed (use `Tools/Setup-Scripts/setup-all` on Raspberry Pi)
- Python dependencies:
  ```
  pip install -r ./requirements.txt
  ```
  Note: `pyserial` is imported as `serial`.

## Build

`colcon` builds all packages in the workspace.

1. Ensure new nodes are registered in `./src/embr/setup.py` and launches in `./src/embr/launch/embr_launch.py`.
2. Build:
	```
	colcon build
	```
3. Source the overlay:
	```
	source install/setup.bash
	```

## Run

Launch all EMBR nodes via the launch file:

```
ros2 launch embr embr_launch.py
```

Run a specific node:

```
ros2 run embr sendRf
```

If you run with `sudo`, manually source ROS 2 in that shell:

```
source /opt/ros/humble/setup.bash
```

## Package layout

```
src/
├── embr/
│   ├── embr/            # nodes: getCube.py, getTemp.py, sendRf.py
│   ├── launch/          # embr_launch.py
│   └── setup.py         # entry points
└── msg_interface/
	 ├── msg/             # custom msgs (e.g., Gps.msg)
	 └── package.xml
```

## Also see

- Top-level quick start: `../README.md`
- Docs index: `../Documentation/README.md`
- Serial and wiring: `../Documentation/2025/Serial-and-pins.md`
- Start scripts and utilities: `../Tools/Readme.md`
