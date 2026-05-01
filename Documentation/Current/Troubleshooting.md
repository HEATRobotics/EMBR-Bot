# Troubleshooting

Common install/build/runtime issues and fixes for the 2025 platform.

## Serial port permissions

```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

## ROS 2 not found when using sudo

When running with sudo, your user shell init files aren’t sourced. Manually source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

## Multiple serial devices present

- Check devices with:
  - `ls -l /dev/serial*`
  - `ls -l /dev/tty*`
- Verify udev rules (e.g., for thermal camera) under `/etc/udev/rules.d/`

## Build failures in ROS 2 workspace

```bash
cd ros2_ws
rm -rf build/ install/ log/
colcon build
```

## DroneKit and Python 3.10+

The setup scripts patch DroneKit automatically during `Tools/Setup-Scripts/setup-all`.

If you need to apply manually, see notes in the repo’s top-level README history or consider reinstalling via the setup scripts.
