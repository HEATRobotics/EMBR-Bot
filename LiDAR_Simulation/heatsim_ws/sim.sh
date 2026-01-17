#!/bin/bash
cd ~/heatsim_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch ember_robot gazebo_model.launch.py