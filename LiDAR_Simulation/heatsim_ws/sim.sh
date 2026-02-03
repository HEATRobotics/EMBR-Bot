#!/bin/bash
cd ~/heatsim_ws
#Source ROS
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
#Source Overlay
source install/setup.bash
#Launch
ros2 launch ember_robot gazebo_model.launch.py