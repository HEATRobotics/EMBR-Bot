#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
cd /home/ros/ros2_ws
colcon build --packages-select lidar_processing
source install/setup.bash
ros2 run lidar_processing pointcloud_reader
