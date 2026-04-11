#!/bin/bash
set -e

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Could not find a ROS 2 setup.bash under /opt/ros"
    exit 1
fi

if [ -d /workspace/ros2_ws ]; then
    cd /workspace/ros2_ws
elif [ -d /home/ros/ros2_ws ]; then
    cd /home/ros/ros2_ws
else
    echo "Could not find ros2_ws in /workspace or /home/ros"
    exit 1
fi

colcon build --packages-select lidar_processing
source install/setup.bash
ros2 run lidar_processing pointcloud_reader
