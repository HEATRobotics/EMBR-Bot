#!/bin/bash
set -e

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source your workspace, if it exists
if [ -f /home/ros/ros2_ws/install/setup.bash ]; then
    source /home/ros/ros2_ws/install/setup.bash
fi

exec "$@"
