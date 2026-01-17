#!/bin/bash
# Load ROS2 environment that ships with Jazzy ~Registers ROS2 Packages
source /opt/ros/jazzy/setup.bash 
# Runs the teleop_twist_keyboard node for teleop_twist_keyboard package
# Essentially this allows us to control the robot and communicate with it
# Will need an alternative or adaptation as we update the robot URDF =
ros2 run teleop_twist_keyboard teleop_twist_keyboard