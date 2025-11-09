###########################################
# Purpose of this file is to launch multiple nodes 
# ROS2 & GZ Launch file of EMBER-Bot
###########################################

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription 
from launch.actions import IncludeLaunchDesription 
from launch.launch_description_sources import PythonLaunchDesciptionSource

from launch_ros.actions import Node 
import xacro

# Tut Time: 42:28