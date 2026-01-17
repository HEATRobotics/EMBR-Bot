###########################################
# Purpose of this file is to launch multiple nodes 
# ROS2 & GZ Launch file of EMBER-Bot
###########################################

import os
import sys 

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch
import launch_ros.actions
import xacro


def generate_launch_description():
    # this name has to match the robot name in the Xacro file 
    robot_xacro_name = 'ember_bot'

    # this is the name of our package
    package_name = 'ember_robot'

    # relative path to the xacro file
    model_file_relative_path = os.path.join(
        'ember_model',
        "urdf",
        'ember_full.xacro'
    )


    # absolute path to the model
    path_model_file = os.path.join(
        get_package_share_directory(package_name),
        model_file_relative_path
    )

    # get the robot description from the xacro model file 
    robot_description = xacro.process_file(path_model_file).toxml()

    return LaunchDescription([
        launch_ros.actions.Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        )
    ])

def main(argv=sys.argv[1:]):
    # Run lifecycle nodes via launch
    ld = generate_launch_description()
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()

if __name__ == '__main__':
    main