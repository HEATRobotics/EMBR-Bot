###########################################
# Purpose of this file is to launch multiple nodes 
# ROS2 & GZ Launch file of EMBER-Bot
###########################################

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
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

    # relative path world
    world_file_relative_path = os.path.join('worlds', 'thermalTestArea.world')

    # absolute path to world
    world_path = os.path.join(
        get_package_share_directory(package_name),
        world_file_relative_path
    )


    # get the robot description from the xacro model file 
    robot_description = xacro.process_file(path_model_file).toxml()

    # this is the launch file from the ros_gz_sim package 
    gazebo_ros_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    )

    gazebo_launch = IncludeLaunchDescription(
        gazebo_ros_launch,
        launch_arguments={
            'gz_args': f'-r -v 4 "{world_path}"',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Gazebo node: spawn the robot from /robot_description
    spawn_model_node_gazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_xacro_name,
            '-topic', 'robot_description',
            # Check Worlds SPAWN Specification
            '-x', '0', '-y', '0', '-z', '0.5',
        ],
        output='screen',
    )

    # Robot State Publisher Node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0','0','0','0','0','0',
            'sf45b_top',
            'ember_bot/sf45b_top/sf45b_gpu_lidar'
        ],
        output='screen'
    )

    #Name custom topics with "custom_" prior to topics that are in it or other names etc.
    costum_gps_imu = Node(
        package='ember_robot',
        executable='gps_imu_topic',
        name='gps_imu_fusion',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Bridge parameters yaml
    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'parameters',
        'bridge_parameters.yaml',
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',   # or config_files:= if your yaml expects that
        ],
        output='screen',
    )

    # Build the launch description
    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_model_node_gazebo)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(lidar_static_tf)
    ld.add_action(costum_gps_imu)
    ld.add_action(start_gazebo_ros_bridge_cmd)

    return ld