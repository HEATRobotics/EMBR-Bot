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

def generate_launch_description():
    #this name has to match the robot name in the Xacro file 
    robotXacroName='ember_bot'

    #this is the name of our package, at the same time this is the name of the 
    #folder that will be used to define the paths 
    namePackage = 'ember_robot'

    #this is a relative path to the xaro file defining the model 
    modelFileRelativePath = 'model/robot.xacro'

    #Ucomment this if you want to define your own emprt world model
    #however, then you have to create empty_world.world
    #this is a relative path o the Gazebo world file 
    #worldFileRelativePath = 'model/empty_world.world'
    
    #this is the absolute path to the model 
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)

    #Uncomment this if you are using your own world model 
    #this is the absolute path to the world model 
    #pathWorldFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)

    #get the robot descripton from the xavro model file 
    robotDescription = xacro.process__file(pathModelFile).toxml()

    #this is the launch file from the gazebo_ros package 
    gazebo_rosPackageLaunch = PythonLaunchDesciptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch', 'gz_sim.launch.py'))

    #this is the launch description 

    #this is if you are using your own wolrd model 
    #gazeboLaunch = IncludeLaunchDesription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4', pathWorldFile'], 'on_exit_shutdown':'true'}.items())

    #this is if you are using an empty world model 
    gazeboLaunch = IncludeLaunchDesription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 empty.sdf'], 'on_exit_shutdown':'true'}.items())

    #Gazebo node
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    #Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robotDescription,
            'use_sim_time':True}
            ]
    )

    #this is very iportant so we can control the robot from ROS2 MARK:ROS2 Ctrlr
    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters',
        'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executables='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_files:={bridge_params}', 
        ],
        output='screen'
    )

    #here we create an empty launch description object
    launchDescriptionObject = LaunchDescription()
    
    #we add gazeboLaunch
    launchDescriptionObject.add_action(gazeboLaunch)

    #we add the two nodes 
    launchDescriptionObject.add_action(spawnModelNodeGazebo)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)

    return launchDescriptionObject



