"""
Launch file with sensor abstraction support.
Sensors are configured via config file (default: config/sensors.json).
Users can override the config file by passing a different one via config_file parameter.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch argument for config file
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to sensor configuration file (default: config/sensors.json)'
    )
    
    # Get launch configuration
    config_file = LaunchConfiguration('config_file')
    
    return LaunchDescription([
        config_file_arg,
        
        Node(
            package='embr',
            executable='getTemp',
            name='getTemp',
            parameters=[{
                'config_file': config_file,
            }]
        ),
        Node(
            package='embr',
            executable='getCube',
            name='getCube',
            parameters=[{
                'config_file': config_file,
            }]
        ),
        Node(
            package='embr',
            executable='sendRf',
            name='sendRf',
            parameters=[{
                'config_file': config_file,
            }]
        )
    ])
