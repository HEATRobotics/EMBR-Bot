"""
Launch file with sensor abstraction support.
Sensors can be configured via config file or environment variables.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    sensor_mode_arg = DeclareLaunchArgument(
        'sensor_mode',
        default_value='auto',
        description='Sensor mode: auto, real, or sim'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='config/sensors.json',
        description='Path to sensor configuration file'
    )
    
    # Get launch configurations
    sensor_mode = LaunchConfiguration('sensor_mode')
    config_file = LaunchConfiguration('config_file')
    
    return LaunchDescription([
        sensor_mode_arg,
        config_file_arg,
        
        Node(
            package='embr',
            executable='getTemp_v2',
            name='getTemp',
            parameters=[{
                'sensor_mode': sensor_mode,
                'config_file': config_file,
            }]
        ),
        Node(
            package='embr',
            executable='getCube_v2',
            name='getCube',
            parameters=[{
                'sensor_mode': sensor_mode,
                'config_file': config_file,
            }]
        ),
        Node(
            package='embr',
            executable='sendRf_v2',
            name='sendRf',
            parameters=[{
                'sensor_mode': sensor_mode,
                'config_file': config_file,
            }]
        )
    ])
