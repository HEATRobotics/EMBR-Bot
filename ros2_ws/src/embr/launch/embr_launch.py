from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='embr',
            executable='getTemp',
            name='getTemp'
        ),
        Node( 
            package='embr',
            executable='getCube',
            name='getCube'
        ),
        Node(
            package='embr',
            executable='sendRf',
            name='sendRf'
        ),
        Node(
            package='embr',
            executable='thermalStream',
            name='thermal_stream',
            output='screen',
            emulate_tty=True
        ),
    ])