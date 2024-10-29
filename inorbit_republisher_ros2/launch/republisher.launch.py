from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to config file'
    )

    config_arg = DeclareLaunchArgument(
        'config',
        default_value='',
        description='YAML configuration string'
    )

    republisher_node = Node(
        package='inorbit_republisher_ros2',
        executable='inorbit_republisher',
        name='inorbit_republisher',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'config': LaunchConfiguration('config')
        }]
    )

    return LaunchDescription([
        config_file_arg,
        config_arg,
        republisher_node
    ]) 