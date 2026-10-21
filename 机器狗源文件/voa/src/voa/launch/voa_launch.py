# voa_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config_file = LaunchConfiguration('config_path')
    declare_config_path_file_cmd = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(
            get_package_share_directory('voa'), 'config', 'voa.yaml'),
        description='Full path to the Gridmap visualization config file to use')

    base2frontcamera_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base2frontcamera_broadcaster',
        arguments=['0.25489', '0', '0.07249', '0', '0.34907', '0', 'base_link', 'camera_link']
    )

    voa_composition = Node(
        package='voa',
        executable='voa_composition',
        name='voa_composition',
        output='screen',
        parameters=[config_file],
        # prefix=['xterm -e gdb -ex run --args']
    )

    ld = LaunchDescription()
    ld.add_action(declare_config_path_file_cmd)
    ld.add_action(base2frontcamera_broadcaster)
    ld.add_action(voa_composition)

    return ld
