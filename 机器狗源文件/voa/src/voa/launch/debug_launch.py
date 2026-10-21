# voa_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

voa_dir = get_package_share_directory('voa')
voa_launch_py_path = os.path.join(voa_dir, 'launch', 'voa_launch.py')
voa_rviz_path = os.path.join(voa_dir, 'rviz', 'voa.rviz')

def generate_launch_description():
    fake_imu_publisher = Node(
        package='voa',
        executable='fake_imu_publisher',
        name='fake_imu_publisher',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', voa_rviz_path]  # Update the path to your VOA RViz configuration file
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([voa_launch_py_path])
        ),
        fake_imu_publisher,
        rviz
    ])

