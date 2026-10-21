

from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

voa_dir = get_package_share_directory('voa')
voa_launch_py_path = os.path.join(voa_dir, 'launch', 'voa_launch.py')
voa_rviz_path = os.path.join(voa_dir, 'rviz', 'voa.rviz')

def generate_launch_description():
    return LaunchDescription([
        # Include voa_launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([voa_launch_py_path])
        ),
        # Start RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', voa_rviz_path]  # Update the path to your VOA RViz configuration file
        )
    ])
