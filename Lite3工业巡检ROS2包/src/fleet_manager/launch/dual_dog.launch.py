"""
Dual-Dog Collaborative System Launch.

Brings up:
  1. fleet_manager (ground station coordinator)
  2. Dog 1 – lite3_01_mapping (LiDAR SLAM + depth camera + patrol)
  3. Dog 2 – lite3_02_picking (OpenMV + depth camera + MyArm-2 arm)

Map flow: Dog 1 SLAM -> /map -> map_bridge -> /shared_map -> Dog 2

Usage:
  ros2 launch fleet_manager dual_dog.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fleet_share = get_package_share_directory("fleet_manager")
    mapping_share = get_package_share_directory("lite3_01_mapping")
    picking_share = get_package_share_directory("lite3_02_picking")

    fleet_config = os.path.join(fleet_share, "config", "fleet.yaml")
    mapping_launch = os.path.join(mapping_share, "launch", "mapping_dog.launch.py")
    picking_launch = os.path.join(picking_share, "launch", "picking_dog.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("inspector_id", default_value="lite3_01"),
            DeclareLaunchArgument("operator_id", default_value="lite3_02"),
            DeclareLaunchArgument("namespace", default_value="/"),
            Node(
                package="fleet_manager",
                executable="fleet_manager",
                name="fleet_manager",
                output="screen",
                parameters=[
                    {
                        "fleet_config": fleet_config,
                        "inspector_id": LaunchConfiguration("inspector_id"),
                        "operator_id": LaunchConfiguration("operator_id"),
                    }
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(mapping_launch),
                        launch_arguments={
                            "namespace": LaunchConfiguration("namespace"),
                            "robot_id": LaunchConfiguration("inspector_id"),
                        }.items(),
                    )
                ],
            ),
            TimerAction(
                period=4.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(picking_launch),
                        launch_arguments={
                            "namespace": LaunchConfiguration("namespace"),
                            "robot_id": LaunchConfiguration("operator_id"),
                        }.items(),
                    )
                ],
            ),
        ]
    )
