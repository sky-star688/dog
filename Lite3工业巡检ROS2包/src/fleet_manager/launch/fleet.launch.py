import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("fleet_manager")
    config = os.path.join(package_share, "config", "fleet.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("fleet_config", default_value=config),
            DeclareLaunchArgument("inspector_id", default_value="lite3_01"),
            DeclareLaunchArgument("operator_id", default_value="lite3_02"),
            Node(
                package="fleet_manager",
                executable="fleet_manager",
                name="fleet_manager",
                output="screen",
                parameters=[
                    {
                        "fleet_config": LaunchConfiguration("fleet_config"),
                        "inspector_id": LaunchConfiguration("inspector_id"),
                        "operator_id": LaunchConfiguration("operator_id"),
                    }
                ],
            ),
        ]
    )
