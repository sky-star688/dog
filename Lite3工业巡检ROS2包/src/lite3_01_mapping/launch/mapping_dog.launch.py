"""Dog 1 (lite3_01) – Mapping + Patrol launch file.

Launches:
  - cmd_vel_mux + cmd_vel_guard (safety chain)
  - localization_monitor
  - map_bridge (freeze + share map with Dog 2)
  - mapping_manager (map lifecycle / save / status)
  - dog1_vision_detector + dog1_depth_projector (perception chain)
  - patrol_manager (waypoint patrol + fleet reporting)

SLAM itself (slam_toolbox) is expected to run as an external launch; this
file only consumes its /map topic.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _true(name):
    return IfCondition(PythonExpression(["'", LaunchConfiguration(name), "' == 'true'"]))


def generate_launch_description():
    pkg_share = get_package_share_directory("lite3_01_mapping")
    inspection_share = get_package_share_directory("lite3_inspection")
    mapping_config = os.path.join(pkg_share, "config", "mapping.yaml")
    inspection_config = os.path.join(inspection_share, "config", "inspection.yaml")

    namespace = LaunchConfiguration("namespace")
    robot_id = LaunchConfiguration("robot_id")
    mission_file = LaunchConfiguration("mission_file")

    common = {
        "namespace": namespace,
        "output": "screen",
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="/"),
            DeclareLaunchArgument("robot_id", default_value="lite3_01"),
            DeclareLaunchArgument("mission_file", default_value=""),
            DeclareLaunchArgument("enable_perception", default_value="true"),
            DeclareLaunchArgument("enable_map_sharing", default_value="true"),
            # Safety chain (shared nodes; topics are configurable per dog).
            Node(
                package="lite3_inspection",
                executable="cmd_vel_mux",
                name="cmd_vel_mux",
                **common
            ),
            Node(
                package="lite3_inspection",
                executable="cmd_vel_guard",
                name="cmd_vel_guard",
                parameters=[inspection_config],
                **common
            ),
            Node(
                package="lite3_inspection",
                executable="localization_monitor",
                name="localization_monitor_dog1",
                parameters=[{"status_topic": "/dog1/localization_status"}],
                **common
            ),
            # Map sharing.
            Node(
                package="lite3_01_mapping",
                executable="map_bridge",
                name="map_bridge_dog1",
                parameters=[mapping_config],
                condition=_true("enable_map_sharing"),
                **common
            ),
            Node(
                package="lite3_01_mapping",
                executable="mapping_manager",
                name="mapping_manager_dog1",
                parameters=[mapping_config],
                **common
            ),
            # Perception chain.
            Node(
                package="lite3_01_mapping",
                executable="dog1_vision_detector",
                name="vision_detector_dog1",
                parameters=[
                    {
                        "camera_topic": "/camera/color/image_raw",
                        "detections_topic": "/dog1/detections",
                    }
                ],
                condition=_true("enable_perception"),
                **common
            ),
            Node(
                package="lite3_01_mapping",
                executable="dog1_depth_projector",
                name="depth_projector_dog1",
                parameters=[mapping_config],
                condition=_true("enable_perception"),
                **common
            ),
            # Patrol brain.
            Node(
                package="lite3_01_mapping",
                executable="patrol_manager",
                name="patrol_manager_dog1",
                parameters=[
                    mapping_config,
                    {
                        "robot_id": robot_id,
                        "namespace": namespace,
                        "mission_file": mission_file,
                    },
                ],
                **common
            ),
        ]
    )
