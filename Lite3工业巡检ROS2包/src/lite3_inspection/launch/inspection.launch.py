import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("lite3_inspection")
    parameters = os.path.join(package_share, "config", "inspection.yaml")
    return LaunchDescription(
        [
            Node(
                package="lite3_inspection",
                executable="cmd_vel_mux",
                name="cmd_vel_mux",
                output="screen",
                parameters=[parameters],
            ),
            Node(
                package="lite3_inspection",
                executable="cmd_vel_guard",
                name="cmd_vel_guard",
                output="screen",
                parameters=[parameters],
            ),
            Node(
                package="lite3_inspection",
                executable="localization_monitor",
                name="localization_monitor",
                output="screen",
                parameters=[parameters],
            ),
            Node(
                package="lite3_inspection",
                executable="arm_serial_bridge",
                name="arm_serial_bridge",
                output="screen",
                parameters=[parameters],
            ),
            Node(
                package="lite3_inspection",
                executable="vision_detector",
                name="vision_detector",
                output="screen",
                parameters=[parameters],
            ),
            Node(
                package="lite3_inspection",
                executable="depth_projector",
                name="depth_projector",
                output="screen",
                parameters=[parameters],
            ),
            Node(
                package="lite3_inspection",
                executable="event_recorder",
                name="event_recorder",
                output="screen",
                parameters=[parameters],
            ),
            Node(
                package="lite3_inspection",
                executable="inspection_manager",
                name="inspection_manager",
                output="screen",
                parameters=[parameters],
            ),
        ]
    )
