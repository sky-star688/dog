"""Dog 2 (lite3_02) – Picking + Arm + VLM launch file.

Launches:
  - map_subscriber      (receive + verify FROZEN shared map from Dog 1)
  - dog2_localization   (initial pose + AMCL convergence + relocalize)
  - cmd_vel_mux / cmd_vel_guard / localization_monitor (safety chain)
  - arm_bridge          (MyArm-2 serial)
  - openmv_positioner   (OpenMV fine positioning)
  - vlm_detector        (large-model recognition, optional)
  - picking_manager     (main picking brain)

Dog 2 refuses targets until map_verified and AMCL localised; PICK additionally
requires the recognition gate configured by target_source/vlm_required.
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
    pkg_share = get_package_share_directory("lite3_02_picking")
    inspection_share = get_package_share_directory("lite3_inspection")
    picking_params = os.path.join(pkg_share, "config", "picking_params.yaml")
    inspection_params = os.path.join(inspection_share, "config", "inspection.yaml")
    app_config = os.path.join(pkg_share, "config", "picking.yaml")

    namespace = LaunchConfiguration("namespace")
    robot_id = LaunchConfiguration("robot_id")

    common = {"namespace": namespace, "output": "screen"}

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="/"),
            DeclareLaunchArgument("robot_id", default_value="lite3_02"),
            DeclareLaunchArgument("enable_map", default_value="true"),
            DeclareLaunchArgument("enable_arm", default_value="true"),
            DeclareLaunchArgument("enable_openmv", default_value="true"),
            DeclareLaunchArgument("enable_vlm", default_value="true"),
            Node(
                package="lite3_02_picking",
                executable="map_subscriber",
                name="map_subscriber_dog2",
                parameters=[picking_params],
                condition=_true("enable_map"),
                **common
            ),
            Node(
                package="lite3_02_picking",
                executable="dog2_localization",
                name="dog2_localization",
                parameters=[picking_params],
                condition=_true("enable_map"),
                **common
            ),
            Node(
                package="lite3_inspection",
                executable="cmd_vel_mux",
                name="cmd_vel_mux",
                parameters=[
                    inspection_params,
                    {
                        "enable_topic": "/dog2/motion_enable",
                        "estop_topic": "/dog2/estop",
                    },
                ],
                **common
            ),
            Node(
                package="lite3_inspection",
                executable="cmd_vel_guard",
                name="cmd_vel_guard",
                parameters=[
                    inspection_params,
                    {"enable_topic": "/dog2/motion_enable"},
                ],
                **common
            ),
            Node(
                package="lite3_inspection",
                executable="localization_monitor",
                name="localization_monitor_dog2",
                parameters=[{"status_topic": "/dog2/localization_status_raw"}],
                **common
            ),
            Node(
                package="lite3_02_picking",
                executable="dog2_arm_bridge",
                name="arm_bridge_dog2",
                parameters=[picking_params],
                condition=_true("enable_arm"),
                **common
            ),
            Node(
                package="lite3_02_picking",
                executable="openmv_positioner",
                name="openmv_positioner_dog2",
                parameters=[picking_params],
                condition=_true("enable_openmv"),
                **common
            ),
            # Large-model recognition (advisory; gated before PICK).
            Node(
                package="lite3_02_picking",
                executable="vlm_detector",
                name="vlm_detector_dog2",
                parameters=[picking_params],
                condition=_true("enable_vlm"),
                **common
            ),
            Node(
                package="lite3_02_picking",
                executable="picking_manager",
                name="picking_manager_dog2",
                parameters=[
                    picking_params,
                    {
                        "robot_id": robot_id,
                        "namespace": namespace,
                    },
                ],
                **common
            ),
        ]
    )
