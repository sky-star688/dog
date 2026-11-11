"""Picking manager for Dog 2 – OpenMV + MyArm-2 precision picking.

Dog 2:
  1. Waits for Dog 1's FROZEN + hash-verified shared map (/dog2/map_status)
  2. Waits for AMCL convergence (/dog2/localization_status)
  3. Waits for target assignment from fleet_manager
  4. Navigates to the workspace approach pose (Nav2)
  5. Fine-positions with OpenMV
  6. Executes arm PICK then PLACE
  7. Retreats to a safe pose
  8. Reports completion to fleet_manager
"""

import json
import os
import time
from typing import Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from lite3_inspection.fleet_client import ClientState, FleetClientModel
from lite3_inspection.geometry import quaternion_from_yaw
from lite3_inspection.robots import RobotContext

from .picking_state import PickingModel, PickingState, TargetAssignment


class PickingManager(Node):
    def __init__(self):
        super().__init__("picking_manager_dog2")

        self.declare_parameter("robot_id", "lite3_02")
        self.declare_parameter("namespace", "/")
        self.declare_parameter("map_id", "lite3_shared")
        self.declare_parameter("navigation_action", "navigate_to_pose")
        self.declare_parameter("nav_server_wait_sec", 2.0)
        self.declare_parameter("target_assignment_topic", "/lite3_02/target_assignment")
        self.declare_parameter("map_status_topic", "/dog2/map_status")
        self.declare_parameter("localization_topic", "/dog2/localization_status")
        self.declare_parameter("openmv_position_topic", "/dog2/openmv_position")
        self.declare_parameter("arm_command_topic", "/dog2/arm_command")
        self.declare_parameter("arm_ready_topic", "/dog2/arm_ready")
        self.declare_parameter("arm_status_topic", "/dog2/arm_status")
        self.declare_parameter("motion_enable_topic", "/dog2/motion_enable")
        self.declare_parameter("estop_topic", "/dog2/estop")
        self.declare_parameter("pick_timeout_sec", 15.0)
        self.declare_parameter("place_timeout_sec", 15.0)
        self.declare_parameter("positioning_timeout_sec", 30.0)
        self.declare_parameter("retreat_timeout_sec", 60.0)
        self.declare_parameter("approach_timeout_sec", 120.0)
        self.declare_parameter("fleet_mode", True)
        self.declare_parameter("fleet_heartbeat_topic_base", "/fleet/heartbeat")
        self.declare_parameter("fleet_status_topic", "/fleet/handoff_status")
        self.declare_parameter("fleet_report_topic", "/fleet/handoff_status")

        self.robot_id = str(self.get_parameter("robot_id").value)
        self.fleet_mode = bool(self.get_parameter("fleet_mode").value)
        self.model = PickingModel(robot_id=self.robot_id)

        try:
            self.context = RobotContext(
                robot_id=self.robot_id,
                namespace=str(self.get_parameter("namespace").value),
                map_id=str(self.get_parameter("map_id").value),
            )
        except ValueError as exc:
            self.get_logger().error("Invalid robot context: %s" % exc)

        self._workspaces = {}
        self._home = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._load_config()

        self.fleet = None
        self._fleet_hb_pub = None
        self._fleet_report_pub = None
        if self.fleet_mode:
            self.fleet = FleetClientModel(robot_id=self.robot_id, robot_role="operator")
            self._fleet_hb_pub = self.create_publisher(
                String,
                "%s/%s"
                % (
                    str(self.get_parameter("fleet_heartbeat_topic_base").value),
                    self.robot_id,
                ),
                10,
            )
            self._fleet_report_pub = self.create_publisher(
                String, str(self.get_parameter("fleet_report_topic").value), 10
            )
            self.create_timer(1.0, self._tick_fleet)

        self._nav_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter("navigation_action").value)
        )
        self._nav_ready = False
        self._nav_active = False
        self._nav_handle = None

        self.localization_ok = False
        self.arm_ready = False
        self.arm_busy = False
        self.estop = False
        self._arm_status_time = 0.0

        self._fine = None
        self._fine_stable = 0
        self._fine_required = 10
        self._action_started = 0.0

        self.arm_pub = self.create_publisher(
            String, str(self.get_parameter("arm_command_topic").value), 10
        )
        self.motion_pub = self.create_publisher(
            Bool, str(self.get_parameter("motion_enable_topic").value), 10
        )

        self.create_subscription(
            String,
            str(self.get_parameter("target_assignment_topic").value),
            self._on_target_assignment,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("map_status_topic").value),
            self._on_map_status,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("localization_topic").value),
            self._on_localization,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("openmv_position_topic").value),
            self._on_openmv,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("arm_ready_topic").value),
            self._on_arm_ready,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("arm_status_topic").value),
            self._on_arm_status,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("estop_topic").value),
            self._on_estop,
            10,
        )

        self.create_service(Trigger, "/dog2/home", self._on_home)
        self.create_service(Trigger, "/dog2/stop", self._on_stop)
        self.create_timer(0.5, self._tick)

        self.model.state = PickingState.MAP_WAITING
        self.get_logger().info(
            "Dog 2 picking manager ready; waiting for frozen map + AMCL convergence"
        )

    # ---- Config ----

    def _load_config(self) -> None:
        path = os.path.join(
            get_package_share_directory("lite3_02_picking"),
            "config",
            "picking.yaml",
        )
        if not os.path.isfile(path):
            self.get_logger().warn("picking.yaml not found: %s" % path)
            return
        try:
            with open(path, "r", encoding="utf-8") as fh:
                data = yaml.safe_load(fh) or {}
            for workspace in data.get("workspaces", []):
                self._workspaces[str(workspace["id"])] = workspace
            home = data.get("home", {}) or {}
            self._home = {
                "x": float(home.get("x", 0.0)),
                "y": float(home.get("y", 0.0)),
                "yaw": float(home.get("yaw", 0.0)),
            }
            self.get_logger().info(
                "Loaded %d workspace configs" % len(self._workspaces)
            )
        except Exception as exc:
            self.get_logger().error("Failed to load picking config: %s" % exc)

    # ---- Inputs ----

    def _on_map_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            verified = bool(status.get("map_verified", False))
        except Exception:
            return
        if verified and not self.model.map_ready:
            self.get_logger().info(
                "Dog 2 shared map VERIFIED; waiting for AMCL convergence"
            )
        if not verified and self.model.map_ready:
            self.get_logger().warn("Dog 2 shared map verification LOST")
        self.model.map_ready = verified
        if self.model.state == PickingState.MAP_WAITING and verified:
            # Wait for localisation before accepting assignments.
            pass

    def _on_localization(self, message: Bool) -> None:
        localized = bool(message.data)
        if localized and not self.model.localization_ok:
            self.get_logger().info("Dog 2 LOCALISED on shared map")
        if not localized and self.model.localization_ok:
            self.get_logger().warn("Dog 2 localisation LOST")
        self.model.localization_ok = localized
        if (
            self.model.state == PickingState.MAP_WAITING
            and self.model.map_ready
            and localized
        ):
            self.model.state = PickingState.IDLE
            self.get_logger().info("Dog 2 ready for target assignments")

    def _on_target_assignment(self, message: String) -> None:
        if self.model.state not in (PickingState.IDLE, PickingState.MAP_WAITING):
            self.get_logger().warn(
                "Target ignored: Dog 2 is in state %s" % self.model.state.value
            )
            return
        if not self.model.map_ready:
            self.get_logger().error(
                "Cannot accept target: shared map not frozen+verified from Dog 1"
            )
            return
        if not self.model.localization_ok:
            self.get_logger().error(
                "Cannot accept target: Dog 2 not localised on shared map"
            )
            return
        try:
            payload = json.loads(message.data)
        except Exception:
            return
        if payload.get("type") != "target_assignment":
            return

        coarse = payload.get("coarse_pose", {}) or {}
        workspace_id = str(payload.get("workspace_id", ""))
        workspace = self._workspaces.get(workspace_id, {})

        assignment = TargetAssignment(
            handoff_id=str(payload.get("handoff_id", "")),
            target_id=str(payload.get("target_id", "")),
            class_name=str(payload.get("class_name", "block")),
            confidence=float(payload.get("confidence", 0.0)),
            coarse_x=float(coarse.get("x", 0.0)),
            coarse_y=float(coarse.get("y", 0.0)),
            coarse_z=float(coarse.get("z", 0.0)),
            frame_id=str(coarse.get("frame_id", "map")),
            workspace_id=workspace_id,
            approach_x=float(
                workspace.get("approach_x", float(coarse.get("x", 0.0)) - 2.0)
            ),
            approach_y=float(workspace.get("approach_y", float(coarse.get("y", 0.0)))),
            approach_yaw=float(workspace.get("approach_yaw", 0.0)),
            centre_x=float(workspace.get("centre_x", float(coarse.get("x", 0.0)))),
            centre_y=float(workspace.get("centre_y", float(coarse.get("y", 0.0)))),
            retreat_x=float(
                workspace.get("retreat_x", float(coarse.get("x", 0.0)) - 3.0)
            ),
            retreat_y=float(workspace.get("retreat_y", float(coarse.get("y", 0.0)))),
            retreat_yaw=float(workspace.get("retreat_yaw", 3.1416)),
        )
        errors = assignment.validate()
        if errors:
            self.get_logger().error("Invalid assignment: %s" % "; ".join(errors))
            self._report("failed", {"handoff_id": assignment.handoff_id, "reason": "; ".join(errors)})
            return

        self.model.assign(assignment)
        self.get_logger().info(
            "Target assigned: %s (%s) approach (%.2f, %.2f)"
            % (
                assignment.target_id,
                assignment.class_name,
                assignment.approach_x,
                assignment.approach_y,
            )
        )
        self._report("accepted", {"handoff_id": assignment.handoff_id})
        self._start_operation()

    def _on_openmv(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except Exception:
            return
        if not payload.get("target_locked", False):
            self._fine_stable = 0
            self.model.fine_position_locked = False
            return
        previous = self._fine
        self._fine = payload
        if previous is not None:
            dx = abs(float(payload.get("pixel_cx", 0.0)) - float(previous.get("pixel_cx", 0.0)))
            dy = abs(float(payload.get("pixel_cy", 0.0)) - float(previous.get("pixel_cy", 0.0)))
            if dx < 10.0 and dy < 10.0:
                self._fine_stable += 1
            else:
                self._fine_stable = max(0, self._fine_stable - 1)
        if self._fine_stable >= self._fine_required:
            self.model.fine_position_locked = True
            self.model.fine_x_mm = float(payload.get("fine_x_mm", 0.0))
            self.model.fine_y_mm = float(payload.get("fine_y_mm", 0.0))
            self.model.fine_z_mm = float(payload.get("fine_z_mm", 0.0))

    def _on_arm_ready(self, message: Bool) -> None:
        self.arm_ready = bool(message.data)
        self._arm_status_time = time.monotonic()

    def _on_arm_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except Exception:
            return
        self._arm_status_time = time.monotonic()
        self.arm_busy = bool(payload.get("busy", False))
        if "ready" in payload:
            self.arm_ready = bool(payload["ready"])

    def _on_estop(self, message: Bool) -> None:
        was = self.estop
        self.estop = bool(message.data)
        if self.estop and not was:
            self.get_logger().error("Dog 2 ESTOP")
            self.set_motion(False)
            self._send_arm("STOP")
            self._cancel_nav()
            self.model.state = PickingState.EMERGENCY

    # ---- Operation ----

    def _start_operation(self) -> None:
        self._ensure_nav_server()
        if not self._nav_ready:
            self.model.state = PickingState.FAILED
            self.model.error_message = "Nav2 not available"
            self._report(
                "failed",
                {
                    "handoff_id": self.model.assignment.handoff_id
                    if self.model.assignment
                    else "",
                    "reason": "Nav2 not available",
                },
            )
            return
        self.model.state = PickingState.APPROACHING
        self.model.started_at = time.monotonic()
        self.set_motion(True)
        self._report(
            "phase",
            {
                "handoff_id": self.model.assignment.handoff_id if self.model.assignment else "",
                "phase": "approaching",
            },
        )
        self._navigate_to_approach()

    def _navigate_to_approach(self) -> None:
        assignment = self.model.assignment
        if assignment is None:
            return
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = assignment.approach_x
        pose.pose.position.y = assignment.approach_y
        qx, qy, qz, qw = quaternion_from_yaw(assignment.approach_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.pose = pose
        self._nav_active = True
        self.get_logger().info(
            "Dog 2 approaching (%.2f, %.2f)"
            % (assignment.approach_x, assignment.approach_y)
        )
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_nav_response)

    def _on_nav_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error("Approach goal failed: %s" % exc)
            self._nav_active = False
            self.model.state = PickingState.FAILED
            self.model.error_message = "approach_failed"
            return
        if not handle.accepted:
            self._nav_active = False
            self.model.state = PickingState.FAILED
            self.model.error_message = "approach_rejected"
            return
        self._nav_handle = handle
        handle.get_result_async().add_done_callback(self._on_approach_result)

    def _on_approach_result(self, future) -> None:
        self._nav_active = False
        self._nav_handle = None
        try:
            status = future.result().status
        except Exception:
            self.model.state = PickingState.FAILED
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.model.state = PickingState.FAILED
            self.model.error_message = "approach_failed_%d" % status
            return
        self.set_motion(False)
        self.model.state = PickingState.POSITIONING
        self.get_logger().info("Approach complete; OpenMV fine positioning")
        self._report(
            "phase",
            {
                "handoff_id": self.model.assignment.handoff_id if self.model.assignment else "",
                "phase": "positioning",
            },
        )

    def _execute_pick(self) -> None:
        self.model.state = PickingState.GRASPING
        self._action_started = time.monotonic()
        self._report(
            "phase",
            {
                "handoff_id": self.model.assignment.handoff_id if self.model.assignment else "",
                "phase": "grasping",
            },
        )
        self._send_arm("PICK")
        self.get_logger().info("Dog 2: PICK command sent")

    def _execute_place(self) -> None:
        self.model.state = PickingState.PLACING
        self._action_started = time.monotonic()
        self._report(
            "phase",
            {
                "handoff_id": self.model.assignment.handoff_id if self.model.assignment else "",
                "phase": "placing",
            },
        )
        self._send_arm("PLACE")
        self.get_logger().info("Dog 2: PLACE command sent")

    def _retreat(self) -> None:
        assignment = self.model.assignment
        if assignment is None:
            return
        self.model.state = PickingState.RETREATING
        self._report(
            "phase", {"handoff_id": assignment.handoff_id, "phase": "retreating"}
        )
        self.set_motion(True)
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = assignment.retreat_x
        pose.pose.position.y = assignment.retreat_y
        qx, qy, qz, qw = quaternion_from_yaw(assignment.retreat_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.pose = pose
        self._nav_active = True
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_retreat_result)

    def _on_retreat_result(self, future) -> None:
        self._nav_active = False
        self.set_motion(False)
        self.model.state = PickingState.COMPLETED
        self.model.completed_at = time.monotonic()
        self._report(
            "completed",
            {
                "handoff_id": self.model.assignment.handoff_id
                if self.model.assignment
                else "",
                "success": True,
            },
        )
        self.get_logger().info("Dog 2 operation complete")
        self.model.reset()

    # ---- Fleet ----

    def _tick_fleet(self) -> None:
        if self.fleet is None:
            return
        self.fleet.localization_ok = self.model.localization_ok
        self.fleet.arm_ready = self.arm_ready
        self.fleet.arm_busy = self.arm_busy
        self.fleet.estop = self.estop
        self.fleet.tick()

    def _report(self, event_type: str, detail: dict) -> None:
        if self._fleet_report_pub is None:
            return
        msg = String()
        msg.data = json.dumps(
            {
                "type": event_type,
                "robot_id": self.robot_id,
                "timestamp": time.time(),
                "detail": detail,
            }
        )
        self._fleet_report_pub.publish(msg)

    # ---- Services ----

    def _on_home(self, request, response):
        del request
        self._send_arm("HOME")
        response.success = True
        response.message = "HOME command sent"
        return response

    def _on_stop(self, request, response):
        del request
        self.set_motion(False)
        self._send_arm("STOP")
        self._cancel_nav()
        self.model.state = PickingState.STOPPED
        response.success = True
        response.message = "stop requested"
        return response

    # ---- Helpers ----

    def set_motion(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.motion_pub.publish(msg)

    def _send_arm(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.arm_pub.publish(msg)

    def _ensure_nav_server(self) -> None:
        if self._nav_ready:
            return
        self._nav_ready = self._nav_client.wait_for_server(
            timeout_sec=float(self.get_parameter("nav_server_wait_sec").value)
        )

    def _cancel_nav(self) -> None:
        if self._nav_handle is not None:
            self._nav_handle.cancel_goal_async()
        self._nav_active = False
        self._nav_handle = None

    # ---- Main tick ----

    def _tick(self) -> None:
        now = time.monotonic()

        if self.model.state == PickingState.MAP_WAITING:
            if self.model.map_ready and self.model.localization_ok:
                self.model.state = PickingState.IDLE
                self.get_logger().info("Dog 2 ready for target assignments")

        elif self.model.state == PickingState.POSITIONING:
            if self.model.fine_position_locked:
                self._execute_pick()
            elif self.model.elapsed_sec > float(
                self.get_parameter("positioning_timeout_sec").value
            ):
                self.model.state = PickingState.FAILED
                self.model.error_message = "positioning_timeout"

        elif self.model.state == PickingState.GRASPING:
            # MyArm has no completion ACK ($DGS); use the measured busy window.
            if (
                self._action_started
                and now - self._action_started
                >= float(self.get_parameter("pick_timeout_sec").value)
            ):
                self.model.pick_complete = True
                self.get_logger().info("Pick busy window elapsed; sending PLACE")
                self._execute_place()

        elif self.model.state == PickingState.PLACING:
            if (
                self._action_started
                and now - self._action_started
                >= float(self.get_parameter("place_timeout_sec").value)
            ):
                self.model.place_complete = True
                self.get_logger().info("Place busy window elapsed; retreating")
                self._retreat()

        if self._nav_active and self.model.assignment is not None:
            if (
                self.model.state == PickingState.APPROACHING
                and self.model.elapsed_sec
                > float(self.get_parameter("approach_timeout_sec").value)
            ):
                self._cancel_nav()
                self.model.state = PickingState.FAILED
                self.model.error_message = "approach_timeout"
            elif (
                self.model.state == PickingState.RETREATING
                and self.model.elapsed_sec
                > float(self.get_parameter("retreat_timeout_sec").value)
            ):
                self._cancel_nav()
                self.model.state = PickingState.FAILED
                self.model.error_message = "retreat_timeout"

        if self.model.is_active and not self.model.localization_ok:
            self.set_motion(False)
            self._send_arm("STOP")
            self._cancel_nav()
            self.model.state = PickingState.FAILED
            self.model.error_message = "localization_lost"

        if self.model.state == PickingState.FAILED:
            self.set_motion(False)
            self._send_arm("STOP")
            self._cancel_nav()
            handoff_id = (
                self.model.assignment.handoff_id if self.model.assignment else ""
            )
            self._report(
                "failed",
                {"handoff_id": handoff_id, "reason": self.model.error_message},
            )
            self.get_logger().error(
                "Dog 2 operation failed: %s" % self.model.error_message
            )
            self.model.reset()


def main(args=None):
    rclpy.init(args=args)
    node = PickingManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.set_motion(False)
        node.destroy_node()
        rclpy.shutdown()
