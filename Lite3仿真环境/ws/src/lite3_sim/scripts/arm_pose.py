#!/usr/bin/env python3

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


LEG_JOINTS = [
    "FL_HipX_joint",
    "FL_HipY_joint",
    "FL_Knee_joint",
    "FR_HipX_joint",
    "FR_HipY_joint",
    "FR_Knee_joint",
    "HL_HipX_joint",
    "HL_HipY_joint",
    "HL_Knee_joint",
    "HR_HipX_joint",
    "HR_HipY_joint",
    "HR_Knee_joint",
]

ARM_JOINTS = [
    "arm_yaw_joint",
    "arm_shoulder_joint",
    "arm_elbow_joint",
    "arm_wrist_joint",
    "arm_roll_joint",
    "gripper_left_joint",
    "gripper_right_joint",
]

LEG_POSE = [
    0.0, -0.652, 1.272,
    0.0, -0.652, 1.272,
    0.0, -0.652, 1.272,
    0.0, -0.652, 1.272,
]

ARM_HOME = [0.0, 0.30, -0.60, 0.30, 0.0, 0.0, 0.0]
ARM_PICK = [0.0, 0.60, -1.00, 0.40, 0.0, 0.015, 0.015]
ARM_PLACE = [0.0, 0.20, -0.40, 0.20, 0.0, 0.0, 0.0]


class RobotPose(Node):
    def __init__(self):
        super().__init__("robot_pose")
        self.arm_pose = list(ARM_HOME)
        self.publisher = self.create_publisher(
            JointTrajectory, "/set_joint_trajectory", 10
        )
        self.create_subscription(String, "/arm/command", self.on_command, 10)
        self.create_timer(0.02, self.publish_pose)

    def on_command(self, message):
        command = message.data.strip().upper()
        if command in ("HOME", "STOP"):
            self.arm_pose = list(ARM_HOME)
        elif command in ("PICK", "GROUP:1"):
            self.arm_pose = list(ARM_PICK)
        elif command in ("PLACE", "GROUP:2"):
            self.arm_pose = list(ARM_PLACE)
        else:
            self.get_logger().info("Unhandled arm command in sim: %s" % message.data)

    def publish_pose(self):
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "TORSO"
        message.joint_names = LEG_JOINTS + ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = LEG_POSE + self.arm_pose
        point.time_from_start = Duration(sec=1, nanosec=0)
        message.points = [point]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = RobotPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
