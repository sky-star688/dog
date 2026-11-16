#!/usr/bin/env python3

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
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

STAND_POSE = [
    0.0, -0.652, 1.272,
    0.0, -0.652, 1.272,
    0.0, -0.652, 1.272,
    0.0, -0.652, 1.272,
]


class StandPose(Node):
    def __init__(self):
        super().__init__("stand_pose")
        self.publisher = self.create_publisher(
            JointTrajectory, "/set_joint_trajectory", 10
        )
        self.create_timer(0.02, self.publish_pose)

    def publish_pose(self):
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "TORSO"
        message.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = STAND_POSE
        point.time_from_start = Duration(sec=0, nanosec=500_000_000)
        message.points = [point]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = StandPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
