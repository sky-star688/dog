#!/usr/bin/python

import rclpy
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler
from nav2_msgs.action import NavigateToPose
import time
from action_msgs.msg import GoalStatus

class TaskTransfer:
    def __init__(self, node):
        self.node = node
        self.move_base_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.move_base_client.wait_for_server()
        self.node.get_logger().info("Action 'navigate_to_pose' is up!")
        self.not_done = True

    def GoalResponseCallback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected :(')
            return
        self.node.get_logger().info('Goal accepted :)')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.GetResultCallback)

    def GetResultCallback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:            
            self.node.get_logger().info('Goal succeeded!')
            self.not_done = False
        else:
            self.node.get_logger().info('Goal failed with status: {0}'.format(status))

    def TaskTransfer(self, src_point, des_point):
        des_point.SetPreTaskPoint(src_point)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = des_point.GetPoseX()
        goal_msg.pose.pose.position.y = des_point.GetPoseY()
        goal_msg.pose.pose.position.z = 0.0
        my_q = quaternion_from_euler(0, 0, des_point.GetYaw())
        goal_msg.pose.pose.orientation.x = my_q[0]
        goal_msg.pose.pose.orientation.y = my_q[1]
        goal_msg.pose.pose.orientation.z = my_q[2]
        goal_msg.pose.pose.orientation.w = my_q[3]

        self.node.get_logger().info(
            "Transfer from [%s] to [%s]" % (src_point.name, des_point.name)
        )
        self.not_done = True
        self.send_goal_future = self.move_base_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.GoalResponseCallback)
        while self.not_done and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.5)
        self.not_done = True
        print("done")
