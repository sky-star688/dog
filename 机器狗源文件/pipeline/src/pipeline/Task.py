#!/usr/bin/python

import os
import json
import rclpy
from rclpy.node import Node
from TaskPoint import TaskPoint
from TaskTransfer import TaskTransfer
import numpy as np
import tf2_ros
from RobotCommander import RobotCommander
import time
import sys

class Task(Node):
    def __init__(self):
        super().__init__("pipeline")
        self.buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer, self)
        self.task_points = []
        self.src_index = 0
        self.des_index = 0
        self.ntask = 0
        self.robot_commander = RobotCommander()
        self.task_transfer = TaskTransfer(self)

    def LoadTaskpoints(self):
        folder = str(os.path.dirname(os.path.abspath(__file__))) + "/../data"
        task_json = None
        if os.path.exists(folder):
            task_json = os.listdir(folder)

        if not task_json:
            raise Exception("No valid task point to tranverse!")

        task_list = []
        for i, file_name in enumerate(task_json):
            with open(folder + "/" + file_name, "r") as json_fp:
                waypoint_record = json.load(json_fp)
                task_list.append(waypoint_record)
        task_list = sorted(task_list, key=lambda s: s["order"])
        for waypoint_record in task_list:
            self.task_points.append(TaskPoint(waypoint_record))

    def GetInitialPose(self):
        self.initial_pose = None
        RATE = 50
        while not self.initial_pose:    
            try:
                rclpy.spin_once(self, timeout_sec=0.5)
                t = self.buffer.lookup_transform(
                    "map", "base_link", rclpy.time.Time(), rclpy.duration.Duration(seconds=1, nanoseconds=0)
                )
                msg_dict = {
                    "pos_x": t.transform.translation.x,
                    "pos_y": t.transform.translation.y,
                    "pos_z": t.transform.translation.z,
                    "ori_x": t.transform.rotation.x,
                    "ori_y": t.transform.rotation.y,
                    "ori_z": t.transform.rotation.z,
                    "ori_w": t.transform.rotation.w,
                }
                self.initial_pose = msg_dict
                time.sleep(1.0 / RATE)
            except tf2_ros.TransformException as e:
                print ("listen to tf failed")
            except:
                exc_type, exc_value, exc_traceback = sys.exc_info()
                print("other fault")
                print("exc_type:", exc_type)
                print("exc_value:", exc_value)
                print("exc_traceback", exc_traceback)

    def GetBestTaskIndex(self):
        self.GetInitialPose()
        initial_task_point = TaskPoint()
        initial_task_point.SetRobotPose(self.initial_pose)
        dist_list = [initial_task_point.CalDistance(task_point) for task_point in self.task_points]
        return np.argmin(np.array(dist_list)), initial_task_point

    def Init(self):
        # load points
        self.LoadTaskpoints()
        nearest_index, initial_point = self.GetBestTaskIndex()

        # go to the nearest point
        self.task_transfer.TaskTransfer(initial_point, self.task_points[nearest_index])

        # set next point
        self.ntask = self.task_points.__len__()
        self.src_index = nearest_index
        self.des_index = (nearest_index + 1) % self.ntask

    def Run(self):
        while rclpy.ok():            
            # start continuos motion
            self.robot_commander.StartContinuousMotionGait()
            time.sleep(0.5)

            self.robot_commander.SetZeroVelocity()
            time.sleep(0.5)

            # switch gait
            if(self.task_points[self.src_index].IsEvenLowSpeedGait()):
                self.robot_commander.SetEvenSlowSpeedGait()
                print("SetEvenSlowSpeedGait")
            if(self.task_points[self.src_index].IsEvenMediumSpeedGait()):
                self.robot_commander.SetEvenMediumSpeedGait()
                print("SetEvenMediumSpeedGait")
            if(self.task_points[self.src_index].IsUnevenHighStepGait()):
                self.robot_commander.SetUnevenHighStepGait()
                print("SetUnevenHighStepGait")
            time.sleep(0.5)

            # switch point
            self.task_transfer.TaskTransfer(
                self.task_points[self.src_index], self.task_points[self.des_index]
            )
            self.src_index = self.des_index
            self.des_index = (self.des_index + 1) % self.ntask
            time.sleep(0.5)

        # stop continuos motion
        self.robot_commander.StopContinuousMotionGait()

if __name__ == "__main__":
    rclpy.init()
    task = Task()
    task.Init()
    task.Run()
