#!/usr/bin/python

import socket
import struct
import rclpy
import threading
import Constants
import time

class RobotCommander:
    def __init__(self, local_port=20001, ctrl_ip="192.168.1.120", ctrl_port=43893):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        self.local_port = local_port
        self.ctrl_addr = (ctrl_ip, ctrl_port)
        self.comm_lock = threading.Lock()

        self.server.bind(("0.0.0.0", self.local_port))
        self.keep_alive = True
        self.keep_alive_thread = threading.Thread(
            target=self.KeepAliveThreadFunc, name="keep_alive"
        )
        self.keep_alive_thread.setDaemon(True)
        self.keep_alive_thread.start()

    def __del__(self):
        self.server = None
        self.keep_alive = False
        self.keep_alive_thread.join()

    def KeepAliveThreadFunc(self):
        while rclpy.ok() and self.keep_alive:
            if self.server:
                self.SendSimpleCommand(Constants.kHeartBeat)
                time.sleep(0.25)

    def SetAutoMode(self):
        self.SendSimpleCommand(Constants.kSetAutoMode)

    def SetHandleMode(self):
        self.SendSimpleCommand(Constants.kSetHandleMode)

    def StartContinuousMotionGait(self):
        self.SendSimpleCommand(Constants.kContinuousMotionGait, -1)

    def StopContinuousMotionGait(self):
        self.SendSimpleCommand(Constants.kContinuousMotionGait, 0)

    def SetEvenSlowSpeedGait(self):
        self.SendSimpleCommand(Constants.kEvenSlowSpeedGait)

    def SetEvenMediumSpeedGait(self):
        self.SendSimpleCommand(Constants.kEvenMediumSpeedGait)

    def SetUnevenHighStepGait(self):
        self.SendSimpleCommand(Constants.kUnevenHighStepGait)

    def SetZeroVelocity(self):
        self.SendComplexCommand(320, 8, 1, 0.0)
        self.SendComplexCommand(325, 8, 1, 0.0)
        self.SendComplexCommand(321, 8, 1, 0.0)

    def SendComplexCommand(self, command_code, command_value, command_type, value):
        data = struct.pack("<3id", command_code, command_value, command_type, value)
        self.comm_lock.acquire()
        if self.server:
            self.server.sendto(data, self.ctrl_addr)
        self.comm_lock.release()

    def SendSimpleCommand(self, command_code, command_value=0, command_type=0):
        data = struct.pack("<3i", command_code, command_value, command_type)
        self.comm_lock.acquire()
        if self.server:
            self.server.sendto(data, self.ctrl_addr)
        self.comm_lock.release()        

