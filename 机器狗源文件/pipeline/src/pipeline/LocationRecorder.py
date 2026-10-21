#!/usr/bin/python

from PyQt5.QtWidgets import (
    QWidget,
    QApplication,
    QLabel,
    QHBoxLayout,
    QVBoxLayout,
    QPushButton,
    QLineEdit,
    QTextEdit,
    QRadioButton
)
from PyQt5.QtGui import QIntValidator

import sys
import rclpy
import json
import os
import tf2_ros

import Constants
from rclpy.time import Time
from rclpy.duration import Duration
import time

import signal
import threading

break_flag = False

def signal_handler(sig, frame):
    global break_flag
    print('Ctrl+C detected. Setting break_flag to True.')
    break_flag = True

class LocationRecorder(QWidget):
    def __init__(self, language, node):
        super(LocationRecorder, self).__init__()
        self.InitUI(language)
        self.robot_record_pose = None
        self.option = None
        self.node = node
        self.buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer, self.node)
        self.show()

        self.thread = threading.Thread(target=self.SpinThreadFunc)
        self.thread.daemon = True
        self.thread.start()

    def __del__(self):
        global break_flag
        break_flag = True
        self.thread.join()
        pass

    def InitUI(self, language):
        self.setWindowTitle('DR Location Recorder')

        self.layout = QVBoxLayout()
        self.option_layout = QHBoxLayout()
        self.order_layout = QHBoxLayout()
        self.record_layout = QHBoxLayout()

        if(language == Constants.kChinese):
            self.even_low_speed_radio_button = QRadioButton('平地低速步态')
            self.even_medium_speed_radio_button = QRadioButton('平地中速步态')
            self.uneven_high_step_radio_button = QRadioButton("越障高踏步步态")
            self.order_layout.addWidget(QLabel("位点编号:"))
            self.receive_button = QPushButton("获取位点")
            self.record_button = QPushButton("记录位点")

        elif(language == Constants.kEnglish): 
            self.even_low_speed_radio_button = QRadioButton('even slow gait')
            self.even_medium_speed_radio_button = QRadioButton('even medium gait')
            self.uneven_high_step_radio_button = QRadioButton("uneven high step gait")
            self.order_layout.addWidget(QLabel("location number:"))
            self.receive_button = QPushButton("get location")
            self.record_button = QPushButton("record location")

        self.order_edit = QLineEdit("")
        self.order_edit.setValidator(QIntValidator())
        self.text_content = QTextEdit()
        self.text_content.setReadOnly(True)
        self.even_low_speed_radio_button.setChecked(True)

        self.option_layout.addWidget(self.even_low_speed_radio_button)
        self.option_layout.addWidget(self.even_medium_speed_radio_button)        
        self.option_layout.addWidget(self.uneven_high_step_radio_button)
        self.order_layout.addWidget(self.order_edit)
        self.record_layout.addWidget(self.receive_button)
        self.record_layout.addWidget(self.record_button)
        self.layout.addLayout(self.option_layout)
        self.layout.addLayout(self.order_layout)
        self.layout.addWidget(self.text_content)
        self.layout.addLayout(self.record_layout)
        self.setLayout(self.layout)

        self.receive_button.clicked.connect(self.Receive)
        self.record_button.clicked.connect(self.Record)

    def SpinThreadFunc(self):
        global break_flag
        while not break_flag and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def ListenTF(self):
        try:
            
            t = self.buffer.lookup_transform(
                "map", "base_link", Time(), Duration(seconds=1, nanoseconds=0)
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
            self.robot_record_pose = msg_dict
            return True
        except tf2_ros.TransformException as e:
            print("listen to tf failed")
            return False
        except: 
            exc_type, exc_value, exc_traceback = sys.exc_info()
            print("other fault")
            print("exc_type:", exc_type)
            print("exc_value:", exc_value)
            print("exc_traceback", exc_traceback)
            return False

    def UpdateOption(self):
        self.option = {}
        self.option["even_low_speed"] = self.even_low_speed_radio_button.isChecked()
        self.option["even_medium_speed"] = self.even_medium_speed_radio_button.isChecked()
        self.option["uneven_high_step"] = self.uneven_high_step_radio_button.isChecked()

    def Receive(self):
        while not self.ListenTF():
            global break_flag
            if break_flag:
                print("break_flag is true, exiting...")
                break
            time.sleep(1)
        self.UpdateOption()
        display_msg = "Robot:\n" + json.dumps(self.robot_record_pose, indent=4) + "\n"
        display_msg += "Option:\n" + json.dumps(self.option, indent=4) + "\n"
        self.text_content.setText(display_msg)

    def Record(self):
        order = self.order_edit.text()
        order = int(order)
        new_record = {
            "order": order,
            "robot_pose": self.robot_record_pose,
            "option": self.option,
        }
        data_dir = str(os.path.dirname(os.path.abspath(__file__))) + "/../data"
        os.system("mkdir -p " + data_dir)
        data_dir = data_dir + "/" + str(order) + ".json"
        with open(data_dir, "w+") as out:
            json.dump(new_record, out, indent=4)

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init()
    node = rclpy.create_node('location_recorder')
    app = QApplication(sys.argv)
    lr = LocationRecorder(Constants.kChinese, node)
    app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit()

if __name__ == "__main__":
    main()
