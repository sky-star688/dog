#!/usr/bin/python

from LocationRecorder import LocationRecorder
import rclpy
import sys
import Constants
from PyQt5.QtWidgets import (
    QApplication,
)

def main(args=None):
    rclpy.init()
    node = rclpy.create_node('location_recorder')
    app = QApplication(sys.argv)
    lr = LocationRecorder(Constants.kEnglish, node)
    app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit()

if __name__ == "__main__":
    main()
