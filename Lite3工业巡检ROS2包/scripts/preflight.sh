#!/usr/bin/env bash
set -u

echo "== ROS version =="
printenv ROS_DISTRO || true

echo "== Lite3 services =="
systemctl is-active transfer_ros2 || true
systemctl is-active realsense_ros2 || true

echo "== ROS interfaces =="
ros2 topic list 2>/dev/null | grep -E 'cmd_vel|imu/data|joint_states|leg_odom|camera' || true

echo "== Arm serial candidates =="
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true

echo "== Model =="
ls -l "$HOME/lite3_industrial_inspection_ws/models/best.onnx" 2>/dev/null || true

echo "== Network =="
ip -br address
ip route
