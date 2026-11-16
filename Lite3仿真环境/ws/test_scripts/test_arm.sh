#!/usr/bin/env bash

source /opt/ros/foxy/setup.bash
source /root/sim_ws/install/setup.bash

xvfb-run -a ros2 launch lite3_sim spawn_lite3_kinematic.launch.py gui:=false > /tmp/launch.log 2>&1 &
pid=$!

sleep 8

echo "===== joint names ====="
timeout 4 ros2 topic echo /joint_states 2>&1 | sed -n '1,60p'

echo "===== arm frames in tf ====="
timeout 4 ros2 topic echo /tf 2>&1 | grep -E 'frame_id: arm_|child_frame_id: arm_|frame_id: gripper_|child_frame_id: gripper_' | head -20

echo "===== topics ====="
ros2 topic list | grep -E 'arm|gripper|set_joint'

echo "===== launch errors ====="
grep -iE 'error|exception|failed' /tmp/launch.log | tail -20

kill "$pid" 2>/dev/null || true
wait "$pid" 2>/dev/null || true
