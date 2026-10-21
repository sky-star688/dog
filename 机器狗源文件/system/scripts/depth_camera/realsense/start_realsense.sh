export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/foxy/setup.bash

export ROS_LOG_DIR=/home/ysc/lite_cog_ros2/driver/realsense_ws/run_log
source /home/ysc/lite_cog_ros2/driver/realsense_ws/install/setup.bash
ros2 launch realsense2_camera dr_camera_launch.py & 

