export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/foxy/setup.bash

export ROS_LOG_DIR=/home/ysc/lite_cog_ros2/system/log
source /home/ysc/lite_cog_ros2/transfer/install/setup.bash
ros2 launch transfer transfer_launch.py & 


