export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/foxy/setup.bash

export ROS_LOG_DIR=/home/ysc/lite_cog_ros2/voa/run_log
source /home/ysc/lite_cog_ros2/voa/install/setup.bash
ros2 launch voa voa_launch.py &
