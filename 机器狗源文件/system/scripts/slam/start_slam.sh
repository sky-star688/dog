#!/bin/sh

# 开启rviz
gnome-terminal -x bash -c "source /home/ysc/lite_cog_ros2/slam/install/setup.bash; ros2 launch faster_lio mapping_c16.launch.py; read -p 'Press Enter to exit...'"

#开启生成grid_map终端
gnome-terminal -x bash -c "bash /home/ysc/lite_cog_ros2/system/scripts/slam/gridmap.sh; read -p 'Press Enter to exit...'"

# 开启保存地图终端
gnome-terminal -x bash -c "bash /home/ysc/lite_cog_ros2/system/scripts/slam/save_map.sh; read -p 'Press any key to exit...'; read -p 'Press Enter to exit...'"
