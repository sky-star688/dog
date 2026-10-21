#!/bin/bash

#输入save时才保存地图
echo "INPUT: 1 (when you want to creat the grid map )"
read COMMAND
case "$COMMAND" in
    "1")
        echo "YYYYYY"
        source /home/ysc/lite_cog_ros2/slam/install/setup.bash
        ros2 launch pcd2grid pcd2grid.launch.py
        ;;
    *)
        echo "wrong"
        ;;
esac

