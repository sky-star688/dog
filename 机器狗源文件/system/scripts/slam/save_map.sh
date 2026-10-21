#!/bin/bash

#输入save时才保存地图
echo "INPUT: 2 (when you want to save the grid map and the point cloud)"
read COMMAND
case "$COMMAND" in
    "2")
        echo "YYYYYY"
	rm /home/ysc/lite_cog_ros2/system/map/lite3.pgm  
	rm /home/ysc/lite_cog_ros2/system/map/lite3.yaml
	source /home/ysc/lite_cog_ros2/slam/install/setup.bash
	ros2 service call /map_saver_server/save_map nav2_msgs/srv/SaveMap '{
map_topic: /projected_map, 
map_url: /home/ysc/lite_cog_ros2/system/map/lite3, 
image_format: pgm,  
map_mode: trinary, 
free_thresh: 0.25, 
occupied_thresh: 0.65 
}'	
        ;;
    *)
        echo "wrong"
        ;;
esac

