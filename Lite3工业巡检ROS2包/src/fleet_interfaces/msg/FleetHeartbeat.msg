# Heartbeat published by each robot to fleet_manager.

std_msgs/Header header
string robot_id
string robot_role
string state
string current_workspace
string target_workspace
string mission_id
int32 waypoint_index
int32 waypoint_total
bool localization_ok
bool arm_ready
bool arm_busy
bool estop
bool battery_low
geometry_msgs/Pose current_pose
float32 battery_voltage
