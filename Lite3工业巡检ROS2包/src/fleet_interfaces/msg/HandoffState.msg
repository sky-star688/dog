# Current state of a target handoff between two robots.

string handoff_id
string target_id
string state
string from_robot
string to_robot
builtin_interfaces/Time requested_at
builtin_interfaces/Time accepted_at
builtin_interfaces/Time completed_at
float32 elapsed_sec
geometry_msgs/Pose target_pose
geometry_msgs/Pose fine_pose
bool fine_pose_valid
bool success
string error_message
