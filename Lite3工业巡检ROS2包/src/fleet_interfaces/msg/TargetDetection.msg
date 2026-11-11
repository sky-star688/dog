# Target detected by the inspection dog, published to fleet_manager.

std_msgs/Header header
string target_id
string inspector_id
string inspector_namespace
string class_name
float32 confidence
geometry_msgs/Point position
float32 position_covariance
string detection_source
int32 stable_frames
builtin_interfaces/Time first_seen_at
bool requires_operation
