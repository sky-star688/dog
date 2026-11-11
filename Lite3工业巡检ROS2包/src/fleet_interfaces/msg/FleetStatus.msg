# Aggregated fleet status published by fleet_manager.

std_msgs/Header header
string fleet_state
FleetHeartbeat[] members
string active_handoff_id
WorkspaceState[] workspaces
TargetDetection[] pending_targets
int32 total_targets_detected
int32 total_targets_handled
int32 total_handoffs_completed
float32 fleet_uptime_sec
