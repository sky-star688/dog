# Workspace reservation state.

string workspace_id
string state
string reserved_by
string reserved_for_target
builtin_interfaces/Time reserved_at
builtin_interfaces/Time expires_at
geometry_msgs/Polygon boundary
geometry_msgs/Point approach_point
geometry_msgs/Point retreat_point
