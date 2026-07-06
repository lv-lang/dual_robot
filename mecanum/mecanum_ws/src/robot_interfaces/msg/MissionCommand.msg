# Dispatch-to-robot command consumed by one mission executor action server.

uint8 TYPE_DELIVERY=1
uint8 TYPE_INSPECTION=2
uint8 TYPE_RECHECK=3

string command_id
string task_id
uint8 task_type
string assigned_robot_id
MissionStep[] steps
bool return_home_after_steps
MissionStep return_home_step
string parent_task_id
string excluded_robot_id
string[] locked_resource_ids
builtin_interfaces/Time issued_at
string note
