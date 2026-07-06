# Dispatch-owned task snapshot for task list and RViz state publishing.

uint8 TYPE_DELIVERY=1
uint8 TYPE_INSPECTION=2
uint8 TYPE_RECHECK=3

uint8 RESULT_NONE=0
uint8 RESULT_OK=1
uint8 RESULT_ABNORMAL=2
uint8 RESULT_REJECT=3

string task_id
uint8 task_type
TaskState state
string assigned_robot_id
string preferred_robot_id
string created_by
builtin_interfaces/Time created_at
builtin_interfaces/Time updated_at
MissionStep[] steps
uint32 current_step_index
string parent_task_id
string excluded_robot_id
string[] locked_resource_ids
uint8 business_result
string message
