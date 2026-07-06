# Dispatch-visible robot state snapshot.

uint8 IDLE=1
uint8 ASSIGNED=2
uint8 EXECUTING=3
uint8 WAITING_CONFIRMATION=4
uint8 RETURNING_HOME=5
uint8 PAUSED=6
uint8 ESTOP=7
uint8 ERROR=8
uint8 WAITING_RESOURCE=9

string robot_id
string robot_namespace
uint8 state
string current_task_id
string current_step_id
string current_point_id
geometry_msgs/PoseStamped pose
builtin_interfaces/Time stamp
string message
