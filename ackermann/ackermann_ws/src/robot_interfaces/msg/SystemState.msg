# Dispatch-owned real-system state.

uint8 WAITING_ROBOTS=1
uint8 STANDBY=2
uint8 READY=3
uint8 ESTOPPED=4
uint8 INTERLOCKED=5

std_msgs/Header header
uint8 state
bool task_creation_allowed
bool requires_operator_action
bool healthy_for_enable
string map_version
string message
string[] warnings
