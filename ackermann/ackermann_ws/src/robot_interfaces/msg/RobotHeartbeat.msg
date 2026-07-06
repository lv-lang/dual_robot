# Mission executor heartbeat published by each real robot.

uint8 IDLE=1
uint8 EXECUTING=2
uint8 CANCELING=3
uint8 ERROR=4
uint8 ESTOP=5

std_msgs/Header header
string robot_id
string robot_namespace
string map_version
uint8 mission_state
string message
