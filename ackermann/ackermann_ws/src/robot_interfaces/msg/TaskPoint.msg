# Business point known by robot_dispatch.

uint8 KIND_WAITING_AREA=1
uint8 KIND_PICKUP=2
uint8 KIND_DELIVERY=3
uint8 KIND_INSPECTION=4

string point_id
uint8 kind
string label
geometry_msgs/PoseStamped pose
bool temporary
builtin_interfaces/Time stamp
