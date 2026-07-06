# Business-point resource lock snapshot.

uint8 TYPE_WAITING_AREA=1
uint8 TYPE_PICKUP=2
uint8 TYPE_DELIVERY=3
uint8 TYPE_INSPECTION=4
uint8 TYPE_RECHECK=5

uint8 STATUS_FREE=0
uint8 STATUS_LOCKED=1
uint8 STATUS_SHARED_ABNORMAL=2

string resource_id
uint8 resource_type
uint8 status
string point_id
geometry_msgs/PoseStamped pose
string locked_by_task_id
string locked_by_robot_id
string[] shared_task_ids
builtin_interfaces/Time stamp
string reason
