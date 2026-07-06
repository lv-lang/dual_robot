# Low-bandwidth robot-local health summary for real robot deployments.

uint8 OK=1
uint8 WARN=2
uint8 ERROR=3
uint8 ESTOP=4

std_msgs/Header header
string robot_id
string robot_namespace
string map_version
string map_bundle_hash
geometry_msgs/PoseStamped map_pose
float32 scan_age_sec
float32 odom_age_sec
float32 tf_age_sec
bool scan_ok
bool odom_ok
bool amcl_active
bool nav2_active
bool map_to_odom_ok
bool mission_ready
uint8 health_state
string[] reasons
