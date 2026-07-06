# One atomic mission step expressed in the shared /map frame.

uint8 STEP_NAVIGATE=1
uint8 STEP_WAIT_CONFIRMATION=2
uint8 STEP_RETURN_HOME=3

uint8 CONFIRMATION_NONE=0
uint8 CONFIRMATION_OK=1
uint8 CONFIRMATION_ABNORMAL=2
uint8 CONFIRMATION_REJECT=3

uint32 sequence
uint8 step_type
string step_id
string point_id
geometry_msgs/PoseStamped target_pose
bool requires_confirmation
builtin_interfaces/Duration timeout
string resource_id
string label
