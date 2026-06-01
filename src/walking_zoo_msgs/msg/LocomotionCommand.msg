std_msgs/Header header

uint8 TYPE_UNKNOWN=0
uint8 TYPE_VELOCITY=1
uint8 TYPE_BODY_POSE=2
uint8 TYPE_FOOTSTEP_PLAN=3
uint8 TYPE_SEMANTIC_ACTION=4
uint8 TYPE_STOP=5

string source
uint8 command_type
uint8 locomotion_mode
geometry_msgs/TwistStamped twist
BodyPoseCommand body_pose
FootstepPlan footstep_plan
float32 priority
float32 timeout_sec
