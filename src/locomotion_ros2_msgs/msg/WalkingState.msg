std_msgs/Header header

uint8 LIFECYCLE_UNKNOWN=0
uint8 LIFECYCLE_UNCONFIGURED=1
uint8 LIFECYCLE_INACTIVE=2
uint8 LIFECYCLE_ACTIVE=3
uint8 LIFECYCLE_FAULT=4
uint8 LIFECYCLE_ESTOPPED=5
uint8 LIFECYCLE_FINALIZED=6

uint8 STATE_UNKNOWN=0
uint8 STATE_IDLE=1
uint8 STATE_STANDING=2
uint8 STATE_WALKING=3
uint8 STATE_TURNING=4
uint8 STATE_BODY_POSE_CONTROL=5
uint8 STATE_EXECUTING_FOOTSTEPS=6
uint8 STATE_STOPPING=7
uint8 STATE_SITTING=8
uint8 STATE_FALLEN=9
uint8 STATE_FAULT=10
uint8 STATE_ESTOPPED=11

uint8 MODE_UNKNOWN=0
uint8 MODE_IDLE=1
uint8 MODE_STAND=2
uint8 MODE_WALK=3
uint8 MODE_BODY_POSE=4
uint8 MODE_FOOTSTEP=5
uint8 MODE_SEMANTIC=6

uint8 SUPPORT_UNKNOWN=0
uint8 SUPPORT_DOUBLE=1
uint8 SUPPORT_LEFT=2
uint8 SUPPORT_RIGHT=3
uint8 SUPPORT_QUADRUPED=4

uint8 lifecycle_state
uint8 locomotion_state
uint8 locomotion_mode
uint8 support_phase
bool is_balanced
bool is_fallen
bool estop_active
bool adapter_connected
string active_adapter
string active_robot_model
string status_text
