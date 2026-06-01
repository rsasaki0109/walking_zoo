std_msgs/Header header

uint8 STATE_UNKNOWN=0
uint8 STATE_OK=1
uint8 STATE_LIMITED=2
uint8 STATE_BLOCKED=3
uint8 STATE_ESTOPPED=4
uint8 STATE_FAULT=5

uint8 state
bool estop_active
bool command_stale
bool fall_detected
bool adapter_healthy
float32 max_linear_x
float32 max_linear_y
float32 max_angular_z
string status_text
