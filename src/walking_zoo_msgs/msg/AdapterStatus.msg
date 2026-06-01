std_msgs/Header header

uint8 STATUS_UNKNOWN=0
uint8 STATUS_DISCONNECTED=1
uint8 STATUS_CONNECTED=2
uint8 STATUS_ACTIVE=3
uint8 STATUS_FAULT=4
uint8 STATUS_ESTOPPED=5

uint8 status
bool connected
bool active
bool allow_motion
string adapter_name
string robot_model
string hardware_id
string status_text
