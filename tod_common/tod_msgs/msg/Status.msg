uint8 mode_status
bool connect_flag
std_msgs/Header operator_header

uint8 CONTROL_MODE_DIRECT = 0
uint8 CONTROL_MODE_INDIRECT = 1
# uint8 CONTROL_MODE_WAYPOINT = 2
# uint8 CONTROL_MODE_PATH_GUIDANCE = 3
# uint8 CONTROL_MODE_PERCEPTION_MODIFICATION = 4
# uint8 CONTROL_MODE_SAFECORRIDOR = 5
# uint8 CONTROL_MODE_NONE = 99

uint8 VIDEO_RATE_CONTROL_MODE_SINGLE = 0
uint8 VIDEO_RATE_CONTROL_MODE_MULTIPLE = 1
uint8 VIDEO_RATE_CONTROL_MODE_AUTOMATIC = 2

uint8 TOD_STATUS_IDLE = 0
uint8 TOD_STATUS_UPLINK_ONLY = 1
uint8 TOD_STATUS_TELEOPERATION = 2


int8 operator_input_device                 # Details at include/tod_msgs/inputDevice.h

string operator_device_ip_address
string operator_PC_ip_address
uint32 operator_device_port

uint8 operator_control_mode
uint8 operator_video_mode

uint8 tod_status
uint16[] oper_v2x_header

uint8 gear
bool stream_flag

std_msgs/Header vehicle_header

string vehicle_device_ip_address
string vehicle_PC_ip_address
uint32 vehicle_device_port

uint8 vehicle_control_mode

uint16[] vehicle_v2x_header

bool vehicle_emergency_stop_released
bool vehicle_ACC_approved
bool vehicle_EPS_approved
bool vehicle_EPS_Status
bool vehicle_ACC_Status