uint8 TOD_CONTROL_RAW = 0
uint8 TOD_CONTROL_ACC = 1
uint8 TOD_CONTROL_VEL = 2

std_msgs/Header header

string operator_id
tod_msgs/RawControlCommand control
tod_msgs/Shift shift
tod_msgs/TurnSignal indicator

float32 velocity
float32 acceleration

uint8 control_type

bool remote_flag
bool stream_flag
bool avm_flag
bool parking_flag
bool object_flag
bool aeb_flag
bool cruise_flag