# std_msgs/Header header
tod_msgs/V2XDataBase v2x_db

uint32 operator_id
tod_msgs/RawControlCommand control
tod_msgs/Shift shift
tod_msgs/TurnSignal indicator

float32 velocity
float32 acceleration

uint8 control_type
uint8 steer_mode
uint8 drive_mode

uint8 stream_flag
uint8 remote_flag
uint8 aeb_flag
# bool parking_flag
# bool avm_flag