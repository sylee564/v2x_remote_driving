std_msgs/Header header
uint32 operator_id
uint8 control_type
bool remote_flag
bool streaming_flag
bool avm_flag
float64 steeringWheelAngle     # radian
# int32 steeringWheelAngle     # degree
uint8 gearPosition
float32 velocity               # velocity control mode: desired speed; acceleration control mode: maximum speed
float32 acceleration           # velocity control mode: acceleration control mode: desired acceleration
uint8 indicator                # 0:off, 1:emergency, left:2, right:4
# uint8 honk
# uint8 wiper
# uint8 headLight
# uint8 flashLight
