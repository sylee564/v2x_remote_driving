std_msgs/Header header
# see VehicleEnums.h
uint8 indicator
uint8 emergencyLight
# int8 wiper
# int8 headLight
# int8 flashLight

uint8 gearPosition
uint8 Gear_Cmd
uint8 Gear_Control_Status

float32 WHEEL_SPD_FL
float32 WHEEL_SPD_FR
float32 WHEEL_SPD_RL
float32 WHEEL_SPD_RR

float32 BRK_CYLINDER

float32 steeringWheelAngle         # steering wheel angle position in rad
float32 steerWheelToque

float32 vehicle_velocity          # vehicle speed m/s
# float32 longitudinalAcc            # vehicle acceleration m/s^2
float32 curvature                  # current vehicle curvature in 1/m
         

geometry_msgs/Vector3 linearAcceleration    # m/s^2