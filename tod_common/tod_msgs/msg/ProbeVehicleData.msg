std_msgs/Header header

# #probe ID
string vehicle_name
string vehicle_id

#vehicle type
uint16 vehicle_type

#vehicle status
float64 longitude
float64 latitude
float32 altitude
float32 heading

float32 steering_wheel
float32 acceleration
uint8 velocity

uint8 gear_status

uint16 vehicle_mile

uint8 vehicle_mode_status
#uint8 acc_status
#uint8 steer_status
#sensor status
uint8 gnss_status
uint8[] camera_status
# uint8[] lidar_status
# uint8[] radar_status
