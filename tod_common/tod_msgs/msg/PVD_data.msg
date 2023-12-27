std_msgs/Header header
#probe ID
string vehicle_name
string vehicle_id

#vehicle type
uint16 vehicle_type

#vehicle status
float64 longitude
float64 latitude
float32 elevation
float32 heading

uint8 gear_status
uint8 velocity
int32 steering_wheel
float32 acceleration
uint16 vehicle_mile

uint8 lat_approved      
uint8 long_approved
uint8 vehicle_mode

#sensor status
uint8 gnss_status
uint8[] camera_status
uint8[] lidar_status
uint8[] radar_status
