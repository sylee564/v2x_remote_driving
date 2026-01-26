uint8 CONTROL_MODE_DIRECT = 0
uint8 CONTROL_MODE_INDIRECT = 1

uint8 VIDEO_MODE_MULTI = 0
uint8 VIDEO_MODE_MULTI_AVM  = 1
uint8 VIDEO_MODE_MULTI_PARKING = 2
uint8 VIDEO_MODE_MULTI_OBJECT  = 3

uint8 TOD_STATUS_IDLE = 0
uint8 TOD_STATUS_UPLINK_ONLY = 1
uint8 TOD_STATUS_TELEOPERATION = 2

uint8 MODE_MANUAL = 0
uint8 MODE_AUTO = 1
uint8 MODE_REMOTE = 2

uint8 STREAM_STOP = 0
uint8 STREAM_VIDEO_ONLY = 1
uint8 STREAM_AUDIO_ONLY = 2
uint8 STREAM_BOTH = 3

uint8 V2X_DISCONNECTED = 0
uint8 V2X_CONNECTING  = 1
uint8 V2X_CONNECTED_1 = 2
uint8 V2X_CONNECTED_2 = 3
uint8 V2X_CONNECTED_3 = 4
uint8 V2X_CONNECTED_4 = 5
uint8 V2X_CONNECTED_5 = 6
uint8 V2X_CONNECTED_6 = 7
uint8 V2X_ERROR = 100

std_msgs/Header operator_header

uint8 operator_control_mode
uint8 operator_video_mode
uint8 streaming_mode
uint8 operator_v2x_connected_status

uint8 mode
uint8 tod_status