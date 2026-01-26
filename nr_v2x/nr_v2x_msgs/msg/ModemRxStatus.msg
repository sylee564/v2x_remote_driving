# nr_v2x_msgs/ModemRxStatus.msg
uint8 DEVTYPE_OBU_MODEM=10
uint8 DEVTYPE_OBU=11
uint8 DEVTYPE_RSU_MODEM=20
uint8 DEVTYPE_RSU=21
uint8 DEVTYPE_RSU_CONTROL=22

uint8 TXRX_TX=0
uint8 TXRX_RX=1

int8  TEMPERATURE_NO_MEASURE=-128
int8  TEMPERATURE_LIMIT_OVER=127
int8  TEMPERATURE_LIMIT_BELOW=-127

std_msgs/Header header

uint8  dev_type                      # =10 or 20 here
uint8  tx_rx                         # =1 (Rx)
uint32 dev_id
uint16 hw_ver
uint16 sw_ver
uint64 device_timestamp

int8   rssi
uint8  rcpi
int32  latitude
int32  longitude

int8   cpu_temp                      # 없으면 TEMPERATURE_NO_MEASURE
int8   peri_temp                     # 없으면 TEMPERATURE_NO_MEASURE
