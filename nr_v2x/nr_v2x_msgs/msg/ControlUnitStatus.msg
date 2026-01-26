# nr_v2x_msgs/ControlUnitStatus.msg
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

uint8  dev_type                      # =22
uint8  tx_rx
uint32 dev_id
uint16 hw_ver
uint16 sw_ver
uint64 device_timestamp

int8   cpu_temp                      # V2만 존재, 아니면 NO_MEASURE
int8   peri_temp
