# nr_v2x_msgs/ModemTxStatus.msg
# --- constants (vendor spec) ---
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

# --- common header ---
std_msgs/Header header               # stamp=ROS 수신 시각

# --- common fields from TLVC status ---
uint8  dev_type                      # =10 or 20 here
uint8  tx_rx                         # =0 (Tx)
uint32 dev_id
uint16 hw_ver
uint16 sw_ver
uint64 device_timestamp              # 장비 타임스탬프(be64→host)

# --- modem TX specific (TLVC_STATUS_Tx_ModemUnit) ---
int8   tx_power
uint16 freq                          # 원 단위 그대로(벤더 스펙)
uint8  bandwidth
uint8  scs
uint8  mcs
int32  latitude                      # 원 데이터(예: 1e-7 deg scale라면 상위에서 변환)
int32  longitude

# --- V2 확장 프레임에서만 존재할 수 있는 항목 ---
int8   cpu_temp                      # 없으면 TEMPERATURE_NO_MEASURE로 세팅
int8   peri_temp                     # 없으면 TEMPERATURE_NO_MEASURE
