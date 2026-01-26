//=============================================================================
// This confidential and proprietary software may be used only as authorized by
// a licensing agreement from CHEMTRONICS Limited.
//
// COPYRIGHT (c) CHEMTRONICS. ALL RIGHTS RESERVED.
//
// The entire notice above must be reproduced on all authorized copies and
// copies may only be made to the extent permitted by a licensing agreement
// from CHEMTRONICS Limited.
//
// Module: v2x_app_ext.h
// Description: Header of V2X Interface & Extensible Message
//
// Update History
// [11/06/2022 jongsik.kim] create
//=============================================================================
#ifndef _5GNR_V2X_APP_EXT_H_
#define _5GNR_V2X_APP_EXT_H_

#include <stdint.h>

#define V2X_INF_EXT_MAGIC								"5GVX"


// 확장 메시지용
#define EM_V2V_MSG										(58200)
#define EM_V2I_MSG										(58201)
#define EM_I2V_MSG										(58202)

#define EM_PT_OVERALL									(58220) // overall package
#define EM_PT_RAW_DATA									(58221) // raw data package
#define EM_PT_SSOV										(58222) // SSOV package 또는 SSOV PSID
													// cf) SSOV의 응용 서비스는 SSOV format 내부 Service ID로 구분
#define EM_PT_STATUS									(58223) // status package
#define EM_PT_VIDEO										(58224) // video
#define EM_PT_TOD										(58225) // video

#define EM_FTP_REQ										(58240)
#define EM_FTP_RESP										(58241)

// #define TEMPERATURE_NO_MEASURE							(-128)
// #define TEMPERATURE_LIMIT_OVER							(127)
// #define TEMPERATURE_LIMIT_BELOW							(-127)

#define UNIT_ID_OBU_MODEM								(0x1)
#define UNIT_ID_OBU_COMMUNICATION						(0x2)
//#define UNIT_ID_OBU_CONTROL							(0x4)
#define UNIT_ID_RSU_MODEM								(0x10)
#define UNIT_ID_RSU_COMMUNICATION						(0x20)
#define UNIT_ID_RSU_CONTROL								(0x40)

typedef enum
{
	reserved0 = 0x00,
	ePayloadId_ACK,
	ePayloadId_NAK,
	ePayloadId_heartbeat,
	ePayloadId_time,
	ePayloadId_status,
	ePayloadId_repair,
	ePayloadId_cmd,
	ePayloadId_TxMsg = 0x10,
	ePayloadId_RxMsg,
	ePayloadId_WsmServiceReq,
	ePayloadId_WsmServiceConfirm,
	ePayloadId_MAX = ePayloadId_WsmServiceConfirm
} eV2x_App_Payload_Id;

typedef enum
{
	eWSCActionResult_Fail = 0x00,
	eWSCActionResult_Add_OK,
	eWSCActionResult_Already_Add,
	eWSCActionResult_Del_OK,
	eWSCActionResult_Not_Exist_Del
} eV2x_App_WSC_Action_Result;

typedef enum
{
	eStatusDevType_ObuModem = 10,
	eStatusDevType_Obu = 11,
	eStatusDevType_RsuModem = 20,
	eStatusDevType_Rsu = 21,
	eStatusDevType_RsuControl = 22
} eV2x_App_Ext_Status_DevType;

typedef enum
{
	eStatusTxRx_Tx = 0,
	eStatusTxRx_Rx = 1
} eV2x_App_Ext_Status_TxRx;

/*
|           Header            |   Data  |  Tail |
| Magic | len | seq |payloadId| Payload | crc16 |
|-------|-----|-----|---------|---------|-------|
|   4   |  2  |  2  |    2    |    n    |   2   |

*/
typedef struct _V2x_App_Hdr
{
	char		magic[4];
	uint16_t	len;			// network byte oder
	uint16_t	seq;			// network byte oder
	uint16_t	payload_id;		// network byte oder
	uint8_t		data[0];
}__attribute__((__packed__)) V2x_App_Hdr;

/*
Data struct (Payload 부분)
 - Tx Message (payload Id = 0x10)
	| PSID | message |
	|------|---------|
	|  4   |    n    |
 
 - Rx Message (payload Id = 0x11)
	| PSID | RCPI | message |
	|------|------|---------|
	|  4   |   1  |    n    |
*/
typedef struct _V2x_App_WSR_Add_Crc
{
	uint8_t		action;
	uint32_t	psid;			// network byte oder
	uint16_t	crc;
}__attribute__((__packed__)) V2x_App_WSR_Add_Crc;

typedef struct _V2x_App_WSC_Add_Crc
{
	uint8_t		action_result;
	uint32_t	psid;			// network byte oder
	uint16_t	crc;
}__attribute__((__packed__)) V2x_App_WSC_Add_Crc;

typedef struct _V2x_App_TxMsg
{
	uint32_t	psid;			// network byte oder
	uint8_t  castMode;
	uint32_t srcId;
	uint32_t dstId;
	uint8_t  reserved[8];
	uint8_t		data[0];
//	uint16_t	crc;			// 마지막에 추가 필요
}__attribute__((__packed__)) V2x_App_TxMsg;

typedef struct _V2x_App_RxMsg
{
	uint32_t	psid;			// network byte oder
	uint8_t  castMode;
	uint32_t srcId;
	uint32_t dstId;
	uint8_t  reserved[8];
	uint8_t		rcpi;
	uint8_t		data[0];
//	uint16_t	crc;			// 디바이스로 전송시에는 마지막에 추가 필요
}__attribute__((__packed__)) V2x_App_RxMsg;

/*
Extensible Message (payload 부분)
 - | PSID | T L V C |
   |------|---------|
   |  4   | 4 2 N 2 |

	PSID : 58200(V2V), 58201(V2I), 58202(I2V)
	T : 58220(Overall), 588221(Raw Data), 58222(SSOV), 58223(Status)
	L : 
	V : 
	C : CRC16
 */

// Define TLVC of Extensible Message
typedef struct _V2x_App_Ext_TLVC
{
	uint32_t	type;			// T : network byte oder
	uint16_t	len;			// L : network byte oder
	uint8_t		data[0];		// V
	uint16_t	crc;			// C : 마지막에 추가 필요
}__attribute__((__packed__)) V2x_App_Ext_TLVC;

typedef struct _TLVC_Overall
{
	uint32_t	type;			// T : network byte oder - 58220
	uint16_t	len;			// L : network byte oder
	char		magic[4];		// V : EMOP
	uint8_t		version;		// V
	uint8_t		num_package;	// V
	uint16_t	len_package;	// V : network byte order
	uint16_t	crc;			// C : network byte order
}__attribute__((__packed__)) TLVC_Overall;

typedef struct _TLVC_Overall_V2
{
	uint32_t	type;			// T : network byte oder - 58220
	uint16_t	len;			// L : network byte oder
	char		magic[4];		// V : EMOP
	uint8_t		version;		// V
	uint8_t		num_package;	// V
	uint16_t	len_package;	// V : network byte order
	uint8_t		bitwize;
	uint16_t	crc;			// C : network byte order
}__attribute__((__packed__)) TLVC_Overall_V2;

typedef struct _TLVC_STATUS_Tx_ModemUnit
{
	uint32_t	type;			// T : network byte oder - 58223
	uint16_t	len;			// L : network byte oder
	uint8_t		dev_type;		// obu_modem:10, obu:11, rsu_modem:20, rsu:21, rsu_control:22
	uint8_t		tx_rx;			// tx:0, rx:1
	uint32_t	dev_id;
	uint16_t	hw_ver;
	uint16_t	sw_ver;
	uint64_t	timestamp;
	uint8_t		tx_power;
	uint16_t	freq;
	uint8_t		bandwidth;
	uint8_t		scs;			// Subcarrier spacing
	uint8_t		mcs;
	int32_t		latitude;
	int32_t		longitude;
	uint16_t	crc;			// C : network byte order
}__attribute__((__packed__)) TLVC_STATUS_Tx_ModemUnit;

typedef struct _TLVC_STATUS_Tx_ModemUnit_V2
{
	uint32_t	type;			// T : network byte oder - 58223
	uint16_t	len;			// L : network byte oder
	uint8_t		dev_type;		// obu_modem:10, obu:11, rsu_modem:20, rsu:21, rsu_control:22
	uint8_t		tx_rx;			// tx:0, rx:1
	uint32_t	dev_id;
	uint16_t	hw_ver;
	uint16_t	sw_ver;
	uint64_t	timestamp;
	uint8_t		tx_power;
	uint16_t	freq;
	uint8_t		bandwidth;
	uint8_t		scs;			// Subcarrier spacing
	uint8_t		mcs;
	int32_t		latitude;
	int32_t		longitude;
	char		cpu_temp;
	char		peri_temp;
	uint16_t	crc;			// C : network byte order
}__attribute__((__packed__)) TLVC_STATUS_Tx_ModemUnit_V2;

typedef struct _TLVC_STATUS_Rx_ModemUnit
{
	uint32_t	type;			// T : network byte oder - 58223
	uint16_t	len;			// L : network byte oder
	uint8_t		dev_type;		// obu_modem:10, obu:11, rsu_modem:20, rsu:21, rsu_control:22
	uint8_t		tx_rx;			// tx:0, rx:1
	uint32_t	dev_id;
	uint16_t	hw_ver;
	uint16_t	sw_ver;
	uint64_t	timestamp;
	int8_t		rssi;
	uint8_t		rcpi;
	int32_t		latitude;
	int32_t		longitude;
	uint16_t	crc;			// C : network byte order
}__attribute__((__packed__)) TLVC_STATUS_Rx_ModemUnit;

typedef struct _TLVC_STATUS_Rx_ModemUnit_V2
{
	uint32_t	type;			// T : network byte oder - 58223
	uint16_t	len;			// L : network byte oder
	uint8_t		dev_type;		// obu_modem:10, obu:11, rsu_modem:20, rsu:21, rsu_control:22
	uint8_t		tx_rx;			// tx:0, rx:1
	uint32_t	dev_id;
	uint16_t	hw_ver;
	uint16_t	sw_ver;
	uint64_t	timestamp;
	int8_t		rssi;
	uint8_t		rcpi;
	int32_t		latitude;
	int32_t		longitude;
	char		cpu_temp;
	char		peri_temp;
	uint16_t	crc;			// C : network byte order
}__attribute__((__packed__)) TLVC_STATUS_Rx_ModemUnit_V2;

typedef struct _TLVC_STATUS_CommUnit
{
	uint32_t	type;			// T : network byte oder - 58223
	uint16_t	len;			// L : network byte oder
	uint8_t		dev_type;		// obu_modem:10, obu:11, rsu_modem:20, rsu:21, rsu_control:22
	uint8_t		tx_rx;			// tx:0, rx:1
	uint32_t	dev_id;
	uint16_t	hw_ver;
	uint16_t	sw_ver;
	uint64_t	timestamp;
	uint16_t	crc;			// C : network byte order
}__attribute__((__packed__)) TLVC_STATUS_CommUnit;

typedef struct _TLVC_STATUS_CommUnit_V2
{
	uint32_t	type;			// T : network byte oder - 58223
	uint16_t	len;			// L : network byte oder
	uint8_t		dev_type;		// obu_modem:10, obu:11, rsu_modem:20, rsu:21, rsu_control:22
	uint8_t		tx_rx;			// tx:0, rx:1
	uint32_t	dev_id;
	uint16_t	hw_ver;
	uint16_t	sw_ver;
	uint64_t	timestamp;
	char		cpu_temp;
	char		peri_temp;
	uint16_t	crc;			// C : network byte order
}__attribute__((__packed__)) TLVC_STATUS_CommUnit_V2;

typedef struct _TLVC_STATUS_ControlUnit
{
	uint32_t	type;			// T : network byte oder - 58223
	uint16_t	len;			// L : network byte oder
	uint8_t		dev_type;		// obu_modem:10, obu:11, rsu_modem:20, rsu:21, rsu_control:22
	uint8_t		tx_rx;			// tx:0, rx:1
	uint32_t	dev_id;
	uint16_t	hw_ver;
	uint16_t	sw_ver;
	uint64_t	timestamp;
	uint16_t	crc;			// C : network byte order
}__attribute__((__packed__)) TLVC_STATUS_ControlUnit;

typedef struct _TLVC_STATUS_ControlUnit_V2
{
	uint32_t	type;			// T : network byte oder - 58223
	uint16_t	len;			// L : network byte oder
	uint8_t		dev_type;		// obu_modem:10, obu:11, rsu_modem:20, rsu:21, rsu_control:22
	uint8_t		tx_rx;			// tx:0, rx:1
	uint32_t	dev_id;
	uint16_t	hw_ver;
	uint16_t	sw_ver;
	uint64_t	timestamp;
	char		cpu_temp;
	char		peri_temp;
	uint16_t	crc;			// C : network byte order
}__attribute__((__packed__)) TLVC_STATUS_ControlUnit_V2;

typedef struct _FtpConnInfoReq
{
	uint32_t	psid;
	uint8_t		unit_id;
	uint32_t	link_id;
//	uint16_t	crc;
}__attribute__((__packed__)) FtpConnInfoReq;

typedef struct _FtpConnInfoResp
{
	uint32_t	psid;
	uint8_t		unit_id;
	uint32_t	link_id;
	uint32_t	ip_addr;
	uint16_t	port;
	char		data[];		// ID, Password string 사이 NULL 데이터
	//uint16_t	crc;		// 마지막에 추가 필요
}__attribute__((__packed__)) FtpConnInfoResp;

#endif		// _5GNR_V2X_APP_EXT_H_

