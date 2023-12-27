
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
// Module: v2x_defs.h
// Description: For 5G NR V2X OBU GW
// Update History
// [2021/04/10 jioh] created
// [2023/05/10 jwlee] updated
// [2023/05/11 jsKim] updated
//=============================================================================

#ifndef _SIB_V2X_DEFS_H_
#define _SIB_V2X_DEFS_H_

#include <stdint.h>

#define V2X_BOOL uint8_t

#define WSM_PROTO 0x88DC
#define IPV6_PROTO 0x86DD
#define IPV4_PROTO 0x0800
#define ARP_PROTO 0x0806

/* Ports */
#define STACK_LIB_PORT_BASE 8500

#define WSM_MSG_H_PORT 8860
#define ITS_CLI_HAL_PORT 8870
#define ATLKS_CV2X_HAL_PORT 8890
#define ATLKS_DSRC_HAL_PORT 8900
#define NXP_DSRC_HAL_PORT 8910

#define MAX_SERVICE_ID_STR_LEN 32

#define MAC_EUI48_LEN 6
#define IPV6_MAX_LEN 16

#define MAX_INTERFACE_NAME_LEN 32
#define MAX_PSC_LEN 32
#define MAX_ADVERTISER_ID_LEN 32
#define MAX_COUNTRY_LEN 3

#define MAX_V2X_DATA_LEN 4096
#define MAX_V2X_FWD_DATA_LEN 2048

#define MAX_WAVE_TX_POWER 23
#define MIN_WAVE_TX_POWER 0

#define MAX_V2X_CH_CNT 7

#define V2X_OPERATING_CLASS 17

#define MAX_V2X_MSG_TYPE_CNT 50

#define MAGIC_WSREQ 0xf1f1
#define MAGIC_WSRESP 0xf1f2
#define MAGIC_DSRC_TX_PDU 0xf2f1
#define MAGIC_CV2X_TX_PDU 0xf2f2
#define MAGIC_DSRC_RX_PDU 0xf3f1
#define MAGIC_CV2X_RX_PDU 0xf3f2

/**
 * @brief
 *
 * @author jwlee
 * @date 2023-04-28
 */
typedef enum _eV2xChannelNum_t
{
	eV2xChannelNum_172 = 172, /* 5860 */
	eV2xChannelNum_174 = 174, /* 5870 */
	eV2xChannelNum_176 = 176, /* 5880 */
	eV2xChannelNum_178 = 178, /* 5890 */
	eV2xChannelNum_180 = 180, /* 5900 */
	eV2xChannelNum_182 = 182, /* 5910 */
	eV2xChannelNum_184 = 184, /* 5920 */
	eV2xChannelNum_Min = eV2xChannelNum_172,
	eV2xChannelNum_Max = eV2xChannelNum_184
} eV2xChannelNum_t;
typedef uint8_t V2xChannelNum_t;

/**
 * @name eV2xPsid_t
 * @brief V2X Msg PSID를 정의한다.
 *        원칙적으로는  SAE(https://standards.ieee.org/products-programs/regauth/psid/public/)에서 관리하는 PSID를 사용하는
 *        참조용으로 사용,
 * @author jwlee
 * @date 2023-04-28
 */
typedef enum _eV2xPsid_t
{
	//** J2735: The number of value is dependent to SAE J2735 ASN.1 spec.
	eV2XMSG_TYPE_NONE = 0,		   //    0x0000,
	eV2XMSG_TYPE_MSGFRAME = 17,	   // 17 is not a spec of ASN.1.
	eV2XMSG_TYPE_CSR = 21,		   // 0x0008,//21,
	eV2XMSG_TYPE_ICA = 23,		   // 0x0020,//23,
	eV2XMSG_TYPE_NMEA = 24,		   // 0x0040,//24,
	eV2XMSG_TYPE_PDM = 25,		   // 0x0080,//25,
	eV2XMSG_TYPE_SRM = 29,		   // 0x0800,//29,
	eV2XMSG_TYPE_SSM = 30,		   // 0x1000,//30,
	eV2XMSG_TYPE_PSM = 32,		   // 0x4000,//32,
	eV2XMSG_TYPE_JPVD = 101,	   // JPVD DSRCMsgID : 101  :SMTB 전용
	eV2XMSG_TYPE_SMTBAVInfo = 102, // SMTBAVInfo  DSRC 추가,  DSRCMsgID : 102  :SMTB 전용
	eV2XMSG_TYPE_PPD = 127,		   // 7f  test psid   ProbePlatooningData:SMTB 전용  SMTBAVIF

	/*for 5GNRV2X*/
	eV2XMSG_TYPE_PIM = 5081, // PSID_PerceptionInfoMsg                  81
	eV2XMSG_TYPE_DMM = 5082, // PSID_DrivingManeuverMsg                 82
	eV2XMSG_TYPE_DNM = 5083, // PSID_DrivingNegotiationMsg              83
	eV2XMSG_TYPE_EDM = 5084, // PSID_EmergencyDrivingMsg                84
	eV2XMSG_TYPE_KDF = 5271, // KETI Data frame을 위한  PSID -V2V
	eV2XMSG_TYPE_EDF = 5272, // KETI data frame을 위한  PSID - V2I

	eV2XMSG_TYPE_ETCS = 82049,
	eV2XMSG_TYPE_BSM = 82050,  // 0x0004,//20,
	eV2XMSG_TYPE_PVD = 82051,  // 0x0100,//26, 0x014083
	eV2XMSG_TYPE_EVA = 82052,  // 0x0010,//22,
	eV2XMSG_TYPE_RSA = 82053,  // 0x0200,//27,
	eV2XMSG_TYPE_TIM = 82054,  // 0x2000,//31,
	eV2XMSG_TYPE_SPAT = 82055, // 0x0002,//19,
	eV2XMSG_TYPE_MAP = 82056,  // 0x0001,//18,		// DSRCmsgID_mapData for J2735
	eV2XMSG_TYPE_RTCM = 82057, // 0x0400,//28,
	eV2XMSG_TYPE_ETC = 90000,
	eV2XMSG_TYPE_MAX
} eV2xPsid_t;

typedef uint32_t V2xPsid_t;

// rcpi threshold (0..255)
typedef enum _eV2xRcpi_t
{
	eRCPI_Min = 0,
	eRcpi_Max = 255,
	eRcpi_Unknown = -99
} eV2xRcpi_t;
typedef int16_t V2xRcpi_t;

// rssi threshold (0..255)
typedef enum _eV2xRSSI_t
{
	eRSSI_Min = -128,
	eRSSI_Max = 0, // 확인 필요
} eV2xRSSI_t;
typedef int16_t V2xRSSI_t;

/**
 * @name V2XPayloadType_t
 * @brief V2X MSG(Data(payload))에 실리는 Data type 종류
 *
 * @author jwlee
 * @date 2023-04-28
 */
typedef enum _eV2xPayloadType_t
{
	/*
	eDot11 = 1,		 // -- 802.11 MPDU (802.11 header included)
	eApplication = 2 //-- Application PDU type (encoded j2735)
	*/
	eRaw = 0,			 // 정의되지 않은 RAW data
	eEncodedbyJ2735 = 1, // J2735 Msg가 ASN.1 encoding 된 data
	eITSK00130 = 2,		 // ITSK-00130 으로 formating 된 data /*Unsupport*/
	eKETI = 3,			 // KETI 가 5GNR 과제를 위해 정의한 data frame
	eETRI = 4			 // ETRI 가 5GNR 과제를 위해 정의한 data frame

} eV2xPayloadType_t;
typedef uint8_t V2xPayloadType_t;

////////////////////////////////////////////////////////////////////

typedef enum _eV2xCv2xMode_t
{
	eV2xCv2xMode_SPS,
	eV2xCv2xMode_ADHOC,
	eV2xCv2xMode_Cnt,
	eV2xCv2xMode_Max = eV2xCv2xMode_Cnt - 1
} eV2xCv2xMode_t;

typedef enum _eV2xRadio_t
{
	eV2xRadio_A = 0,
	eV2xRadio_B,
	eV2xRadio_Cnt,
	eV2xRadio_Max = eV2xRadio_Cnt - 1,
	eV2xRadio_DsrcCnt = eV2xRadio_Cnt,
	eV2xRadio_Cv2xCnt = 1
} eV2xRadio_t;
typedef uint8_t V2xRadio_t;

typedef enum _eV2xChannelMode_t
{
	eV2xChannelMode_Continuous = 0,
	eV2xChannelMode_Alternating,
	eV2xChannelMode_Max
} eV2xChannelMode_t;
typedef uint8_t V2xChannelMode_t;

/* 1609.3 - 7.3.2.2 */
typedef enum _eV2xTimeSlot_t
{
	eV2xTimeSlot_0 = 0,
	eV2xTimeSlot_1,
	eV2xTimeSlot_Continuous,
	eV2xTimeSlot_Cnt,
	eV2xTimeSlot_Max = eV2xTimeSlot_Cnt - 1,
	eV2xTimeSlot_SlotCnt = 2
} eV2xTimeSlot_t;
typedef uint8_t V2xTimeSlot_t;

typedef enum _eV2xPsidType_t
{
	eV2xPsidType_Normal = 0,
	eV2xPsidType_P_Encoded
} eV2xPsidType_t;
typedef uint8_t V2xPsidType_t;

typedef enum _eV2xDataRate_t
{
	eV2xDataRate_NA = 0,
	eV2xDataRate_3MBPS = 0x06,
	eV2xDataRate_4_5MBPS = 0x09,
	eV2xDataRate_6MBPS = 0x0C,
	eV2xDataRate_9MBPS = 0x12,
	eV2xDataRate_12MBPS = 0x18,
	eV2xDataRate_18MBPS = 0x24,
	eV2xDataRate_24MBPS = 0x30,
	eV2xDataRate_27MBPS = 0x36
} eV2xDataRate_t;
typedef uint16_t V2xDataRate_t;

/**
 * @brief DSRC 0~23 (국내 기준 20)
 *        C-V2X -30~23
 *
 * @author jwlee
 * @date 2023-04-28
 */
typedef int8_t V2xPowerDbm_t;

/**
 * @brief V2xTime_t expiry_time
 *        Tx 최대 대기 시간, default 0
 *
 * @author jwlee
 * @date 2023-04-28
 */
typedef uint64_t V2xTime_t;

/* Message TX/RX priority
 * CV2X Socket Priority for SP socket policy
 * DSRC User message priority
 */
typedef enum _eV2xMsgPriority_t
{
	eV2xPriority_CV2X_PPPP_MIN = 0, /* CV2X */
	eV2xPriority_CV2X_PPPP_0 = 0,
	eV2xPriority_CV2X_PPPP_1 = 1,
	eV2xPriority_CV2X_PPPP_2 = 2,
	eV2xPriority_CV2X_PPPP_3 = 3,
	eV2xPriority_CV2X_PPPP_4 = 4,
	eV2xPriority_CV2X_PPPP_5 = 5,
	eV2xPriority_CV2X_PPPP_6 = 6,
	eV2xPriority_CV2X_PPPP_7 = 7,
	eV2xPriority_CV2X_PPPP_MAX = 7,
	eV2xPriority_CV2X_PPPP_INVALID = 8,
	eV2xPriority_DSRC_USER_PRIO_MIN = 0, /* DSRC */
	eV2xPriority_DSRC_USER_PRIO_00 = 0,
	eV2xPriority_DSRC_USER_PRIO_01 = 1,
	eV2xPriority_DSRC_USER_PRIO_02 = 2,
	eV2xPriority_DSRC_USER_PRIO_03 = 3,
	eV2xPriority_DSRC_USER_PRIO_04 = 4,
	eV2xPriority_DSRC_USER_PRIO_05 = 5,
	eV2xPriority_DSRC_USER_PRIO_06 = 6,
	eV2xPriority_DSRC_USER_PRIO_07 = 7,
	eV2xPriority_DSRC_USER_PRIO_MAX = 7,
	eV2xPriority_DSRC_USER_PRIO_INVALID = 8
} eV2xMsgPriority_t;
typedef uint8_t V2xMsgPriority_t;

/**
 * @name
 * @brief 1609.2 의 signerID, default 는 eV2xSignerId_ALTERNATE, 사용하지 않는 경우 eV2xSignerId_UNSECURED
 *
 * @author jwlee
 * @date 2023-04-28
 */
typedef enum _eV2xSignerId_t
{
	eV2xSignerId_UNSECURED = 0,
	eV2xSignerId_CERTIFICATE,
	eV2xSignerId_DIGEST,
	eV2xSignerId_ALTERNATE,
	eV2xSignerId_Cnt,
	eV2xSignerId_Max = eV2xSignerId_Cnt - 1
} eV2xSignerId_t;
typedef uint8_t V2xSignerId_t;

/**
 * @name eV2xAction_t
 * @brief V2X WME MIB에 profile/csr(Channel service request)/
 *        wsr(WSM service request)/psr(Provider service request) 서비스 요청을
 *        add/del 등  action 정보를 나타낸다.
 *
 * @author jwlee
 * @date 2023-04-28
 */
typedef enum _eV2xAction_t
{
	eV2xAction_ADD = 0,
	eV2xAction_DELETE = 1,
	eV2xAction_ResetAll = 3, /*Unsupport*/
} eV2xAction_t;
typedef uint8_t V2xAction_t;

typedef enum _eV2xUserRequestType_t
{
	eV2xUserRequestType_Auto_Access = 0,
	eV2xUserRequestType_No_Sch,
	eV2xUserRequestType_Cnt,
	eV2xUserRequestType_Max = eV2xUserRequestType_Cnt - 1
} eV2xUserRequestType_t;
typedef uint8_t V2xUserRequestType_t;

typedef enum _eV2xWsaType_t
{
	eV2xWsaType_Secured = 0,
	eV2xWsaType_Unsecured,
	eV2xWsaType_Secured_Or_Unsecured, // Only OBU use
	eV2xWsaType_Any					  // Only OBU use
} eV2xWsaType_t;
typedef uint8_t V2xWsaType_t;

typedef enum _eV2xAntenna_t
{
	eV2xAntenna_Invalid = 0U,
	eV2xAntenna_1 = 1U,
	eV2xAntenna_2 = 2U,
	//	eV2xAntenna_3		= 4U,
	//	eV2xAntenna_4		= 8U,
	eV2xAntenna_1And2 = eV2xAntenna_1 | eV2xAntenna_2
} eV2xAntenna_t;
typedef uint8_t V2xAntenna_t;

/**
 * @name eV2XCommType_t
 * @brief
 *
 * @author jwlee
 * @date 2023-04-28
 */
typedef enum _eV2XCommType_t
{
	eV2XCommType_ON_CONFIG = 0, /**사전 설정정보에 따라서*/
	eV2XCommType_DSRC,
	eV2XCommType_LTEV2X,
	eV2XCommType_5GNRV2X,
	eV2XCommType_Cnt,
	eV2XCommType_Max = eV2XCommType_Cnt - 1,
	eV2XCommType_Start = eV2XCommType_ON_CONFIG
} eV2XCommType_t;
typedef uint8_t V2XCommType_t;

typedef enum _eV2xFrequency_t
{
	eV2xFrequency_5860 = 5860, /* 172 ch */
	eV2xFrequency_5870 = 5870, /* 174 ch */
	eV2xFrequency_5880 = 5880, /* 176 ch */
	eV2xFrequency_5890 = 5890, /* 178 ch */
	eV2xFrequency_5900 = 5900, /* 180 ch */
	eV2xFrequency_5910 = 5910, /* 182 ch */
	eV2xFrequency_5920 = 5920, /* 184 ch */
	eV2xFrequency_Min = eV2xFrequency_5860,
	eV2xFrequency_Max = eV2xFrequency_5920
} eV2xFrequency_t;
typedef int16_t V2xFrequency_t;

typedef enum _eV2xStatusCmdCode_t
{
	eV2xStatusCmdCode_Invalid = 0,
	eV2xStatusCmdCode_CertificateInfo,
	eV2xStatusCmdCode_MemoryPoolInfo,
	eV2xStatusCmdCode_MemoryPoolLeakInfo,
	eV2xStatusCmdCode_Cnt,
	eV2xStatusCmdCode_Max = eV2xStatusCmdCode_Cnt - 1
} eV2xStatusCmdCode_t;
typedef uint8_t V2xStatusCmdCode_t;

typedef enum _eCv2xFlowType_t
{
	eCv2xFlowType_NoValue = 0,
	eCv2xFlowType_Sps,
	eCv2xFlowType_Adhoc
} eCv2xFlowType_t;
typedef uint8_t Cv2xFlowType_t;

typedef struct _PsidConfig_t
{
	uint32_t service_id;
	char service_id_str[MAX_SERVICE_ID_STR_LEN];
	V2xPsid_t psid;
} PsidConfig_t;

/* 1609.3 - 8.3.4 */
typedef struct _V2xInfoElementsIndicator_t
{
	uint8_t reserved : 5;
	uint8_t channel_id : 1;
	uint8_t data_rate : 1;
	uint8_t tx_pwr_level : 1;
}
__attribute__((__packed__)) V2xInfoElementsIndicator_t;

typedef struct _V2xMacEui48_t
{
	uint8_t octets[MAC_EUI48_LEN];
}
__attribute__((__packed__)) V2xMacEui48_t;

typedef struct _V2xIPv6Address_t
{
	uint8_t octets[IPV6_MAX_LEN];
}
__attribute__((__packed__)) V2xIPv6Address_t;

typedef uint32_t V2xCertTmpId;

typedef uint16_t V2xPacketLength_t;

#endif /* _SIB_V2X_DEFS_H_ */