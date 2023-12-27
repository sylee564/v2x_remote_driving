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
// Module: v2x_ext_type.h
// Description: For 5G NR V2X OBU GW
// Update History
// [2023/05/10 jwlee] create
// [2023/05/11 jsKim] updated
//=============================================================================
#pragma once
#ifndef _SIB_EXT_V2X_TYPE_H_
#define _SIB_EXT_V2X_TYPE_H_

#include <stdint.h>
#include "v2x_defs.h"

typedef struct _Ext_WSReq_t
{
	uint16_t magic_num; /*0xF1F1*/
	uint16_t ver;
	V2xAction_t e_action;
	V2xPayloadType_t e_payload_type; // Support
	uint8_t reserved1[2];
	V2xPsid_t psid;
	uint8_t reserved2[4];
}
__attribute__((__packed__)) Ext_WSReq_t;

typedef struct _Ext_WSResp_t
{
	uint16_t magic_num; /*0xF1F2*/
	uint16_t ver;
	V2xAction_t e_action;
	V2X_BOOL is_confirmed; // Support
	uint8_t reserved1[2];
	V2xPsid_t psid;
	uint8_t reserved2[4];
}
__attribute__((__packed__)) Ext_WSResp_t;

typedef struct _Ext_V2xMsg_t
{
	V2xPacketLength_t length;
	uint8_t data[];
}
__attribute__((__packed__)) Ext_V2xMsg_t;

typedef struct _WAVE_Tx_Struct
{
	/*WAVE*/
	V2xFrequency_t freq;
	V2xDataRate_t e_data_rate;
	V2xTimeSlot_t e_time_slot;

	uint8_t peer_mac_addr[MAC_EUI48_LEN]; /* if manual_flg is true,
											 ignore */
} WAVE_Tx_Struct;

typedef struct _CV2X_Tx_Struct
{
	/*CV2X*/
	uint32_t transmitter_profile_id; /* Unsupport  RRC ?„¤? • profile id ,  */
	uint32_t peer_l2id;
} CV2X_Tx_Struct;

typedef struct _Ext_V2X_TxPDU_t
{
	/*Common field*/
	uint16_t magic_num; /* DSRC : 0xF2F1, CV2X : 0xF2F2 */
	uint16_t ver;
	V2xPsid_t psid;
	V2XCommType_t e_v2x_comm_type;
	V2xPayloadType_t e_payload_type;			   // Support
	V2xInfoElementsIndicator_t elements_indicator; /* if manual_flg is true, ignore, default 1,1,1ë¡? ?„¤? •?•œ?‹¤. */
	V2xPowerDbm_t tx_power;
	V2xSignerId_t e_signer_id;
	V2xMsgPriority_t e_priority;
	uint8_t channel_load;
	uint8_t reserved;
	V2xTime_t expiry_time;

	union
	{
		WAVE_Tx_Struct config_wave; // for DSRC
		CV2X_Tx_Struct config_cv2x; // for CV2X
	} __attribute__((__packed__)) u;

	uint8_t reserved2;
	uint8_t reserved3[8];
	uint32_t crc;
	Ext_V2xMsg_t v2x_msg;
}
__attribute__((__packed__)) Ext_V2X_TxPDU_t;

typedef struct _Ext_V2X_RxPDU_t
{
	uint16_t magic_num; /* DSRC : 0xF3F1, CV2X : 0xF3F2*/
	uint16_t ver;
	V2xPsid_t psid;
	V2XCommType_t e_v2x_comm_type;
	V2xPayloadType_t e_payload_type; // Support
	V2xFrequency_t freq;
	V2xRSSI_t rssi;
	V2X_BOOL is_signed;
	uint8_t reserved1;

	union
	{
		uint8_t peer_mac_addr[MAC_EUI48_LEN]; // for DSRC
		uint32_t peer_l2id;					  // for CV2X
	} u;

	uint8_t reserved2[2];
	uint8_t reserved3[8];
	uint32_t crc;
	Ext_V2xMsg_t v2x_msg;
}
__attribute__((__packed__)) Ext_V2X_RxPDU_t;
#endif
