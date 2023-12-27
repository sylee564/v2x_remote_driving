#ifndef	_DB_V2X_H_
#define	_DB_V2X_H_

/******************************************************************************
*
* Copyright (C) 2023 - 2028 KETI, All rights reserved.
*                           (Korea Electronics Technology Institute)
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running for Korean Government Project, or
* (b) that interact with KETI project/platform.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* KETI BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the KETI shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from KETI.
*
******************************************************************************/
/******************************************************************************/
/**
*
* @file db_v2x.h
*
* This file contains a data format design
*
* @note
*
* V2X Data Format Header
*
* MODIFICATION HISTORY:
* Ver   Who  Date     Changes
* ----- ---- -------- ----------------------------------------------------
* 1  bman  23.03.22 First release
*
******************************************************************************/


/***************************** Include ***************************************/
#include <stdint.h>

/***************************** Definition ************************************/
# define DB_V2X_VERSION                 = 1

/***************************** Enum and Structure ****************************/
typedef enum {
    DB_V2X_DEVICE_TYPE_UNKNOWN          = 0x0000,
    DB_V2X_DEVICE_TYPE_OBU              = 0x0001,
    DB_V2X_DEVICE_TYPE_RSU              = 0x0002,
    DB_V2X_DEVICE_TYPE_UNDEFINED_0,
    DB_V2X_DEVICE_TYPE_UNDEFINED_1,
    DB_V2X_DEVICE_TYPE_UNDEFINED_2,
    DB_V2X_DEVICE_TYPE_UNDEFINED_3,
    DB_V2X_DEVICE_TYPE_MAX              = 0xFFFF
} DB_V2X_DEVICE_TYPE_E;
typedef uint16_t DB_V2X_DEVICE_TYPE_t;


typedef enum {
    DB_V2X_TELECOMM_TYPE_UNKNOWN        = 0x0000,
    DB_V2X_TELECOMM_TYPE_4G             = 0x0001,   /* LTE */
    DB_V2X_TELECOMM_TYPE_5G_PC5         = 0x0002,
    DB_V2X_TELECOMM_TYPE_5G_UU          = 0x0003,
    DB_V2X_TELECOMM_TYPE_6G             = 0x0004,
    DB_V2X_TELECOMM_TYPE_UNDEFINED_0,
    DB_V2X_TELECOMM_TYPE_UNDEFINED_1,
    DB_V2X_TELECOMM_TYPE_UNDEFINED_2,
    DB_V2X_TELECOMM_TYPE_MAX            = 0xFFFF
} DB_V2X_TELECOMMUNICATION_TYPE_E;
typedef uint16_t DB_V2X_TELECOMMUNICATION_TYPE_t;


typedef enum {
    DB_V2X_SERVICE_ID_UNKNOWN           = 0x0000,
    DB_V2X_SERVICE_ID_PLATOONING        = 0x0001,
    DB_V2X_SERVICE_ID_SENSOR_SHARING    = 0x0002,
    DB_V2X_SERVICE_ID_REMOTE_DRIVING    = 0x0003,
    DB_V2X_SERVICE_ID_ADVANCED_DRIVING  = 0x0004,
    DB_V2X_SERVICE_ID_UNDEFINED_0       = 0x0005,
    DB_V2X_SERVICE_ID_UNDEFINED_1       = 0x0006,
    DB_V2X_SERVICE_ID_MAX               = 0xFFFF
} DB_V2X_SERVICE_ID_E;
typedef uint16_t DB_V2X_SERVICE_ID_t;

typedef enum {
    DB_V2X_ACTION_TYPE_UNKNOWN          = 0x0000,
    DB_V2X_ACTION_TYPE_REQUEST          = 0x0001,
    DB_V2X_ACTION_TYPE_RESPONSE         = 0x0002,
    DB_V2X_ACTION_TYPE_MAX              = 0xFFFF
} DB_V2X_ACTION_TYPE_E;
typedef uint16_t DB_V2X_ACTION_TYPE_t;

typedef enum {
    DB_V2X_REGION_ID_UNKNOWN            = 0x0000,
    DB_V2X_REGION_ID_SEOUL              = 0x0001,
    DB_V2X_REGION_ID_SEJONG             = 0x0002,
    DB_V2X_REGION_ID_BUSAN              = 0x0003,
    DB_V2X_REGION_ID_DAEGEON            = 0x0004,
    DB_V2X_REGION_ID_INCHEON            = 0x0005,
    DB_V2X_REGION_ID_DAEGU              = 0x0006,
    DB_V2X_REGION_ID_DAEGU_KIAPI_PG     = 0x0007,
    DB_V2X_REGION_ID_CHEONGJU           = 0x0008,
    DB_V2X_REGION_ID_SEONGNAM           = 0x0009,
    DB_V2X_REGION_ID_UNDEFINED_0,
    DB_V2X_REGION_ID_UNDEFINED_1,
    DB_V2X_REGION_ID_UNDEFINED_2,
    DB_V2X_REGION_ID_MAX                = 0xFFFF
} DB_V2X_REGION_ID_E;
typedef uint16_t DB_V2X_REGION_ID_t;


typedef enum {
    DB_V2X_PAYLOAD_TYPE_UNKNOWN                 = 0x0000,
    DB_V2X_PAYLOAD_TYPE_SAE_J2735_BSM           = 0x0001,
    DB_V2X_PAYLOAD_TYPE_SAE_J2735_PVD           = 0x0002,
    DB_V2X_PAYLOAD_TYPE_SAE_J2735_UNDEFINED_0   = 0x0003,
    DB_V2X_PAYLOAD_TYPE_SAE_J2735_UNDEFINED_1   = 0x0004,
    DB_V2X_PAYLOAD_TYPE_SAE_J2735_UNDEFINED_2   = 0x0005,
    DB_V2X_PAYLOAD_TYPE_STANDARD_UNDEFINED_0    = 0x0006,
    DB_V2X_PAYLOAD_TYPE_STANDARD_UNDEFINED_1    = 0x0007,
    DB_V2X_PAYLOAD_TYPE_STANDARD_UNDEFINED_2    = 0x0008,
    DB_V2X_PAYLOAD_TYPE_STANDARD_UNDEFINED_3    = 0x0009,
    DB_V2X_PAYLOAD_TYPE_STANDARD_UNDEFINED_4    = 0x0009,
    DB_V2X_PAYLOAD_TYPE_PLATOONING              = 0x000A,
    DB_V2X_PAYLOAD_TYPE_SENSOR_SHARING          = 0x000B,
    DB_V2X_PAYLOAD_TYPE_REMOTE_DRIVING          = 0x000C,
    DB_V2X_PAYLOAD_TYPE_ADVANCED_DRIVING        = 0x000D,
    DB_V2X_PAYLOAD_TYPE_UNDEFINED_0             = 0x000F,
    DB_V2X_PAYLOAD_TYPE_UNDEFINED_1             = 0x0010,
    DB_V2X_PAYLOAD_TYPE_UNDEFINED_2             = 0x0011,
    DB_V2X_PAYLOAD_TYPE_UNDEFINED_3             = 0x0012,
    DB_V2X_PAYLOAD_TYPE_UNDEFINED_4             = 0x0013,
    DB_V2X_PAYLOAD_TYPE_MAX                     = 0xFFFF
} DB_V2X_PAYLOAD_TYPE_E;
typedef uint16_t DB_V2X_PAYLOAD_TYPE_t;

typedef enum {
    DB_V2X_COMM_ID_UNKNOWN              = 0x0000,
    DB_V2X_COMM_ID_V2V                  = 0x0001,
    DB_V2X_COMM_ID_V2I                  = 0x0002,
    DB_V2X_COMM_ID_V2P                  = 0x0003,
    DB_V2X_COMM_ID_V2N                  = 0x0004,
    DB_V2X_COMM_ID_I2V                  = 0x0005,
    DB_V2X_COMM_ID_I2P                  = 0x0006,
    DB_V2X_COMM_ID_I2N                  = 0x0007,
    DB_V2X_COMM_ID_P2V                  = 0x0008,
    DB_V2X_COMM_ID_P2I                  = 0x0009,
    DB_V2X_COMM_ID_P2N                  = 0x000A,
    DB_V2X_COMM_ID_N2V                  = 0x000B,
    DB_V2X_COMM_ID_N2I                  = 0x000C,
    DB_V2X_COMM_ID_N2P                  = 0x000D,
    DB_V2X_COMM_ID_UNDEFINED_0          = 0x000F,
    DB_V2X_COMM_ID_MAX                  = 0xFFFF
} DB_V2X_COMMUNCATION_ID_E;
typedef uint16_t DB_V2X_COMMUNCATION_ID_t;

typedef struct DB_V2X {
    DB_V2X_DEVICE_TYPE_t                eDeviceType;
    DB_V2X_TELECOMMUNICATION_TYPE_t     eTeleCommType;
    uint32_t                            unDeviceId;
    uint64_t                            ulTimeStamp;
    DB_V2X_SERVICE_ID_t                 eServiceId;
    DB_V2X_ACTION_TYPE_t                eActionType;
    DB_V2X_REGION_ID_t                  eRegionId;
    DB_V2X_PAYLOAD_TYPE_t               ePayloadType;
    DB_V2X_COMMUNCATION_ID_t            eCommId;
    uint16_t                            usDbVer;
    uint16_t                            usHwVer;
    uint16_t                            usSwVer;
    uint32_t                            ulPayloadLength;
    uint32_t                            ulPayloadCrc32;
    uint8_t                             payload[];
} __attribute__((__packed__))DB_V2X_T;

/***************************** Function Protype ******************************/


#endif	/* _DB_V2X_H_ */
