#ifndef	_PVD_V2X_H_
#define	_PVD_V2X_H_

/******************************************************************************
*
* Copyright (C) 2023 - 2028 AVG, All rights reserved.
*                           (AVGenius.co.ltd)
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
* (b) that interact with AVG project/platform.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* AVG BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the AVG shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from AVG.
*
******************************************************************************/
/******************************************************************************/
/**
*
* @file v2x_pvd.h
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
* 1  sylee  23.08.22 First release
*
******************************************************************************/


/***************************** Include ***************************************/
#include <stdint.h>


/***************************** Enum and Structure ****************************/

typedef struct _Vehicle_Ident
{
	char vehicle_name[8];
    char vehicle_id[18];
}
__attribute__((__packed__)) Vehicle_Ident_t;


typedef enum {
    NONE                            = 0x0000,   // Not Equipped, Not known or unavailable
    UNKNOWN                         = 0x0001,   // Does not fill any other category
    SPECIAL                         = 0x0002,   // Special use
    MOTO                            = 0x0003,   // Motocycle
    CAR                             = 0x0004,   // Passenger car
    CAR_OTHER                       = 0x0005,   // Four tire single units
    BUS                             = 0x0006,   // Buses
    AXLE_CNT_2                      = 0x0007,   // Two axle, six tire single units
    AXLE_CNT_3                      = 0x0008,   // Three axle, single units
    AXLE_CNT_4                      = 0x0009,   // Four or more axle, single unit
    AXLE_CNT_4_TRAILER              = 0x000a,   // Four or less axle, single trailer
    AXLE_CNT_5_TRAILER              = 0x000b,   // Five or less axle, single trailer
    AXLE_CNT_6_TRAILER              = 0x000c,   // Six or more axle, single trailer
    AXLE_CNT_5_MULTI_TRAILER        = 0x000d,   // Five or less axle, multi-trailer
    AXLE_CNT_6_MULTI_TRAILER        = 0x000e,   // Six axle, multi-trailer
    AXLE_CNT_7_MULTI_TRAILER        = 0x000f,   // Seven or more axle, multi-trailer
} Vehicle_Type_E;
typedef uint16_t Vehicle_Type_t;

typedef struct _Full_Position_Vector
{
	double longitude;
    double latitude;
    float elevationd;
    float heading;
}__attribute__((__packed__)) Full_Position_Vector_t;

typedef struct _Vehicle_Status
{
	uint8_t gear_status;
    uint8_t speed;
    int steering_wheel;
    float acceleration;
}__attribute__((__packed__)) Vehicle_Status_t;

typedef struct _Control_status
{
	uint8_t lat_approved;
    uint8_t long_approved;
    uint8_t remote_status;
}__attribute__((__packed__)) Control_status_t;


typedef struct V2X_PVD {
    Vehicle_Ident_t                 vehicle_ident;
    Vehicle_Type_t                  vehicle_type;
    Full_Position_Vector_t          vehicle_pos;
    Vehicle_Status_t                vehicle_status;
    Control_status_t                control_status;
}  __attribute__((__packed__))V2X_PVD_T;

/***************************** Function Protype ******************************/


#endif	/* _V2x_PVD_H_ */
