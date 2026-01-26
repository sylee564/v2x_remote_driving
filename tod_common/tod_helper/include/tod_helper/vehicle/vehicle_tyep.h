#ifndef	_VEHICLE_TYPE_H_
#define	_VEHICLE_TYPE_H_

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
* @file vehicle_type.h
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

/***************************** Function Protype ******************************/


#endif	/* _VEHICLE_TYPE_H_ */
