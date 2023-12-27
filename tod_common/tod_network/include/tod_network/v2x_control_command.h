#ifndef	_CCD_V2X_H_
#define	_CCD_V2X_H_

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
* @file v2x_control_command.h
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

typedef struct _Probe_ID
{
	uint32_t operator_id;
    uint8_t control_type;
}
__attribute__((__packed__)) Probe_ID_t;

typedef struct _Start_Signals
{
	uint8_t remote_signal;  //0:off, 1:on
	uint8_t stream_signal;  //0:off, 1:on
}__attribute__((__packed__)) Start_Signals_t;

typedef struct _Control_Command
{
	double steering_wheel;  //rad
    uint8_t gear;
    float velocity;         // m/s
    float acceleration;     // m/s^2
    uint8_t indicator;      //0:off, 1:emergency, left:2, right:4
}__attribute__((__packed__)) Control_Command_t;


typedef struct V2X_CCD {
    Probe_ID_t                       operator_probe;
    Start_Signals_t                  operator_signal;
    Control_Command_t                control_command;
}  __attribute__((__packed__))V2X_CCD_T;

/***************************** Function Protype ******************************/


#endif	/* _V2x_CCD_H_ */
