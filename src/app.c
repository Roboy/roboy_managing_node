/**
********************************************************************************
\file   app.c

\brief  Demo MN application which implements a running light

This file contains a demo application for digital input/output data.
It implements a running light on the digital outputs. The speed of
the running light is controlled by the read digital inputs.

\ingroup module_demo_mn_console
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronik GmbH
Copyright (c) 2013, Kalycito Infotech Private Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "app.h"

#include <oplk/oplk.h>
#include <oplk/debugstr.h>

#include <obdpi.h>
#include <eventlog/eventlog.h>

#include "xap.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DEFAULT_MAX_CYCLE_COUNT 20      // 6 is very fast
#define APP_LED_COUNT_1         8       // number of LEDs for CN1
#define APP_LED_MASK_1          (1 << (APP_LED_COUNT_1 - 1))
#define MAX_NODES               255

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    // managing node output/ controlled node input
    signed CN1_MotorCommand_setPoint_I32_1:32;
    signed CN1_MotorCommand_setPoint_I32_2:32;
    signed CN1_MotorCommand_setPoint_I32_3:32;
    signed CN1_MotorCommand_setPoint_I32_4:32;
    signed CN1_MotorCommand_setPoint_I32_5:32;
    signed CN1_MotorCommand_setPoint_I32_6:32;
    signed CN1_MotorCommand_setPoint_I32_7:32;
    signed CN1_MotorCommand_setPoint_I32_8:32;
    unsigned CN1_MotorSelecta_motor_U8:8;
    // controlled node output/ managing node input
    signed pwmRef_I16:16;
    signed actualPosition_I32:32;
    signed actualVelocity_I16:16;
    signed actualCurrent_I16:16;
    signed springDisplacement_I16:16;
    signed sensor1_I16:16;
    signed sensor2_I16:16;
} APP_NODE_VAR_T;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static UINT             cnt_l;
static APP_NODE_VAR_T   aNodeVar_l[MAX_NODES];
static PI_IN*           pProcessImageIn_l;
static const PI_OUT*    pProcessImageOut_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError       initProcessImage(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize the synchronous data application

The function initializes the synchronous data application

\return The function returns a tOplkError error code.

\ingroup module_demo_mn_console
*/
//------------------------------------------------------------------------------
tOplkError initApp(void)
{
    tOplkError  ret = kErrorOk;
    cnt_l = 0;

    ret = initProcessImage();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown the synchronous data application

The function shuts down the synchronous data application

\return The function returns a tOplkError error code.

\ingroup module_demo_mn_console
*/
//------------------------------------------------------------------------------
void shutdownApp(void)
{
    tOplkError  ret;

    ret = oplk_freeProcessImage();
    if (ret != kErrorOk)
    {
        fprintf(stderr,
                "oplk_freeProcessImage() failed with \"%s\" (0x%04x)\n",
                debugstr_getRetValStr(ret),
                ret);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Synchronous data handler

The function implements the synchronous data handler.

\return The function returns a tOplkError error code.

\ingroup module_demo_mn_console
*/
//------------------------------------------------------------------------------
tOplkError processSync(void)
{
    tOplkError  ret;

    ret = oplk_waitSyncEvent(100000);
    if (ret != kErrorOk)
        return ret;

    ret = oplk_exchangeProcessImageOut();
    if (ret != kErrorOk)
        return ret;

    cnt_l++;

    aNodeVar_l[0].pwmRef_I16 = pProcessImageOut_l->CN1_MotorStatus_pwmRef_I16;
    aNodeVar_l[0].actualPosition_I32 = pProcessImageOut_l->CN1_MotorStatus_actualPosition_I32;
    aNodeVar_l[0].actualVelocity_I16 = pProcessImageOut_l->CN1_MotorStatus_actualVelocity_I16;
    aNodeVar_l[0].actualCurrent_I16 = pProcessImageOut_l->CN1_MotorStatus_actualCurrent_I16;
    aNodeVar_l[0].springDisplacement_I16 = pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16;
    aNodeVar_l[0].sensor1_I16 = pProcessImageOut_l->CN1_MotorStatus_sensor1_I16;
    aNodeVar_l[0].sensor2_I16 = pProcessImageOut_l->CN1_MotorStatus_sensor2_I16;

    static int iter = 1;
    if((iter++)%300==0) {
        printf("\n############## myoFPGA ###################\n");
        printf("pwmRef:             %d\n", aNodeVar_l[0].pwmRef_I16);
        printf("actualPosition:     %d\n", aNodeVar_l[0].actualPosition_I32);
        printf("actualVelocity:     %d\n", aNodeVar_l[0].actualVelocity_I16);
        printf("actualCurrent:      %d\n", aNodeVar_l[0].actualCurrent_I16);
        printf("springDisplacement: %d\n", aNodeVar_l[0].springDisplacement_I16);
    }
    // setpoints for 8 motors
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_1 = 0;
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_2 = 0;
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_3 = 0;
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_4 = 0;
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_5 = 0;
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_6 = 0;
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_7 = 0;
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_8 = 0;
    // select from which motor you want to receive motorStatus
    pProcessImageIn_l->CN1_MotorSelecta_motor_U8 = 0;

    ret = oplk_exchangeProcessImageIn();

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize process image

The function initializes the process image of the application.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initProcessImage(void)
{
    tOplkError  ret = kErrorOk;
    UINT        errorIndex = 0;

    printf("Initializing process image...\n");
    printf("Size of process image: Input = %lu Output = %lu\n",
           (ULONG)sizeof(PI_IN),
           (ULONG)sizeof(PI_OUT));
    eventlog_printMessage(kEventlogLevelInfo,
                          kEventlogCategoryGeneric,
                          "Allocating process image: Input:%lu Output:%lu",
                          (ULONG)sizeof(PI_IN),
                          (ULONG)sizeof(PI_OUT));

    ret = oplk_allocProcessImage(sizeof(PI_IN), sizeof(PI_OUT));
    if (ret != kErrorOk)
        return ret;

    pProcessImageIn_l = (PI_IN*)oplk_getProcessImageIn();
    pProcessImageOut_l = (const PI_OUT*)oplk_getProcessImageOut();

    errorIndex = obdpi_setupProcessImage();
    if (errorIndex != 0)
    {
        eventlog_printMessage(kEventlogLevelFatal,
                              kEventlogCategoryControl,
                              "Setup process image failed at index 0x%04x\n",
                              errorIndex);
        ret = kErrorApiPINotAllocated;
    }

    return ret;
}

/// \}