/*
 * Copyright (c) 2023, Kenneth Lindsay
 * All rights reserved.
 * Author: Kenneth Lindsay
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *      following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *      following disclaimer in the documentation and/or other materials provided with the distribution.
 *   3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *      products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/******************************************************************************************************************
*            BEAGLEBONE AI64                                 -------------
*         -------------------                                |           |
*         |                 |                                |  Putty    |
*         |        ETHERNET |-----<  <----------------->  >--|  (SSH)    |
*         |                 |                                |           |
*         |                 |                                -------------                  -------------------
*         |                 |                                -------------------            |                 |
*         |           5.0V  |---> P9-3 <----------> J1-4  >--| 3.3V            |--> P1-4 >--| 4 Blu           |
*         |                 |                                |                 |            |                 |
*         |             GND |---> P9-2 >----------> J1-10 >--|                 |--> P1-3 >--| 3 Grn GND       |
*         |                 |                                |                 |            |                 |
*         |  eqep1_b  P8_33 |---> P8-33 <---------> J1-9  <--|------------*----|--> P1-5 >--| 5 Yel Encoder A |
*         |                 |                                |            |    |            |                 |
*         |  eqep1_a  P8_35 |---> P8-35 <---------> J1-7  <--|-----*-----------|    |--> P1-6 >--| 6 Wht Encoder B |
*         |                 |                                |     |      |    |            |                 |
*         |------------------                                |  ------ ------  |            -------------------
*         |      PRU0       |                                |  |    | |    |  |            |      Motor      |
*         |                 |                                |  | R1 | | R2 |  |            |                 |
*         |  gpio0_62 P8_14 |---> P8-16 To Scope             |  |    | |    |  |--> P1-1 >--| 1 Red           |
*         |                 |                                |  ------ ------  |            |                 |
*         |                 |                                |     |      |    |--> P1-2 >--|2 Blk            |
*         | pru_apwm  P8_15 |--> P8-15 >-----------> J1-1 >--| PWM |      |    |            |                 |
*         |                 |                                |     |      |    |            -------------------
*         | prg0_pru0_gpo16 |--> P8-12 >----                 |     --------    |
*         |                 |              |                 |         |       |
*         |-----------------|              |                 |        GND      |  Note: R1 & R2 Form divider for
*         |      PRU1       |              --------> J1-3 <--| Direction       |  Encoder internal resistor to
*         |                 |                                |                 |  divide 5V down to 3.3V.
*         |  gpio0_75 P8_16 |---> P8-14 To scope             |                 |
*         |                 |                                |                 |
*         |            5.0V |---> P9-5 <-------------->   <--| 5V Logic        |
*         |                 |                                |                 |
*         |             GND |---> P9-1  <------------->   <--| GND             |
*         -------------------                                -------------------
*
*         Note: I can never get the feedback polarity right. So if using the wiring
*         as shown above  in the case of a run awy motor, no control.  Try reversing
*         the motor leads.  Or the encoder A and B inputs what ever is easiest.  If
*         that doesn't work reduce the PID integrator gain to 0 and the Proportional
*         gain to 0.  Then try increasing the proportional gain.  PRU0 is the motor
*         controler while PRU1 is for handling linux remoteproc messaging.  The linux
*         application motor-ctrl-msg.h is designed to load PRU0 and PRU1 code.  It also
*         uses sets the pin configurations.
*
**********************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <rsc_types.h>
#include <pru_rpmsg.h>
//#include <pru_virtqueue.h>
#include <pru_gpio.h>
#include "motor_cntrl_msg.h"
#include "motor_control.h"

#include "resource_table.h"
#include "intc_map_0.h"

/* PRU GPIO Registers */
volatile register uint32_t __R30;
volatile register uint32_t __R31;

/* Non-CT register defines */
//#define CM_PER_EPWMSS1 (*((volatile unsigned int *)0x44E000CC)) // Clock module

/* Host-1 Interrupt sets bit 31 in register R31 */
#define HOST_INT               ((uint32_t) 1 << 30)
//  The PRU-ICSS system events used for RPMsg are defined in the Linux device tree.
//  PRU1 uses system event 18 (to ARM) and 19 (from ARM)
#define TO_ARM_HOST              16
#define FROM_ARM_HOST            17


/* The name 'rpmsg-pru' corresponds to the rpmsg_pru driver found at linux-x.y.z/drivers/rpmsg/rpmsg_pru.c */
#define CHAN_NAME               "rpmsg-pru"
//#define CHAN_DESC               "Channel 30"
#define CHAN_PORT               30
/* Used to make sure the Linux drivers are ready for RPMsg communication. Found at linux-x.y.z/include/uapi/linux/virtio_config.h */
#define VIRTIO_CONFIG_S_DRIVER_OK   4

#define PRU0_DATA_RAM           0x00000000
#define PRU1_DATA_RAM           0x00002000
#define PRU_SHARED_RAM0         0x00010000
#define PRU_SHARED_RAM1         0x00011000
#define PRU_SHARED_RAM2         0x00012000
#define PRU_GLB_MEM             0x00012fa0
#define CLKFREQ                 250u

// added   - output section: .share_buff        >  GLB_BUF,      PAGE 2
/* Shared memory setup */
#pragma DATA_SECTION(sharedMem, ".shared_mem")
volatile far shared_mem_t sharedMem;

/* RPMsg variables */
struct pru_rpmsg_transport transport;
uint16_t src;
uint16_t dst;
uint16_t len;


int32_t  direction = NO_DIRECTION;  // No direction = 0, CW = 1, CCW = -1
int32_t  step = 0;
int32_t  targetCps = 0; // Target speed
uint32_t stopFlag = false;

/* Prototypes */
void  rampUpDown(void);
void  initRpmsg(void);
void  rpmsgInterrupt(void);
void  rpmsgIsr(uint16_t src, uint16_t dst);
rpmsg_t rpmsg;

#define DEBUG
#ifdef DEBUG
    unsigned period = 100; // Scope debug period, initialized value
#endif

void main(void) // pru_cfg, pru_ecap.h, sys_pwmss.h
{
    sharedMem.encoderInputCps = targetCps;
    sharedMem.startStop = STOP;
    sharedMem.initFlag = 0;
#ifdef DEBUG
    initScopeDebug1(); // Scope debug pin setup
#endif
    /* Initialize RPMsg */
    initRpmsg();
    sharedMem.initFlag = 1; // Set init flag to signal MotorControl0 to run
    while(true)
    {
        __delay_cycles(CLKFREQ*10000); // Loop rate approximately 10ms
    #ifdef DEBUG
        scopeDebug1(period); // use scope to debug message sent
    #endif
        rpmsgInterrupt();        // Get user space messages
        if(sharedMem.startStop)
            rampUpDown();
    }
}
void rampUpDown(void)
{
    if(step == 0)
        return;
    if(step > 0)
    {
        sharedMem.encoderInputCps+= step;
        if(sharedMem.encoderInputCps >= targetCps)
        {
            sharedMem.encoderInputCps = targetCps;
            if(stopFlag == true)
                sharedMem.startStop = 0;
            step = 0;
        }
    }
    else if(step < 0)
    {
        sharedMem.encoderInputCps+= step;
        if(sharedMem.encoderInputCps <= targetCps)
        {
            sharedMem.encoderInputCps = targetCps;
            if(stopFlag == true)
                sharedMem.startStop = 0;
            step = 0;
        }
    }
}
void rpmsgIsr(uint16_t src, uint16_t dst)
{
    switch(rpmsg.cmd)
    {
    case SHARED_MEM_TEST_CMD:
        sharedMem.test = rpmsg.value;
        rpmsg.value = sharedMem.test;
        break;
    case START_STOP_CMD:
        if(rpmsg.value == 0)
        {   //Stop command
            stopFlag = true;
            targetCps = 0;
            if(direction == 1)
                step = -50;
            else
                step = 50;
        }
        else
            sharedMem.startStop = rpmsg.value;
        break;
    case ENCODER_INPUT_CPS_CMD:
        if(rpmsg.value > 5000)
            rpmsg.value = 5000;
        if(rpmsg.value < -5000)
            rpmsg.value = -5000;
        targetCps = rpmsg.value;
        if(targetCps > 0)
        {
            direction = CW;
            if(targetCps > sharedMem.encoderInputCps)
                step = 50;
            else if(targetCps < sharedMem.encoderInputCps)
                step = -50;
        }
        else
        {
            direction = CCW;
            if(targetCps < sharedMem.encoderInputCps)
                step = -50;
            else if(targetCps > sharedMem.encoderInputCps)
                step = 50;
        }
        break;
    case KP_CMD:
        sharedMem.Kp = rpmsg.value;
        period = 300 + rpmsg.value;
        break;
    case KI_CMD:
        sharedMem.Ki = rpmsg.value;
        period = 400 + rpmsg.value ;
        break;
    case KD_CMD:
        sharedMem.Kd = rpmsg.value;
        period = 500 + rpmsg.value;
        break;
    case GET_PWM_OUTPUT_CMD:
        rpmsg.value = sharedMem.pwmOut;
        break;
    case GET_PID_ERROR_CMD:
        rpmsg.value = sharedMem.pidError;
        break;
    case RETURN_KP_VALUE_CMD:
        rpmsg.value = sharedMem.Kp;
        break;
    case RETURN_KI_VALUE_CMD:
        rpmsg.value = sharedMem.Ki;
        break;
    case RETURN_KD_VALUE_CMD:
        rpmsg.value = sharedMem.Kd;
        break;
    case GET_ENCODER_IN_CPS_CMD:
        rpmsg.value = targetCps;
        break;
    case GET_ENCODER_CPS_CMD:
        rpmsg.value = sharedMem.encoderCps;
        break;
    default:
        break;
    }
    pru_rpmsg_send(&transport, dst, src, (rpmsg_t *)&rpmsg, sizeof(rpmsg));
}
//  Revised for system events rather than mailboxes.
void initRpmsg(void)
{
    volatile uint8_t *status;

    /* Clear the status of the PRU-ICSS system event that the ARM will use to 'kick' us */
    CT_INTC.STATUS_CLR_INDEX_REG_bit.STATUS_CLR_INDEX = FROM_ARM_HOST;

    /* Make sure the Linux drivers are ready for RPMsg communication */
    status = &resourceTable.rpmsg_vdev.status;
    while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

    /* Initialize the RPMsg transport structure */
    pru_rpmsg_init(&transport, &resourceTable.rpmsg_vring0, &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

    /* Create the RPMsg channel between the PRU and ARM user space using the transport structure. */
    while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, CHAN_PORT) != PRU_RPMSG_SUCCESS);
    /* Check bit 30 of register R31 to see if the ARM has kicked us */
}
/* rpmsg_interrupt */
void rpmsgInterrupt(void)
{
    uint16_t src, dst, len;
    // Check bit 31 of register R31 to see if the ARM has kicked us */
    if(__R31 & HOST_INT)
    {
        CT_INTC.STATUS_CLR_INDEX_REG_bit.STATUS_CLR_INDEX = FROM_ARM_HOST; // Clear the event status
        // Receive all available messages, multiple messages can be sent per kick
        if(pru_rpmsg_receive(&transport, &src, &dst, (rpmsg_t *)&rpmsg, &len) == PRU_RPMSG_SUCCESS)
            rpmsgIsr(src, dst); // Service the new interrupt
    }
}
