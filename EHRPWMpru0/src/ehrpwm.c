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

/****************************************************************************************************
 * This app is for ICCSG0:pru0, see below for remoteproc mapping
 *  GPIO0_62 is on P8_16 as output and __r30 P8_12 use MotorControl.dts as overlay
 *  Demonstrates the use of System Peripherals, GPIO's and PWM via pru register control
 *
 * Tested with kernel version 5.10.168-ti-r103, bbai64-debian-11.7-xfce-arm64-2023-12-03
 * And with kernel version 5.10.168-ti-r103, bbai64-debian-12.2-minimal-arm64-2023-10-07
 *         after replacing the /opt/source/dtb-5.10-ti/src/arm64/. and /overlays with
 *         kernel version 5.10.168-ti-r103, bbai-debian-11.7 device trees
 *
 * Source code built and uploaded from CCS12.5, TI's Code Composer Studio, pru compiler version 2.3.3
 * TI's pru support package is contained within the WORKSPACE as "PruSoftwareSupport"
****************************************************************************************************
 *
 *                             -----------------------
 *                             |                     |
 *                             |       r30_16  P8_12 |-->
 *                             |                     |
 *                             |      gpio0_62 P8-16 |-->
 *                             |                     |
 *                             |      ehrpwm_b P9_25 |-->
 *                             |                     |
 *                             |           gnd P9-1  |-->
 *                             |                     |
 *                             |           gnd P9-2  |-->
 *                             |                     |
 *                             |           gnd P8-1  |-->
 *                             |                     |
 *                             |           gnd P8-2  |-->
 *                             -----------------------
*/

/*
This source code requires that the permissions on the Beaglebone-AI64 /lib/firmware be changed to
group write privileges.  In addition the ownership of the library /lib/firmware changed from
root:root to root:userxx.  This is done so that the pru code can be loaded into the firmware
library.  Else copying to firmware will need sudo privileges.
 *
$ sudo chown root:userxx /lib/firmware
$ sudo chmod g+w /lib/firmware

 Note: The following instructions are based on uploading from your PC/Workstation from a CCS Project
 Debug Binaries folder "*.out" file folder to BBAI-64. In my case I drag and drop using the
 RemoteSystems Sftp Files/MyHome on the BBAI-64 using a SSH connection.

The pru code needs to be copied into the firmware library using the shell commands on the BBAI-64.
In this case I assumed that the "chown" and "chmod" from above were not used.

ICCSG Mapping: Refer to k3-j721e-main.dtsi
    ICCSG0_PRU0 (aka PRG0) -> /dev/remoteproc/j7-pru0_0
    ICCSG0_PRU1 (aka PRG0) -> /dev/remoteproc/j7-pru0_1
    ICCSG1_PRU0 (aka PRG1) -> /dev/remoteproc/j7-pru1_0
    ICCSG1_PRU1 Iaka PRG1) -> /dev/remoteproc/j7-pru1_1

IF --   /properties/Build/Basic Options/Specify output file name = j7-pru0_0-fw.out
    $ sudo cp ~/SquareWave0.out /lib/firmware/j7-pru0_0-fw
    $ echo start > /dev/remoteproc/j7-pru0_0/state
    $ echo stop > /dev/remoteproc/j7-pru0_0/state
ELSE IF -- /properties/Build/Basic Options/Specify output file name = square-wave.out
    $ cp EHRPWMpru0out /lib/firmware/EHRPWMpru0
    $ echo EHRPWMpru0 > /dev/remoteproc/j7-pru0_0/firmware
    $ echo start > /dev/remoteproc/j7-pru0_0/state
    $ echo stop > /dev/remoteproc/j7-pru0_0/state
ENDIF

Enable PRU pins thru Overlay Example "SquareWave.dts" as follows:
$ cd /opt/source/dtb-5.10-ti
$ sudo cp ~/MotorControl.dts src/arm64/overlays
$ make all_arm64
$ make install_arm64
$ cd /boot/firmware/extlinux
Add SquareWave.dtbo to extlinux.conf
$ sudo nano extlinux.conf
label Linux microSD
    kernel /Image
    append root=/dev/mmcblk1p2 ro rootfstype=ext4 rootwait net.iframes=0 quiet
    fdtdir /
    fdtoverlays /overlays/MotorControl.dtbo
    initrd /initrd.img
$ sudo reboot
*/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <j721e/pru_intc.h>
#include <rsc_types.h>

/* Defintion to control the duty cycle (1-99) thru /dev/rmpmsg_pru30 comment out if not wanted
 * #define RPMSG_CONTROL
 *         or
 * Use compiler predefined symbol: Project->Properties->Build->PRU Compiler->Predefined Symbols
 * Usage:  $ echo xx > /dev/rpmsg_pru30
 */
#define RPMSG_CONTROL
#ifdef RPMSG_CONTROL
    #include <pru_rpmsg.h>
    #include <pru_virtqueue.h>
#endif
#include <sys_pwmss.h>
#include <sys_ctrlmmr.h>
#include <pru_gpio.h>
#include <intc_map_0.h> /* Added to suppress warning #10068-D: no matching section */
#include <resource_table.h>

volatile register uint32_t __R30;
volatile register uint32_t __R31;

#define CLK_FREQ_MHZ       250
#define PULSE_WIDTH_US     500
#define PW_CLK_CYCLES      (CLK_FREQ_MHZ - 11) // Clock cycles adjusted for for loop
/* Set Time base clock to: TBCLK = SYSCLkOUT/(HSPCLKDIV * CLKDDIV) = 250Mhz/(2) = 125 Mhz
 * Example:
 * PWM Frequency = 20 Khz - 125,000,000/20,000 = 6,250 for TPRD count value
 * PWM Frequency =  5 Khz - 125,000,000/5,000 = 25,000 for TPRD count value
 */
#define PWM_CLOCK_SCALAR_HZ  125000000
#define PWM_PERIOD_COUNTS     (PWM_CLOCK_SCALAR_HZ/5000)

/* This helps me convert bits to hex
 * | b19 b18 b17 b16 | b15 b14 b13 b12 | b11 b10  b9  b8 |  b7  b6  b5  b4 |  b3  b2  b1  b0 |
 * -------------------------------------------------------------------------------------------
 * |  0   0   0   1  |  0   0   0   0  |  0   0   0   0  |  0   0   0   01  |  0   0   0   0  |
 *          1                 0                 0                 0                  0        |
 *  0x00010000 for b16
 */
#ifdef RPMSG_CONTROL
/* The following code was copied and pasted from the:
 * ~/ti/pru-software-support-package/examples/j721ePRU_RPMsg_Echo_interrupt0/main.c
 * lines 41 - 72
 */
    /* Host-0 Interrupt sets bit 30 in register R31 */
    #define HOST_INT            ((uint32_t) 1 << 30)
    /* The PRU-ICSS system events used for RPMsg are defined in the Linux device tree
     * PRU0 uses system event 16 (To ARM) and 17 (From ARM)
     * PRU1 uses system event 18 (To ARM) and 19 (From ARM) */
    #define TO_ARM_HOST         16
    #define FROM_ARM_HOST       17
    /* Using the name 'rpmsg-pru' will probe the rpmsg_pru driver found
     * at linux-x.y.z/drivers/rpmsg/rpmsg_pru.c */
    #define CHAN_NAME           "rpmsg-pru"
    #define CHAN_PORT           30
    #define XSTR(x)             #x
    #define CHAN_DESC(x)        "Channel "XSTR(x)
    /* Used to make sure the Linux drivers are ready for RPMsg communication
     * Found at linux-x.y.z/include/uapi/linux/virtio_config.h */
    #define VIRTIO_CONFIG_S_DRIVER_OK   4

    uint8_t payload[RPMSG_MESSAGE_SIZE];
#endif

#define PERIOD    (PULSE_WIDTH_US*CLK_FREQ_MHZ)

uint32_t initPwm(uint32_t freq);
int char2int(uint8_t *cStr, size_t length);
void main(void)
{
    uint32_t periodCounts, temp, period = 1000; // Period in US for scope, 1000 Hz
    uint32_t ctrlMMR_stat, dutyCycle = 50; // 50%

#ifdef RPMSG_CONTROL
    /* The following code was copied and pasted from the:
     * ~/ti/pru-software-support-package/examples/j721ePRU_RPMsg_Echo_interrupt0/main.c
     * lines 79 - 94
     */
    struct pru_rpmsg_transport transport;
    uint16_t src, dst, len;
    volatile uint8_t *status;

    /* Clear the status of the PRU-ICSS system event that the ARM will use to 'kick' us */
    CT_INTC.STATUS_CLR_INDEX_REG_bit.STATUS_CLR_INDEX = FROM_ARM_HOST;

    /* Make sure the Linux drivers are ready for RPMsg communication */
    status = &resourceTable.rpmsg_vdev.status;
    while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

    /* Initialize the RPMsg transport structure */
    pru_rpmsg_init(&transport, &resourceTable.rpmsg_vring0, &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

    /* Create the RPMsg channel between the PRU and ARM user space using the transport structure. */
    while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, CHAN_DESC(CHAN_PORT), CHAN_PORT) != PRU_RPMSG_SUCCESS);
#endif
    initScopeDebug0();
    periodCounts = initPwm(5*period); // 5000 Hz
    EHRPWM_CTRLMMR.EHRPWM4_bit.TB_CLKEN = 1; // Enable clocks
    ctrlMMR_stat = EHRPWM_CTRLMMR.EHRPWM4_bit.TB_CLKEN;
    while(1)
    {
    #ifdef RPMSG_CONTROL
        /* The following code was copied and pasted from the:
         * ~/ti/pru-software-support-package/examples/j721ePRU_RPMsg_Echo_interrupt0/main.c
         * lines 96 - 106
         */
        /* Check bit 31 of register R30 to see if the ARM has kicked us */
        if (__R31 & HOST_INT)
        {
            /* Clear the event status */
            CT_INTC.STATUS_CLR_INDEX_REG_bit.STATUS_CLR_INDEX = FROM_ARM_HOST;
            /* Receive all available messages, multiple messages can be sent per kick */
            while(pru_rpmsg_receive(&transport, &src, &dst, payload, &len) == PRU_RPMSG_SUCCESS)
            {
                /* Echo the message back to the same address from which we just received */
                 pru_rpmsg_send(&transport, dst, src, payload, len);
                 dutyCycle = char2int(payload, len); /* Convert to integer */
                 temp = periodCounts;
                 temp*= dutyCycle;
                 temp/= 100;
                 EPWM4.CMPB = temp;
           }
        }
    #endif
        scopeDebug0(period); // Uses P8_16
        // __delay_cycles(PERIOD);  // Note: this will modifiy the actual period
        __R30^= 0x10000; // b16 this pruout pit toggles at 2x the period
    }
}
/* Initialize EHRPWM B */
uint32_t initPwm(uint32_t freq) // pru_ecap.h
{
    uint32_t periodCounts;
    /* Clear Time base control: Sets the following
     * No SYNC, Active Period load from shadow register, No load from phase register, Counter mode count up
     */
    EPWM4.TBCTL = 0x0000; // Clear Time base Control
    /* Set Time base clock to: TBCLK = SYSCLkOUT/(HSPCLKDIV * CLKDDIV) = 250Mhz/(2) = 125 Mhz
     * PWM Frequency = 5000 - 125,000,000/5,000 = 25,000 for TPRD count value
     */
    EPWM4.TBCTL_bit.CTRMODE = TB_COUNT_UP;
    EPWM4.TBCTL_bit.PHSEN = TB_DISABLE;
    EPWM4.TBCTL_bit.PRDLD = TB_SHADOW;
    EPWM4.TBCTL_bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPWM4.TBCTL_bit.HSPCLKDIV = TB_DIV1; // Divide by 1
    EPWM4.TBCTL_bit.CLKDIV = TB_DIV1;    // Divide by 1
    EPWM4.CMPCTL = 0x0000;               // Clear register
//    EPWM4.CMPCTL_bit.LOADAMODE = CC_CTR_ZERO;
//    EPWM4.CMPCTL_bit.SHDWAMODE = CC_SHADOW;
    EPWM4.CMPCTL_bit.LOADBMODE = CC_CTR_ZERO;
    EPWM4.CMPCTL_bit.SHDWBMODE = CC_SHADOW;
//    EPWM4.AQCTLA = 0x0000;         // Clear register
//    EPWM4.AQCTLA_bit.ZRO = AQ_SET; // Force EPWM4A Output HI when TBCNT = 0
//    EPWM4.AQCTLA_bit.CAU = AQ_CLEAR; // Force EPWM4A Output LO when TBCNT = CMPA
//    EPWM4.AQCTLA_bit.PRD = AQ_NO_ACTION; // Do nothing when TBCNT = TPRD
    EPWM4.AQCTLB = 0x0000;         // Clear register
    EPWM4.AQCTLB_bit.ZRO = AQ_SET; // Force EPWM4B Output HI when TBCNT = 0
    EPWM4.AQCTLB_bit.CBU = AQ_CLEAR; // Force EPWM4B Output LO when TBCNT = CMPB
    EPWM4.AQCTLB_bit.PRD = AQ_NO_ACTION; // Do nothing when TBCNT = TPRD
    periodCounts = PWM_CLOCK_SCALAR_HZ;
    periodCounts/= freq;
    EPWM4.TBPRD = periodCounts;
    EPWM4.TBPHS = 0x0000;    // Clear register
    EPWM4.CMPB = periodCounts/2;
    return periodCounts;
}
/* This function is needed to convert un-terminated (no null char) ASCII string to number.
 * Library functions like atoi(), or strtol() don't work.
 */
int char2int (uint8_t *cStr, size_t n)
{
    int number = 0;
    int mult = 1;

    n = (int)n < 0 ? -n : n; /* quick absolute value check  */
    /* for each character in the string */
    while (n--)
    {
        /* if not digit or '-', check if number > 0, break or continue */
        if((cStr[n] < '0' || cStr[n] > '9') && cStr[n] != '-')
        {
            if (number)
                break;
            else
                continue;
        }
        if (cStr[n] == '-')
        {   /* if '-' if number, negate, break */
            if (number) {
                number = -number;
                break;
            }
        }
        else
        {   /* convert digit to numeric value   */
            number += (cStr[n] - '0') * mult;
            mult *= 10;
        }
    }
    return number;
}
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
/* PWM out on P9_25 EHRPWM4_B
 * PRU0 Scope Debug prg0_pru0_gpo19 out on P8-16
 * Motor Direction  prg0_pru0_gpo16 out on P8-12
 * PRU1 Scope Debug prg0_pru1_gpo12
 * GND         P9-1,2 and P8-1,2
 * DC_VDD_5V   P9-5,6
 * VSYS_IO_3V3 P9-3,4
 * Note:!!!!!!
 * Don't know what is wrong but this app works only by using Linux commands to enable the
 * /dev/bone/pwm/bbai64_P9_25_ehrpwm4_b
 * The solution is probably found in the device tree/overlays.
 */



#define RPMSG_CONTROL

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <j721e/pru_intc.h>
#include <rsc_types.h>
#include <pru_cfg.h>
#include <sys_pwmss.h>
#include <sys_gpio.h>
#include <sys_ctrlmmr.h>
#ifdef RPMSG_CONTROL
    #include <pru_rpmsg.h>
    #include <pru_virtqueue.h>
#endif
#include <resource_table.>
#include <intc_map_0.h>

volatile register uint32_t __R30;
volatile register uint32_t __R31;

#define CLK_FREQ_MHZ       250
#define PULSE_WIDTH_US     500
#define PW_CLK_CYCLES      (CLK_FREQ_MHZ - 11) // Clock cycles adjusted 1us for loop
/*
 * | b19 b18 b17 b16 | b15 b14 b13 b12 | b11 b10  b9  b8 |  b7  b6  b5  b4 |  b3  b2  b1  b0 |
 * -------------------------------------------------------------------------------------------
 * |  0   0   0   1  |  0   0   0   0  |  0   0   0   0  |  0   0   0   0  |  0   0   0   0  |
 *          1                 0                 0                 0                 0        |
 *  0x00010000 for gpio_b16
 *  On the J721E Pru Connector p8_16
*/

#ifdef RPMSG_CONTROL
/* The following code was copied and pasted from the:
 * ~/ti/pru-software-support-package/examples/j721ePRU_RPMsg_Echo_interrupt0/main.c
 * lines 41 - 72
 */
    /* Host-0 Interrupt sets bit 30 in register R31 */
    #define HOST_INT            ((uint32_t) 1 << 30)
    /* The PRU-ICSS system events used for RPMsg are defined in the Linux device tree
     * PRU0 uses system event 16 (To ARM) and 17 (From ARM)
     * PRU1 uses system event 18 (To ARM) and 19 (From ARM) */
    #define TO_ARM_HOST         16
    #define FROM_ARM_HOST       17
    /* Using the name 'rpmsg-pru' will probe the rpmsg_pru driver found at linux-x.y.z/drivers/rpmsg/rpmsg_pru.c */
    #define CHAN_NAME           "rpmsg-pru"
    #define CHAN_PORT           30
    #define XSTR(x)             #x
    #define CHAN_DESC(x)        "Channel "XSTR(x)
    /* Used to make sure the Linux drivers are ready for RPMsg communication. Found at linux-x.y.z/include/uapi/linux/virtio_config.h */
    #define VIRTIO_CONFIG_S_DRIVER_OK   4

uint8_t payload[RPMSG_MESSAGE_SIZE];
#endif

void initPwm(void);
int  char2int(uint8_t *cStr, size_t length);

void main(void)
{
    volatile uint32_t gpio, i, j;
    uint32_t pulseWidth, ctrlMMR_stat;
    uint32_t dutyCycle = 0;

#ifdef RPMSG_CONTROL
    /* The following code was copied and pasted from the:
     * ~/ti/pru-software-support-package/examples/j721ePRU_RPMsg_Echo_interrupt0/main.c
     * lines 79 - 94
     */
    struct pru_rpmsg_transport transport;
    uint16_t src, dst, len;
    volatile uint8_t *status;

    __R31 = 0x00000000; // Clear any pending interrupts
    __R30 = 0x00000000; // Clear any pending interrupts

    /* Clear the status of the PRU-ICSS system event that the ARM will use to 'kick' us */
    CT_INTC.STATUS_CLR_INDEX_REG_bit.STATUS_CLR_INDEX = FROM_ARM_HOST;

    /* Make sure the Linux drivers are ready for RPMsg communication */
    status = &resourceTable.rpmsg_vdev.status;
    while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

    /* Initialize the RPMsg transport structure */
    pru_rpmsg_init(&transport, &resourceTable.rpmsg_vring0, &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

    /* Create the RPMsg channel between the PRU and ARM user space using the transport structure. */
    while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, CHAN_DESC(CHAN_PORT), CHAN_PORT) != PRU_RPMSG_SUCCESS);
#endif

    GPIO0.DIR23_bit.OE_bit30 = 0;      // BBAI-64 Connector P8_16 set as output
    GPIO0.OUT_DATA23_bit.DO_bit30 = 1; // Set GPIO0 P8-16 HI

    gpio = 0x00010000;
    pulseWidth = PULSE_WIDTH_US;
   __R30 = gpio;

    initPwm();
    EHRPWM_CTRLMMR.EHRPWM4_bit.TB_CLKEN = 1; // Enable clocks
    ctrlMMR_stat = EHRPWM_CTRLMMR.EHRPWM4_bit.TB_CLKEN;
    GPIO0.DIR45_bit.OE_bit11 = 0;    // Set Debug pin GPIO0_75 as output
    GPIO0.OUT_DATA45_bit.DO_bit11 = ctrlMMR_stat; // Set state of TB_CLKEN
    while(1)
    {
    #ifdef RPMSG_CONTROL
        /* The following code was copied and pasted from the:
         * ~/ti/pru-software-support-package/examples/j721ePRU_RPMsg_Echo_interrupt0/main.c
         * lines 96 - 106
         */
        if (__R31 & HOST_INT)
        {
            /* Clear the event status */
            CT_INTC.STATUS_CLR_INDEX_REG_bit.STATUS_CLR_INDEX = FROM_ARM_HOST;
            /* Receive all available messages, multiple messages can be sent per kick */
            while(pru_rpmsg_receive(&transport, &src, &dst, payload, &len) == PRU_RPMSG_SUCCESS)
            {  /* Echo the message back to the same address from which we just received */
                pru_rpmsg_send(&transport, dst, src, payload, len);
                pulseWidth = char2int(payload, len); /* Convert to integer */
            }
        }
    #endif
        __R30 = gpio; // Set HI
        GPIO0.OUT_DATA23_bit.DO_bit30 = 1; // P8-16 HI
        for(i = 0; i<pulseWidth; i++)
            __delay_cycles(PW_CLK_CYCLES);
        __R30 = 0;
        GPIO0.OUT_DATA23_bit.DO_bit30 = 0; // P8-16 LO
        for(i = 0; i<pulseWidth; i++)
            __delay_cycles(PW_CLK_CYCLES);
        if(dutyCycle < 6250)
        {
            EPWM4.CMPB = dutyCycle;
            dutyCycle++;
        }
        else
            dutyCycle = 0;
    }
}
/* Init EHRPWM mode */
void initPwm(void) // pru_ecap.h
{
    /* Clear Time base control: Sets the following
     * No SYNC, Active Period load from shadow register, No load from phase register, Counter mode count up
     */
    EPWM4.TBCTL = 0x0000; // Clear Time base Control
    /* Set Time base clock to: TBCLK = SYSCLkOUT/(HSPCLKDIV * CLKDDIV) = 250Mhz/(2) = 125 Mhz
     * PWM Frequency = 20 Khz - 125,000/20 = 6250 for TPRD count value
     */
    EPWM4.TBCTL_bit.CTRMODE = TB_COUNT_UP;
    EPWM4.TBCTL_bit.PHSEN = TB_DISABLE;
    EPWM4.TBCTL_bit.PRDLD = TB_SHADOW;
    EPWM4.TBCTL_bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPWM4.TBCTL_bit.HSPCLKDIV = TB_DIV1; // Divide by 1
    EPWM4.TBCTL_bit.CLKDIV = TB_DIV1;    // Divide by 1
    EPWM4.CMPCTL = 0x0000;               // Clear register
//    EPWM4.CMPCTL_bit.LOADAMODE = CC_CTR_ZERO;
//    EPWM4.CMPCTL_bit.SHDWAMODE = CC_SHADOW;
    EPWM4.CMPCTL_bit.LOADBMODE = CC_CTR_ZERO;
    EPWM4.CMPCTL_bit.SHDWBMODE = CC_SHADOW;
//    EPWM4.AQCTLA = 0x0000;         // Clear register
//    EPWM4.AQCTLA_bit.ZRO = AQ_SET; // Force EPWM4A Output HI when TBCNT = 0
//    EPWM4.AQCTLA_bit.CAU = AQ_CLEAR; // Force EPWM4A Output LO when TBCNT = CMPA
//    EPWM4.AQCTLA_bit.PRD = AQ_NO_ACTION; // Do nothing when TBCNT = TPRD
    EPWM4.AQCTLB = 0x0000;         // Clear register
    EPWM4.AQCTLB_bit.ZRO = AQ_SET; // Force EPWM4B Output HI when TBCNT = 0
    EPWM4.AQCTLB_bit.CBU = AQ_CLEAR; // Force EPWM4B Output LO when TBCNT = CMPB
    EPWM4.AQCTLB_bit.PRD = AQ_NO_ACTION; // Do nothing when TBCNT = TPRD
    EPWM4.TBPRD = 6250;      // 20 Khz sample rate
    EPWM4.TBPHS = 0x0000;    // Clear register
//    EPWM4.CMPA = 3125; // 50% Duty cycle
    EPWM4.CMPB = 3125; // 50% Duty cycle

}
/* This function is needed to convert un-terminated (no null char) ASCII string to number.
 * Library functions like atoi(), or strtol() don't work.
 */
int char2int (uint8_t *cStr, size_t n)
{
    int number = 0;
    int mult = 1;

    n = (int)n < 0 ? -n : n; /* quick absolute value check  */
    /* for each character in the string */
    while (n--)
    {
        /* if not digit or '-', check if number > 0, break or continue */
        if((cStr[n] < '0' || cStr[n] > '9') && cStr[n] != '-')
        {
            if (number)
                break;
            else
                continue;
        }
        if (cStr[n] == '-')
        {   /* if '-' if number, negate, break */
            if (number) {
                number = -number;
                break;
            }
        }
        else
        {   /* convert digit to numeric value   */
            number += (cStr[n] - '0') * mult;
            mult *= 10;
        }
    }
    return number;
}

