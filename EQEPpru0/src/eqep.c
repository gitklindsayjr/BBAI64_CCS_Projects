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
 *                                 MotorControl Pin-out
 *                          -----------------------------
 *                          |                           |
 *                          | prg0_pru0_gpo16     P8_12 |--> Direction
 *                          | gpio0_75            P8-14 |--> Scope Debug PRU1
 *                          | ECAP_APWM_OUT_gpo18 P8-15 |--> PWM
 *                          | gpio0_62            P8_16 |--> Scope Debug PRU0
 *                          |                           |                   Encoder
 *                          |                           |              ---------------------
 *                          |        eqep1_b      P8_33 |--<  <--------| White  EQPB       |
 *                          |        eqep1_a      P8_35 |--<  <--------| Yellow EQPA       |
 *                          |                           |              |                   |
 *                          |     VSYS_IO_3V3      P9-3 |-->  >--------| Blue  3.3V - 5V   |
 *                          |             Gnd      P9-1 |-->  >--------| Green GND         |
 *                          |     VSYS_IO_3V3      P9-4 |--> 3.3V      |                   |
 *                          |             Gnd      P9-2 |-->           --------------------
 *                          |     DC_VDD_5V        P9-5 |--> 5.0V
 *                          |     DC_VDD_5V        P9-6 |-->
 *                          |                           |
 *                          -----------------------------
 *
 * Tested with kernel version 5.10.168-ti-r61,a and SD image bbai64-debian-11.7-xfce-arm64-2023-05-03.img.xz
 *
*/

/*
This source code requires that the permissions on the Beaglebone-AI64 /lib/firmware be changed to
group write privileges.  In addition the ownership of the library /lib/firmware changed from
root:root to root:userxx.  This is done so that the pru code can be loaded into the firmware
library.  Else copying to firmware will need sudo privileges.
 *
$ sudo chown root:userxx /lib/firmware
$ sudo chmod g+w /lib/firmware

 Note: The following instructions are based on uploading from your PC/Workstation CCS Project
 Debug Binaries folder "*.out" filefolder to BBAI-64. In my case I drag and drop using the
 RemoteSystems Sftp Files/MyHome on the BBAI-64 using a SSH connection.

The pru code needs to be copied into the firmware library using the shell commands on the BBAI-64.
In this case I assumed that the "chown" and "chmod" from above were not used.

IF --   /properties/Build/Basic Options/Specify output file name = j7-pru0_0-fw.out
    $ sudo cp ~/j7-pru0_0-fw.out /lib/firmware/j7-pru0_0-fw
    $ echo start > /dev/remoteproc/j7-pru0_0/state
    $ echo stop > /dev/remoteproc/j7-pru0_0/state
ELSE IF -- /properties/Build/Basic Options/Specify output file name = EQEPpru0.out
    $ cp EQEPpru0.out /lib/firmware/square-wave
    $ echo square-wave > /dev/remoteproc/j7-pru0_0/firmware
    $ echo start > /dev/remoteproc/j7-pru0_0/state
    $ echo stop > /dev/remoteproc/j7-pru0_0/state
ENDIF

Enable PRU pins thru Overlay Example "MotorControl.dts" as follows:
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
#include <stdbool.h>
#include <limits.h>
//#include <pru_cfg.h>
#include <pru_ecap.h>
//#include <sys_pwmss.h>
#include <pru_intc.h>
//#include <rsc_types.h>
//#include <pru_iep.h>
#include <sys_eqep.h>
#include <sys_gpio.h>
#include <pru_gpio.h>
#include "resource_table.h"
#include "intc_map_0.h"

//#define RPMSG_CONTROL
#ifdef RPMSG_CONTROL
    #include <pru_rpmsg.h>
    #include <pru_virtqueue.h>
#endif

volatile register uint32_t __R30;
volatile register uint32_t __R31;

/* Encoder definitions */
#define TICKS_PER_REV            640
#define SAMPLES_PER_SEC          1000
#define SEC_PER_MIN              60
#define COUNTS_20_KHZ            12500 // 250 Mhz/20 Khz
#define COUNTS_DUTY_CYCLE        125   // COUNTS_20_KHZ/100

/*
 * | b19 b18 b17 b16 | b15 b14 b13 b12 | b11 b10  b9  b8 |  b7  b6  b5  b4 |  b3  b2  b1  b0 |
 * -------------------------------------------------------------------------------------------
 * |  0   0   0   1  |  0   0   0   0  |  0   0   0   0  |  0   0   0   0  |  0   0   0   0  |
 *          1                 0                 0                 0                 0        |
 *  0x10000 for b16
 */
#define DIRECTION_BIT    0x10000

#ifdef RPMSG_CONTROL
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
#endif

static unsigned getEncoderValue();
static void initQep();

#define PRU_ECAP_LOC       0x03000000
volatile far ecap *CT_ECAP_PTR = (volatile ecap *)(PRU_ECAP_LOC);

/* Shared memory setup */
#pragma  DATA_SECTION(sharedMem, ".share_buff")
/* Shared data structure */
typedef struct
{
    int  encoderCps; // Encoder position counts per second
    int  encoderInputCps;
    int  initFlag;
    int  pwmOut;
    /* PID controls */
    int  Kp, Ki, Kd;
    int  setpoint;
    int  pidError;
    int  startStop;
} shared_mem_t;
volatile far shared_mem_t sharedMem;

void main(void)
{
	int32_t scopePeriod, i, dutyCycle = 0;

#ifdef RPMSG_CONTROL
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
    while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, CHAN_PORT) != PRU_RPMSG_SUCCESS);
#endif
    initQep();
    initializeGpio_pru0();
	while(1)
    {
#ifdef RPMSG_CONTROL
	    if (__R31 & HOST_INT)
        {
            /* Clear the event status */
            CT_INTC.STATUS_CLR_INDEX_REG_bit.STATUS_CLR_INDEX = FROM_ARM_HOST;
            /* Receive all available messages, multiple messages can be sent per kick */
            while(pru_rpmsg_receive(&transport, &src, &dst, &dutyCycle, &len) == PRU_RPMSG_SUCCESS)
            {   /* Echo the message back to the same address from which we just received */
                pru_rpmsg_send(&transport, dst, src, &dutyCycle, sizeof(dutyCycle));
                dutyCycle*= COUNTS_DUTY_CYCLE;
                CT_ECAP.CAP2 = dutyCycle;
            }
        }
#endif
/* Statements are used for testing to verify QEP */
	    scopePeriod = EQEP1.QPOSCNT_bit.QPOSCNT;
	    scopePeriod&= 0x3ff;
	    scopePeriod+= 0x200;
	    GPIO0.OUT_DATA23_bit.DO_bit30 = 1;  // Debug signal HI to scope if required
	    for(i=0; i<scopePeriod; i++)
	        __delay_cycles(SCOPE_DEBUG_FREQ);
	    GPIO0.OUT_DATA23_bit.DO_bit30 = 0;  // Debug signal HI to scope if required
        for(i=0; i<scopePeriod; i++)
            __delay_cycles(SCOPE_DEBUG_FREQ);
    }

}
/* Init eQEP */
static void initQep(void) // sys_pwmss.h
{
    // Set RPM memory to 0
    sharedMem.encoderCps = 0;
    // Set RPM memory to 0
    sharedMem.encoderCps = 0;
    EQEP1.QDEC_QEP_CTL = 0;  // Clear control register
    EQEP1.QDEC_QEP_CTL_bit.SWAP = 0;  // Don't swap A & B inputs, bit 10
    EQEP1.QDEC_QEP_CTL_bit.UTE = 1;   // Enable unit timer, bit 17
    EQEP1.QDEC_QEP_CTL_bit.QCLM = 1;  // Enable capture latch on unit time out, bit 18
    EQEP1.QDEC_QEP_CTL_bit.QPEN = 1;  // Enable quadrature position counter, bit 19
    EQEP1.QDEC_QEP_CTL_bit.SWI = 1;   // Enable software loading of position counter, bit 23
    EQEP1.QCAP_QPOS_CTL_bit.CCPS = 7; // EQEP_FICLK/128, UPEVENT = QCLK/1
    EQEP1.QPOSCNT_bit.QPOSCNT = 0x00000000; // Clear eQEP POS counter
    EQEP1.QPOSMAX_bit.QPOSMAX = UINT_MAX; // Set max encoder count
    EQEP1.QUTMR_bit.UNITTMR = 0x00000000; // Clear unit timer
    EQEP1.QINT_CLR_FRC = 0x0000FFFF;      // Clear interrupt flagso
}
//  get_enc_rpm()
static unsigned getEncoderValue(void)
{
    static unsigned lastCount = 0;
    unsigned count;
    int      deltaCount;

    // Check for overrun/overflow errors, show RPM as 0
    count = EQEP1.QPOSCNT_bit.QPOSCNT;
    deltaCount = (int)(count - lastCount);
    lastCount = count;
    deltaCount*= SAMPLES_PER_SEC;
    sharedMem.encoderCps= deltaCount;
    return count;
}
