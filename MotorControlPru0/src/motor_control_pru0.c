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
/*********************************************************************************
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
*         |        gpio0_62 |---> P8-16 To Scope             |  |    | |    |  |--> P1-1 >--| 1 Red           |
*         |                 |                                |  ------ ------  |            |                 |
*         |                 |                                |     |      |    |--> P1-2 >--|2 Blk            |
*         | pru_apwm  P8_15 |--> P8-15 >-----------> J1-1 >--| PWM |      |    |            |                 |
*         |                 |                                |     |      |    |            -------------------
*         | prg0_pru0_gpo16 |--> P8-12 >----                 |     --------    |
*         |                 |              |                 |         |       |
*         |-----------------|              |                 |        GND      |  Note: R1 & R2 Form divider for
*         |      PRU1       |              --------> J1-3 <--| Direction       |  Encoder internal resistor to
*         |                 |                                |                 |  divide 5V down to 3.3V.
*         |        gpio0_75 |---> P8-14 To scope             |                 |
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
*         controler and handles the linux remoteproc messaging.  The linux application
*         motor-ctrl-msg.h is designed to load PRU0.  Pin configurations is handled via
*         the MotorControl.dts overlay.
*
**********************************************************************************/
// Project built with CCS12.3 with pru_support-package-20.2 running on the BBAI-64
// Linux BeagleBone 5.10.168-ti-arm64-r103 Bullseye

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <pru_cfg.h>
#include <pru_ecap.h>
#include <sys_pwmss.h>
#include <pru_intc.h>
#include <rsc_types.h>
#include <pru_iep.h>
#include <sys_eqep.h>
#include <pru_gpio.h>
#include <sys_gpio.h>
#include <pru_rpmsg.h>
#include "motor_cntrl_msg.h"
#include "motor_control.h"
#include "resource_table.h"
#include "intc_map_0.h"

volatile register uint32_t __R30;
volatile register uint32_t __R31;

/* Host-1 Interrupt sets bit 31 in register R31 */
#define HOST_INT               ((uint32_t) 1 << 30)
//  The PRU-ICSS system events used for RPMsg are defined in the Linux device tree.
#define TO_ARM_HOST              16
#define FROM_ARM_HOST            17

/* The name 'rpmsg-pru' corresponds to the rpmsg_pru driver found at linux-x.y.z/drivers/rpmsg/rpmsg_pru.c */
#define CHAN_NAME               "rpmsg-pru"
#define CHAN_PORT               30
/* Used to make sure the Linux drivers are ready for RPMsg communication. Found at linux-x.y.z/include/uapi/linux/virtio_config.h */
#define VIRTIO_CONFIG_S_DRIVER_OK   4

/* RPMsg variables */
struct pru_rpmsg_transport transport;
uint16_t src;
uint16_t dst;
uint16_t len;
int32_t  direction = NO_DIRECTION;  // No direction = 0, CW = 1, CCW = -1
int32_t  step = 0;
int32_t  targetCps = 0; // Target speed
uint32_t stopFlag = false;
uint32_t period = 100; // Scope debug period, initialized value

/* Prototypes */
void  rampUpDown(void);
void  initRpmsg(void);
void  rpmsgInterrupt(void);
void  rpmsgIsr(uint16_t src, uint16_t dst);
rpmsg_t rpmsg;

#define HOST_INT1         ((uint32_t) 1 << 31)

// Global memory map page 185, global PRU_ICSS offsets
//#define PRU0_DATA_RAM           0x00000000
//#define PRU1_DATA_RAM           0x00002000
//#define PRU_SHARED_RAM0         0x00010000
//#define PRU_SHARED_RAM1         0x00011000
//#define PRU_SHARED_RAM2         0x00012000
//#define PRU_GLB_MEM             0x00012fa8

#define Delta_T                .001   // Sample time 1ms
#define Sample_rate            1000   // Sample rate 1/Delta_T
#define CLK_FREQ_MHZ           250
#define PULSE_WIDTH_US         500
#define PW_CLK_CYCLES         (CLK_FREQ_MHZ - 11) // Clock cycles adjusted for for loop

/* Function prototypes */
uint32_t getEncoderValue();
void initQep();
void updatePid(void);
void initPid(void);
void initPwm(void); // pru_ecap.h
void initializeTimer(void);
//void initializeIntc(void);
//bool interruptSvc(void);

//#define SIMULATE_MOTOR
#ifdef SIMULATE_MOTOR
    int  simulateMotor(void);
#endif

/* Shared memory setup */
//#pragma  DATA_SECTION(motorParams, ".shared_mem")
volatile motor_params_t motorParams;

//volatile far gpio_t *GPIO0_PTR = (volatile gpio_t *)(GPIO0_OFFSET);
//volatile far eqep_t *EQEP0_PTR = (volatile eqep_t *)(EQEP0_OFFSET);

void main(void)
{
    volatile uint32_t gpio;
    int32_t i;
    int32_t count;
//    uint32_t hit = false;
    __R31 = 0x00000000; // Clear any pending interrupts
    __R30 = 0x00000000; // Clear any pending interrupts

   /* Initialize RPMsg */
    initRpmsg();
    rpmsgInterrupt();
    initializeTimer();
    // Enable timer
    CT_IEP0.global_cfg_reg_bit.cnt_enable = 1;

    initQep();
    initPwm();
    initPid();

    motorParams.startStop = 0;
    motorParams.test = 500;
    initScopeDebug1();
    initScopeDebug0();
    motorParams.initFlag = 1; // Set init flag to signal MotorControl0 to run
#define DEBUG
#ifdef DEBUG
    initScopeDebug0();
    initScopeDebug1();
#endif
    while(true)
    {
#ifdef RPMSG
#endif
        // Poll until timeout
        if(CT_IEP0.cmp_status_reg_bit.cmp_status == 1)
        {   // Clear overflow status register for debug puposes for single stepping
            CT_IEP0.global_status_reg_bit.cnt_ovf = 1;
            CT_IEP0.cmp0_reg0 = 250000; // 1ms @ 250MHz
            CT_IEP0.count_reg0 = 0;
            rpmsgInterrupt();        // Get user space messages
            if(step != 0)
                rampUpDown();
            count = getEncoderValue();
            updatePid();
            CT_ECAP.CAP2_bit.CAP2 = motorParams.pwmOut; //  Duty Cycle
        #ifdef DEBUG
            count = motorParams.pidError;
            if(count <= -500)
                    count = -500;
            else if(count >= 500)
                count = 500;
            count+= 500; // Add offset
//            count = motorParams.pwmOut + PWM_OFFSET;
//            count/= 20;
//             scopeDebug0(count);
            GPIO0.OUT_DATA23_bit.DO_bit30 = 1; // P8_16 GPIO0_62 HI
            for(i = 0; i<count; i++)
                __delay_cycles(PW_CLK_CYCLES);
             GPIO0.OUT_DATA23_bit.DO_bit30 = 0; // P8_16 GPIO0_62 LO
        #endif

        }
    }
}
void updatePid(void)
{
    float  xDot, p, d, output;
    static float x = 0, y = 0, i = 0;
    int    error;
    #ifdef SIMULATE_MOTOR
        motorParams.encoderCps = simulateMotor();
    #endif

    /* Calculate error */
    error = (motorParams.encoderInputCps - motorParams.encoderCps); // Input encoder RPM's time base, calculate error signal
    xDot = (float)(error - x);
    x+= .025*xDot; // Filter freq Delta_T x 50 Hz
    /* Calculate P term */
    motorParams.pidError = (int)x;
    p = motorParams.Kp*x;
    /* Integrate I term */
    i+= Delta_T*motorParams.Ki*x;
    /* Calculate D term */
    d = motorParams.Kd*(x - y);
    y = x;
    /* Sum PID output */
    output = p + d + i;
    // Set Motor Control Direction
    if(output < CCW)
    {   // Reverse
        __R30|= DIRECTION_BIT;  // Direction reverse
        motorParams.pwmOut = -output;
    }
    else if(output > CCW)
    {   // Forward
        __R30&= ~DIRECTION_BIT; // Direction Forward
        motorParams.pwmOut = output;
    }
    else
        motorParams.pwmOut = 0;
    /* Check min/max output */
    if(motorParams.pwmOut < 0)
        motorParams.pwmOut = 0;
    if(motorParams.pwmOut > PERIOD_COUNT)
        motorParams.pwmOut = PERIOD_COUNT;
    if(motorParams.startStop == 0)
    {
        motorParams.pwmOut = 0; // No pwmOut
        i = 0; y = 0;          // 0 integrator, and differential last values
    }
}
/* init_pid */
void initPid(void)
{
    /* Set default PID tunings */
    motorParams.Kp    = 32;
    motorParams.Ki    = 16;
    motorParams.Kd    = 64;
    motorParams.pwmOut = 0;
    motorParams.encoderInputCps = 0;
    motorParams.encoderCps = 0;
    motorParams.pidError = 0;
}
/* Init APWM */
void initPwm(void) // pru_ecap.h
{
    /* Set PWM memory to 0 */
    motorParams.pwmOut = 0;
    /* Enable ECAP clock */
    CT_CFG.cgr_reg_bit.ecap_clk_en = 1;
    CT_ECAP.ECCTL2_ECCTL1_bit.TSCNTSTP = 0;  // S0p counter ECAP PWM Free running
     /* Set number of clk cycles in a PWM period (APRD) CAP2 used as compare/duty cycle */
    CT_ECAP.CAP1_bit.CAP1 = 12500; // 250 Mhz/20 Khz = 12500
    CT_ECAP.CAP2_bit.CAP2 = 6250;  // Duty Cycle
   /* Enable ECAP PWM Free run counter */
    CT_ECAP.ECCTL2_ECCTL1 = 0;               // Clear ECCTL2 and ECCTL1
    CT_ECAP.ECCTL2_ECCTL1_bit.CAP_APWM = 1;  // ECAP operates in APWM mode
    CT_ECAP.ECCTL2_ECCTL1_bit.APWMPOL = 0;   // Active high
    CT_ECAP.ECCTL2_ECCTL1_bit.SYNCO_SEL = 3; // Disable sync out
    CT_ECAP.ECCTL2_ECCTL1_bit.TSCNTSTP = 1;  // Start counter ECAP PWM Free running
//    __delay_cycles(1000*CLK_FREQ_MHZ);
//    CT_ECAP.CAP2 = 6250;  // Duty Cycle
}
void initializeTimer(void)
{
    /* Disable counter */
    CT_IEP0.global_cfg_reg_bit.cnt_enable = 0;
    /* Reset Count registers */
    CT_IEP0.count_reg0 =0;
    CT_IEP0.count_reg1 = 0;
    /* Clear overflow status register */
    CT_IEP0.global_status_reg_bit.cnt_ovf = 1;
    /* Clear compare status */
    CT_IEP0.cmp_status_reg_bit.cmp_status = 1;
    /* Set compare value LO register */
    CT_IEP0.cmp0_reg0 = 250000; // 1ms @ 250MHz
    CT_IEP0.cmp0_reg1 = 0;
    /* Enable compare events 0-15 */
    CT_IEP0.cmp_cfg_reg_bit.cmp_en = 1;
    /* Disable reset counter for CMP0 event */
    CT_IEP0.cmp_cfg_reg_bit.cmp0_rst_cnt_en = 0;
    /* Set increment value to 1 count */
    CT_IEP0.global_cfg_reg_bit.default_inc = 1;
}
/*
void initializeIntc(void)
{
    // Clear channel map register
    CT_INTC.CH_MAP_REG1 = 0; // 0x20404
    // Clear host interrupt map reg 0
    CT_INTC.HINT_MAP_REG0 = 0;
    // Map IEP event to channel 7/
    CT_INTC.CH_MAP_REG1_bit.CH_MAP_7  = 0x0; // PRU_IEP_EVT   --> Channel 0
    // Map channel to host interrupt
     CT_INTC.HINT_MAP_REG0_bit.HINT_MAP_0 = 0x1;  // Channel 0 --> Host 1
     CT_INTC.HINT_MAP_REG0_bit.HINT_MAP_1 = 0x0;  // Channel 1 --> Host 0

     // Ensure system events are cleared
     CT_INTC.ENA_STATUS_REG0_bit.ENA_STATUS_7 = 1;
     // Enable system events
     CT_INTC.ENABLE_REG0_bit.ENABLE_7 = 1;
     // Enable host interrupts 0 and 1
     CT_INTC.ENABLE_HINT_REG0_bit.ENABLE_HINT_0 = 1;
     CT_INTC.ENABLE_HINT_REG0_bit.ENABLE_HINT_1 = 1;
     // Globally enable host interrupts
     CT_INTC.GLOBAL_ENABLE_HINT_REG_bit.ENABLE_HINT_ANY = 1;
}
*/
/* Init eQEP */
void initQep(void) // sys_pwmss.h
{
    // Set RPM memory to 0
    motorParams.encoderCps = 0;
    // Set RPM memory to 0
    motorParams.encoderCps = 0;
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
//  Get the encoder count/position value
uint32_t getEncoderValue(void)
{
    static uint32_t lastCount = 0;
    uint32_t count;
    int32_t  deltaCount;

    // Check for overrun/overflow errors, show RPM as 0
    count = EQEP1.QPOSCNT_bit.QPOSCNT;
    // Mixed signed and unsigned is used for wraparound when values hit 2^32 counts
    deltaCount = (int)(count - lastCount);
    lastCount = count; // Save the value for next sample to compute the velocity
    deltaCount*= SAMPLES_PER_SEC; // Calculate the velocity
    motorParams.encoderCps= deltaCount;
    return count;
}
#ifdef SIMULATE_MOTOR
/************************************************************************************************************************
* Note" !!!!!! May not work anymore
* Simulator Equations:
*  s  -- Laplace operator
*  Vm -- Voltage applied to motor
*  Tc =  Rm*Jm/(K1*Kt*Kemf)
*  K2 =  Pid_to_Vm/Kemf
*
* motorRevsSec        = (Vm/Kemf)/(S*Rm*Jm/(K1*Kt*K_emf) + 1) where S laplace operator
* motorRevsSec(s*Rm*Jm/(K1*Kt*K_emf) + 1) = (Vm/Kemf)
* motorRevsSec(s*Tc + 1) = (pidOutput*Pid_to_Vm)/Kemf)
* motorRevsSec(s*Tc + 1) = K2*pidOutput
*
* motorRevsSec*s*Tc = K2*pidOutput - motorRevsSec
* motorRevsSecDot*Tc = K2*pidOutput - motorRevsSec
* motorRevsSecDot = (K2*pidOutput - motorRevsSec)/Tc
* motorRevsSecDot = K3*(K2*pidOutput - motorRevsSec)
*
* motorRevsSec+= motorRevsSecDot
* encoderCountsPerSec = K_encoder*motorRevsSec
*
***************************************************************************************************************************/
#define Pid_to_Vm     .006   // Pid PWM output to Vm motor voltage, +/- 12V = 24/4096
#define K_encoder     640.0  // Encoder counts/Revolution
#define K_emf         1.44   // Volts/Revs/sec -- 12Volts/500 rpm
#define K1            .16    // Rad/secs to Revs/secs
#define K2            .0042   // (Pid_to_Vm/K_emf)
#define Kt            16.25  // oz-in/Amp
#define Rm            7.5    // ohms
#define Jm            .02    // Inertia oz-in/sec^2
#define Tc            .04    // Motor load time constant
#define K3            25.0   // 1/Tc

static float motorRevsSecDot = 0.0;
static float motorRevsSec    = 0.0;
static float encoderCountsPerSec = 0.0;

int simulateMotor(void)
{
    motorRevsSecDot = K2*motorParams.pwmOut;
    motorRevsSecDot-= motorRevsSec;
    motorRevsSecDot*= K3;
    motorRevsSec+= Delta_T*motorRevsSecDot;
    encoderCountsPerSec = K_encoder*motorRevsSec;
    return (int)encoderCountsPerSec;
}
#endif
// Handles changes in speeds or stopping avoids limit cycling
void rampUpDown(void)
{
    if(step > 0)
    {
        motorParams.encoderInputCps+= step;
        if(motorParams.encoderInputCps >= targetCps)
        {
            motorParams.encoderInputCps = targetCps;
            if(stopFlag == true)
                motorParams.startStop = 0;
            step = 0;
        }
    }
    else if(step < 0)
    {
        motorParams.encoderInputCps+= step;
        if(motorParams.encoderInputCps <= targetCps)
        {
            motorParams.encoderInputCps = targetCps;
            if(stopFlag == true)
                motorParams.startStop = 0;
            step = 0;
        }
    }
}
// Parses linux system messages, sent in the form of a 2 letter token and possible parameter value
void rpmsgIsr(uint16_t src, uint16_t dst)
{
    switch(rpmsg.cmd)
    {
    case SHARED_MEM_TEST_CMD:
        motorParams.test = rpmsg.value;
        period = rpmsg.value + 100;
        break;
    case START_STOP_CMD:
        if(rpmsg.value == 0)
        {   //Stop command
            stopFlag = true;
            targetCps = 0;
            if(direction == CW)
                step = -5;
            else
                step = 5;
        }
        else
        {
            motorParams.startStop = rpmsg.value;
            stopFlag = false;
        }
        break;
    case ENCODER_INPUT_CPS_CMD:
        if(rpmsg.value > MAX_SPEED)
            rpmsg.value = MAX_SPEED;
        if(rpmsg.value < -MAX_SPEED)
            rpmsg.value = -MAX_SPEED;
        targetCps = rpmsg.value;
        if(targetCps > 0)
        {
            direction = CW;
            if(targetCps > motorParams.encoderInputCps)
                step = 5;
            else if(targetCps < motorParams.encoderInputCps)
                step = -5;
        }
        else
        {
            direction = CCW;
            if(targetCps < motorParams.encoderInputCps)
                step = -5;
            else if(targetCps > motorParams.encoderInputCps)
                step = 5;
        }
        break;
    case KP_CMD:
        motorParams.Kp = rpmsg.value;
        period = 300 + rpmsg.value;
        break;
    case KI_CMD:
        motorParams.Ki = rpmsg.value;
        period = 400 + rpmsg.value ;
        break;
    case KD_CMD:
        motorParams.Kd = rpmsg.value;
        period = 500 + rpmsg.value;
        break;
    case GET_PWM_OUTPUT_CMD:
        rpmsg.value = motorParams.pwmOut;
        break;
    case GET_PID_ERROR_CMD:
        rpmsg.value = motorParams.pidError;
        break;
    case RETURN_KP_VALUE_CMD:
        rpmsg.value = motorParams.Kp;
        break;
    case RETURN_KI_VALUE_CMD:
        rpmsg.value = motorParams.Ki;
        break;
    case RETURN_KD_VALUE_CMD:
        rpmsg.value = motorParams.Kd;
        break;
    case GET_ENCODER_IN_CPS_CMD:
        rpmsg.value = targetCps;
        break;
    case GET_ENCODER_CPS_CMD:
        rpmsg.value = motorParams.encoderCps;
        break;
    default:
        break;
    }
    pru_rpmsg_send(&transport, dst, src, (rpmsg_t *)&rpmsg, sizeof(rpmsg));
}
//  Handles linux system events when rpmsg's are sent
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
/* rpmsg_interrupt not a true vectored interrupt but a polled event from linux host */
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
/*
 * These functions are left here as place holders for for future use if required
bool interruptSvc(void)
{
    if((__R31 & HOST_INT1) == HOST_INT1)
    { // Poll until R31.30 is set
        if(CT_INTC.ENABLE_CLR_REG0_bit.ENABLE_7_CLR) // Verify that the IEP is the source of the interrupt
        {
            CT_IEP0.cmp_status_reg_bit.cmp_status = 1; // Clear status bit
            CT_IEP0.cmp0_reg0 = 250000; // 1ms @ 250MHz
            CT_IEP0.count_reg0 = 0;     // Reload timer register
            CT_INTC.ENA_STATUS_REG0_bit.ENA_STATUS_7; // Action clears __R31 bit 31 flag
            return true;
        }
    }
    return false;
}

void initializeIntc(void)
{
    // Clear channel map register
    CT_INTC.CH_MAP_REG1 = 0; // 0x20404
    // Clear host interrupt map reg 0
    CT_INTC.HINT_MAP_REG0 = 0;
    // Map IEP event to channel 7/
    CT_INTC.CH_MAP_REG1_bit.CH_MAP_7  = 0x0; // PRU_IEP_EVT   --> Channel 0
    // Map channel to host interrupt
     CT_INTC.HINT_MAP_REG0_bit.HINT_MAP_0 = 0x1;  // Channel 0 --> Host 1
     CT_INTC.HINT_MAP_REG0_bit.HINT_MAP_1 = 0x0;  // Channel 1 --> Host 0

     // Ensure system events are cleared
     CT_INTC.ENA_STATUS_REG0_bit.ENA_STATUS_7 = 1;
     // Enable system events
     CT_INTC.ENABLE_REG0_bit.ENABLE_7 = 1;
     // Enable host interrupts 0 and 1
     CT_INTC.ENABLE_HINT_REG0_bit.ENABLE_HINT_0 = 1;
     CT_INTC.ENABLE_HINT_REG0_bit.ENABLE_HINT_1 = 1;
     // Globally enable host interrupts
     CT_INTC.GLOBAL_ENABLE_HINT_REG_bit.ENABLE_HINT_ANY = 1;
}
*/

