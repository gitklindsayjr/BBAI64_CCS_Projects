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
*         |                 |                                -------------                 -------------------
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

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

/* PWM generation definitions*/
#define PERIOD_COUNT               12500 // 250Mhz/20Khz = 12500
#define PWM_OFFSET                 6250  // PERIOD__COUNT/2 -- TIMED output

/* Encoder definitions */
#define MAX_SPEED               5000
#define TICKS_PER_REV           640
#define SAMPLES_PER_SEC         1000
#define SEC_PER_MIN             60
#define CCW                     -1
#define CW                      1
#define NO_DIRECTION            0
#define STOP                    0
#define RUN                     1

/*
 * | b19 b18 b17 b16 | b15 b14 b13 b12 | b11 b10  b9  b8 |  b7  b6  b5  b4 |  b3  b2  b1  b0 |
 * -------------------------------------------------------------------------------------------
 * |  0   0   0   1  |  0   0   0   0  |  0   0   0   0  |  0   0   0   0  |  0   0   0   0  |
 *          1                 0                 0                 0                 0        |
*/
#define DIRECTION_BIT      0x10000

enum mode_ { TIMED, POSITION, AUTO };

/* Shared data structure */
typedef struct
{
    int32_t  encoderCps; // Encoder position counts per second
    int32_t  encoderInputCps;
    uint32_t initFlag;
    int32_t  pwmOut;
    /* PID controls */
    int32_t  Kp, Ki, Kd;
    int32_t  setpoint;
    int32_t  pidError;
    uint32_t startStop;
    int32_t  test;
} motor_params_t;

#endif /* MOTOR_CONTROL_H_ */
