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


#ifndef MOTOR_CNTRL_MSG_H_
#define MOTOR_CNTRL_MSG_H_

#define SIZE_OF_MSG_BUFFER    80

// Form command tokens into uint16_t integers
#define KP_CMD                  (('K'<<8)|'P') // 19280, Proportional gain
#define KD_CMD                  (('K'<<8)|'D') // 19268, Differential gain
#define KI_CMD                  (('K'<<8)|'I') // 19273, Integrator gain
#define START_STOP_CMD          (('S'<<8)|'T') // 21332, Start/Stop: Start = 1, stop = 0
#define ENCODER_INPUT_CPS_CMD   (('E'<<8)|'I') // 17737, Target encoder input counts/sec
#define RETURN_KP_VALUE_CMD     (('R'<<8)|'P') // 21328, Return proportional gain value
#define RETURN_KD_VALUE_CMD     (('R'<<8)|'D') // 21316, Return differential gain value
#define RETURN_KI_VALUE_CMD     (('R'<<8)|'I') // 21321, Return integrator gain value
#define GET_PID_ERROR_CMD       (('G'<<8)|'E') // 18245, Return PID error signal
#define GET_PWM_OUTPUT_CMD      (('G'<<8)|'O') // 18255, Return PID output value
#define GET_ENCODER_IN_CPS_CMD  (('G'<<8)|'I') // 18249, Return Target velocity
#define GET_ENCODER_CPS_CMD     (('G'<<8)|'C') // 18243, Return encoder value
#define SHARED_MEM_TEST_CMD     (('S'<<8)|'M') // Used to set a value in shared memory for testing
#define HELP_CMD                (('H'<<8)|'E') // Display 2 letter command tokens

#define NUM_MSGS       15
char helpMsgs[NUM_MSGS][80] =
{
	"KP xxxx -- Proportional PID gain command",
	"KD xxxx -- Differential PID gain command",
	"KI xxxx -- Integrator   PID gain command",
	"ST n    -- Start n=1, Stop n=0",
	"EI +/-n -- Desired speed, counts/sec, minus (-) for reverse, 5000 max",
	"RP      -- Returns the current Proportional gain",
	"RD      -- Returns the current Differential gain",
	"RI      -- Returns the current Integrator gain",
	"GE      -- Return the PID error signal value",
	"GO      -- Return the PID PWM output signal value",
	"GI      -- Return the speed value +/- counts/sec",
	"GC      -- Return the encoder value +/- counts/sec",
	"HE      -- The help command",
	"SM      -- Shared Memory Test",
	"Q or q  -- To quit this application and stop the motor"
};
/* RPMsg data struct */
typedef struct {
    int cmd;
    int value;
} rpmsg_t ;


#endif /* MOTOR_CNTRL_MSG_H_ */
