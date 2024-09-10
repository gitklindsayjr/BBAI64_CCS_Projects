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

#ifndef SYS_CTRLMMR_H_
#define SYS_CTRLMMR_H_

#ifdef J721E_TDA4VM // BBAI-64
	#define CTRLMMR_EPWM         0x00104140
	#define CTRLMMR_EQEP         0x001041A0
#endif

#ifdef AM335x
#endif

#ifdef AM64X
#endif

typedef struct
{
    union
    {    // PWM0 Control register
        volatile unsigned EPHRWM0;
        volatile struct
        {
            unsigned TB_CLKEN   : 1; // Enable eHRPWM timebase clock, 0 disabled, 1 enabled
            unsigned rsvrd0     : 3; // 3-1
            unsigned EALLOW     : 1; // Enable write access to ePWM tripzone and HRPWM config registers
            unsigned rsvrd1     : 3; // 7-5
            unsigned SYNCIN_SEL : 3; // Selects the source of the PWM0 synchronization input
            unsigned rsvrd2     : 21;    // 31-11
        } EHRPWM0_bit;
    }; // 00h - 03h
    union
    {   // PWM1 Control register
        volatile unsigned EHRPWM1;
        volatile struct
        {
            unsigned TB_CLKEN :   1; // Enable eHRPWM timebase clock, 0 disabled, 1 enabled
            unsigned rsvrd0     : 3; // 3-1
            unsigned EALLOW     : 1; // Enable write access to ePWM tripzone and HRPWM config registers
            unsigned rsvrd1     : 27; // 31-5
        } EHRPWM1_bit;
    }; //04h-07h, reset: Xh
    union
    {   // PWM2 Control register
        volatile unsigned EHRPWM2;
        volatile struct
        {
            unsigned TB_CLKEN :   1; // Enable eHRPWM timebase clock, 0 disabled, 1 enabled
            unsigned rsvrd0     : 3; // 3-1
            unsigned EALLOW     : 1; // Enable write access to ePWM tripzone and HRPWM config registers
            unsigned rsvrd1     : 27; // 31-5
        } EHRPWM2_bit;
    } ; //08h-0bh, reset: 0h
    union
    {   // PWM3 Control register
        volatile unsigned EHRPWM3;
        volatile struct
        {
            unsigned TB_CLKEN :   1; // Enable eHRPWM timebase clock, 0 disabled, 1 enabled
            unsigned rsvrd0     : 3; // 3-1
            unsigned EALLOW     : 1; // Enable write access to ePWM tripzone and HRPWM config registers
            unsigned rsvrd1     : 3; // 7-5
            unsigned SYNCIN_SEL : 3; // Selects the source of the PWM0 synchronization input
            unsigned rsvrd2 : 21;    // 31-11
        } EHRPWM3_bit;
    }; //0ch-0fh, reset: 0h
    union
    {   // PWM4 Control register
        volatile unsigned EHRPWM4;
        volatile struct
        {
            unsigned TB_CLKEN :   1; // Enable eHRPWM timebase clock, 0 disabled, 1 enabled
            unsigned rsvrd0     : 3; // 3-1
            unsigned EALLOW     : 1; // Enable write access to ePWM tripzone and HRPWM config registers
            unsigned rsvrd1     : 27; // 31-5
        } EHRPWM4_bit;
    }; //10h-13h
    union
    {   // PWM5 Control register
        volatile unsigned EHRPWM5;
        volatile struct
        {
            unsigned TB_CLKEN :   1; // Enable eHRPWM timebase clock, 0 disabled, 1 enabled
            unsigned rsvrd0     : 3; // 3-1
            unsigned EALLOW     : 1; // Enable write access to ePWM tripzone and HRPWM config registers
            unsigned rsvrd1     : 27; // 31-5
        } EHRPWM5_bit;
    }; //14h-17h

} epwm_ctrlmmr_t; //size = 18h
#define EHRPWM_CTRLMMR (*((volatile epwm_ctrlmmr_t *)CTRLMMR_EPWM))

typedef struct
{
    union
    {   // QEP position counter latch register
        volatile unsigned EQEP_STAT;
        volatile struct
        {
            unsigned PHASE_ERR0 : 1; // eQEP0 Phase error status, 0 - No error, 1 - Phase error occurred
            unsigned PHASE_ERR1 : 1; // eQEP1 Phase error status, 0 - No error, 1 - Phase error occurred
            unsigned PHASE_ERR2 : 1; // eQEP2 Phase error status, 0 - No error, 1 - Phase error occurred
            unsigned rsvrd0     : 29;
        } EQEP_STAT_bit;
    }; // 00h-03h
} eqep_ctrlmmr_t; //00h-03h

#define EQEP_CTRLMMR (*((volatile eqep_ctrlmmr_t *)CTRLMMR_EQEP))
#endif
