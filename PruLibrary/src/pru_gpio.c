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
#include "sys_gpio.h"
#include "pru_gpio.h"

// Definitions are for using pointers to gpio registers
//volatile far gpio_t *GPIO0_PTR = (volatile gpio_t *)(GPIO0_OFFSET);
//volatile far gpio_t *GPIO1_PTR = (volatile gpio_t *)(GPIO1_OFFSET);

#ifdef AM335X
void initGpioDebug0_pru0(void)
{
    GPIO1.DATAOUT_bit.DO_bit14 = 0;  // Set Debug pin LO
    GPIO1.DIR_bit.OE_bit14 = 0;       // Set Debug pin as output
}
void scopeDebug0(uint32_t periodUs)
{
    uint32_t i, count;
    count = periodUs/2 ;
    GPIO1.DATAOUT_bit.DO_bit14 = 1;  // Debug signal HI to scope if required
    for(i=0; i<count; i++)
        __delay_cycles(1*PW_CLK_CYCLES);
    GPIO1.DATAOUT_bit.DO_bit14 = 0; // Debug signal LO to scope if required
    for(i=0; i<count; i++)
        __delay_cycles(1*PW_CLK_CYCLES);
}
void initGpioDebug1(void)
{
    GPIO0.DATAOUT_bit.DO_bit26 = 0;  // Set Debug pin LO
    GPIO0.DIR_bit.OE_bit26 = 0;       // Set Debug pin as output
}
void scopeDebug1(uint32_t periodUs)
{
    uint32_t i, count;
    count = periodUs/2;
    GPIO0.DATAOUT_bit.DO_bit26 = 1;  // Debug signal HI to scope if required
    for(i=0; i<count; i++)
        __delay_cycles(1*PW_CLK_CYCLES);
    GPIO0.DATAOUT_bit.DO_bit26 = 0; // Debug signal LO to scope if required
    for(i=0; i<count; i++)
        __delay_cycles(1*PW_CLK_CYCLES);
}
#else
void initScopeDebug0(void)
{
    GPIO0.OUT_DATA23_bit.DO_bit30 = 0;  // Set Debug pin GPIO0_62 LO
    GPIO0.DIR23_bit.OE_bit30 = 0;       // Set Debug pin GPIO0_62 as output
}
void scopeDebug0(uint32_t periodUs)
{
    uint32_t i, count;
    count = periodUs/2 ;
    GPIO0.OUT_DATA23_bit.DO_bit30 = 1;  // Debug signal HI to scope if required
   for(i=0; i<count; i++)
        __delay_cycles(1*PW_CLK_CYCLES);
    GPIO0.OUT_DATA23_bit.DO_bit30 = 0; // Debug signal LO to scope if required
    for(i=0; i<count; i++)
        __delay_cycles(1*PW_CLK_CYCLES);
}
void initScopeDebug1(void)
{
    GPIO0.OUT_DATA45_bit.DO_bit11 = 0;  // Set Debug pin GPIO0_75 LO
    GPIO0.DIR45_bit.OE_bit11 = 0;    // Set Debug pin GPIO0_75 as output
}
void scopeDebug1(uint32_t periodUs)
{
    uint32_t i, count;
    count = periodUs/2;
    GPIO0.OUT_DATA45_bit.DO_bit11 = 1;  // Debug signal HI to scope if required
    for(i=0; i<count; i++)
        __delay_cycles(1*PW_CLK_CYCLES);
    GPIO0.OUT_DATA45_bit.DO_bit11 = 0; // Debug signal LO to scope if required
    for(i=0; i<count; i++)
        __delay_cycles(1*PW_CLK_CYCLES);
}
#endif

