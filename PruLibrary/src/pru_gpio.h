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

#include <stdint.h>
#ifndef PRU_GPIO_H_
#define PRU_GPIO_H_

/*
 * | 31 30 29 28 | 27 26 25 24 | 23 22 21 20 | 19 18 17 16 | 15 14 13 12 | 11 10  9  8 |  7  6  5  4 |  3  2  1  0 |
 * -----------------------------------------------------------------------------------------------------------------
 * |  0  0  0  0 |  0  0  0  0 |  0  0  0  0 |  0  0  0  1 |  0  0  0  0 |  0  0  0  0 |  0  0  0  0 |  0  0  0  0 |
 *         0             0             0             1             0             0             0             0
 * |  0  0  0  0 |  0  0  0  0 |  0  0  0  0 |  0  0  0  0 |  0  1  0  0 |  0  0  0  0 |  0  0  0  0 |  0  0  0  0 |
 *         0             0             0             0             4             0             0             0
 * |  0  0  0  0 |  0  0  0  0 |  0  0  0  0 |  0  0  0  0 |  0  0  0  0 |  0  0  0  0 |  0  0  0  0 |  0  0  0  0 |

 */
// PRU Peripheral memory locations, and IO pin definitions refer to am335x_manual.pdf page 180
// From memory mapped GPIO Registers Table 12-127 am64x_tech_ref.pdf page 4981


#ifndef AM335X
    #define CLK_FREQ_MHZ       250
    #define PW_CLK_CYCLES      (CLK_FREQ_MHZ - 11) // Clock cycles adjusted for for loop
#else
    // Beaglebone Black R30 & R31 register pins
    #define R30_P8_11_BIT15          0x00008000
    #define R30_P8_12_BIT14          0x00004000
    #define R31_P8_15_BIT15          0x00008000
    #define P31_P8_16_BIT14          0x00004000
    #define R31_P9_25_BIT7           0x00000080
    #define R31_P9_27_BIT5           0x00000020
    #define R31_P9_28_BIT3           0x00000008
    #define R31_P9_29_BIT1           0x00000002
    #define R31_P9_30_BIT2           0x00000004
    #define R31_P9_31_BIT0           0x00000001
    #define CLK_FREQ_MHZ       250
    #define PW_CLK_CYCLES      (CLK_FREQ_MHZ - 11) // Clock cycles adjusted for for loop
#endif

void initScopeDebug0(void);
void scopeDebug0(uint32_t periodUs);
void initScopeDebug1(void);
void scopeDebug1(uint32_t periodUs);

#endif /* PRU_GPIO_H_ */
