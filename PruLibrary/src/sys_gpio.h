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

/* Declare gpio structure examples
  * volatile far gpio *GPIO0 = (volatile gpio *)(GPIO0_OFFSET);
  * volatile far gpio *GPIO1 = (volatile gpio *)(GPIO1_OFFSET);
  * volatile far gpio *MCU_GPIO = (volatile gpio *)(MCU_GPIO_OFFSET);
  */

// From memory mapped GPIO Registers Table 25-5 am335x_manual.pdf page 4877
#ifndef SYS_GPIO_H_
#define SYS_GPIO_H

#ifdef AM335X
    #define CLKFREQ            200u
    #define SCOPE_DEBUG_FREQ   (CLKFREQ - 18)
    #define GPIO0_OFFSET       0x44E07000
    #define GPIO1_OFFSET       0x4804C000
    #define GPIO2_OFFSET       0x481AC000
    #define GPIO3_OFFSET       0x481AE000
typedef struct
{
  union {
    volatile unsigned REVISION;
    volatile struct {
      unsigned MINOR : 6;
      unsigned CUSTOM: 2;
      unsigned MAJOR: 3;
      unsigned RTL: 5;
      unsigned FUNC: 12;
      unsigned rsrvd: 2;
      unsigned SCHEME: 2;
    } REVISION_bit;
  } ; //0h - 3h, reset: 50600801h
  unsigned rsrvd0[3]; //4h-fh
  union{
    volatile unsigned SYSCONFIG;
    volatile struct{
      unsigned AUTOIDLE: 1;
      unsigned SOFTRESET: 1;
      unsigned ENAWAKEUP: 1;
      unsigned IDLEMODE: 2;
      unsigned rsrvd: 27;
    } SYSCONFIG_bit;
  } ;//10h-13h, reset: 0h
  unsigned rsrvd1[3]; //13h-1fh
  union{
    volatile unsigned EOI;
    volatile struct{
      unsigned DMAEVENT_Ack: 1;
      unsigned rsrvd: 31;
    } EOI_bit;
  } ;//20h-23h, reset: 0h
  volatile unsigned IRQSTATUS_RAW_0; //24h-27h, reset: 0h
  volatile unsigned IRQSTATUS_RAW_1; //28h-2Bh, reset: 0h
  volatile unsigned IRQSTATUS_0; //2Ch-2Fh, reset: 0h
  volatile unsigned IRQSTATUS_1; //30h-33h, reset: 0h
  volatile unsigned IRQSTATUS_SET_0; //34h-37h, reset: 0h
  volatile unsigned IRQSTATUS_SET_1; //38h-3Bh, reset: 0h
  volatile unsigned IRQSTATUS_CLR_0; //3Ch-3Fh, reset: 0h
  volatile unsigned IRQSTATUS_CLR_1; //40h-43h, reset: 0h
  volatile unsigned IRQWAKEN_0; //44h-47h, reset: 0h
  volatile unsigned IRQWAKEN_1; //48h-4Bh, reset: 0h
  unsigned rsrvd2[0x32]; //4Ch-113h
  union{
    volatile unsigned SYSSTATUS;
    volatile struct{
      unsigned RESETDONE: 1;
      unsigned rsrvd: 31;
    } SYSSTATUS_bit;
  } ; //114h-117h
  unsigned rsrvd3[0x6]; //118h-12fh
  union{
    volatile unsigned CTRL;
    volatile struct {
      unsigned DISABLEMODULE: 1;
      unsigned GATINGRATIO: 2;
      unsigned rsrvd: 29;
    } CTRL_bit;
  } ; //130h-133h
  union{
    volatile unsigned DIR;
    volatile struct{
      unsigned OE_bit0: 1; // 0
      unsigned OE_bit1: 1; // 1
      unsigned OE_bit2: 1; // 2
      unsigned OE_bit3: 1; // 3

      unsigned OE_bit4: 1; // 2
      unsigned OE_bit5: 1; // 5
      unsigned OE_bit6: 1; // 6
      unsigned OE_bit7: 1; // 7

      unsigned OE_bit8: 1;
      unsigned OE_bit9: 1;
      unsigned OE_bit10: 1; // 10
      unsigned OE_bit11: 1; // 11

      unsigned OE_bit12: 1; // 12
      unsigned OE_bit13: 1; // 13
      unsigned OE_bit14: 1; // 14
      unsigned OE_bit15: 1; // 15

      unsigned OE_bit16: 1;
      unsigned OE_bit17: 1;
      unsigned OE_bit18: 1;
      unsigned OE_bit19: 1;

      unsigned OE_bit20: 1;
      unsigned OE_bit21: 1;
      unsigned OE_bit22: 1;
      unsigned OE_bit23: 1;

      unsigned OE_bit24: 1;
      unsigned OE_bit25: 1;
      unsigned OE_bit26: 1;
      unsigned OE_bit27: 1;

      unsigned OE_bit28: 1;
      unsigned OE_bit29: 1;
      unsigned OE_bit30: 1;
      unsigned OE_bit31: 1;
    } DIR_bit;
  }; //134h-137h, reset: ffffffffh
  union{
    volatile unsigned DATAIN;
    volatile struct{
      unsigned DI_bit0: 1;
      unsigned DI_bit1: 1;
      unsigned DI_bit2: 1;
      unsigned DI_bit3: 1;

      unsigned DI_bit4: 1;
      unsigned DI_bit5: 1;
      unsigned DI_bit6: 1;
      unsigned DI_bit7: 1;

      unsigned DI_bit8: 1;
      unsigned DI_bit9: 1;
      unsigned DI_bit10: 1;
      unsigned DI_bit11: 1;

      unsigned DI_bit12: 1;
      unsigned DI_bit13: 1;
      unsigned DI_bit14: 1;
      unsigned DI_bit15: 1;

      unsigned DI_bit16: 1;
      unsigned DI_bit17: 1;
      unsigned DI_bit18: 1;
      unsigned DI_bit19: 1;

      unsigned DI_bit20: 1;
      unsigned DI_bit21: 1;
      unsigned DI_bit22: 1;
      unsigned DI_bit23: 1;

      unsigned DI_bit24: 1;
      unsigned DI_bit25: 1;
      unsigned DI_bit26: 1;
      unsigned DI_bit27: 1;

      unsigned DI_bit28: 1;
      unsigned DI_bit29: 1;
      unsigned DI_bit30: 1;
      unsigned DI_bit31: 1;
    } DATAIN_bit;
  };//138h-13Bh, reset: 0h
  union{
      volatile unsigned DATAOUT; //13Ch-13Fh, reset: 0h
      volatile struct{
        unsigned DO_bit0: 1;
        unsigned DO_bit1: 1;
        unsigned DO_bit2: 1;
        unsigned DO_bit3: 1;

        unsigned DO_bit4: 1;
        unsigned DO_bit5: 1;
        unsigned DO_bit6: 1;
        unsigned DO_bit7: 1;

        unsigned DO_bit8: 1;
        unsigned DO_bit9: 1;
        unsigned DO_bit10: 1;
        unsigned DO_bit11: 1;

        unsigned DO_bit12: 1;
        unsigned DO_bit13: 1;
        unsigned DO_bit14: 1;
        unsigned DO_bit15: 1;

        unsigned DO_bit16: 1;
        unsigned DO_bit17: 1;
        unsigned DO_bit18: 1;
        unsigned DO_bit19: 1;

        unsigned DO_bit20: 1;
        unsigned DO_bit21: 1;
        unsigned DO_bit22: 1;
        unsigned DO_bit23: 1;

        unsigned DO_bit24: 1;
        unsigned DO_bit25: 1;
        unsigned DO_bit26: 1;
        unsigned DO_bit27: 1;

        unsigned DO_bit28: 1;
        unsigned DO_bit29: 1;
        unsigned DO_bit30: 1;
        unsigned DO_bit31: 1;
      } DATAOUT_bit;
  };
  volatile unsigned LEVELDETECT0; //140h, reset: 0h
  volatile unsigned LEVELDETECT1; //144h, reset: 0h
  volatile unsigned RISINGDETECT; //148h-14Bh, reset: 0h
  volatile unsigned FALLINGDETECT; //14ch-14fh, reset: 0h
  volatile unsigned DEBOUNCENABLE; //150h-153h, reset: 0h
    union{
        volatile unsigned DEBOUNCINGTIME;
        volatile struct{
            unsigned DEBOUNCETIME: 8;
            unsigned rsrvd: 24;
        } DEBOUNCINGTIME_bit;
    } ;//154h-157h
    unsigned rsrvd4[0xE];//158h-18fh
    union{
        volatile unsigned CLEAR;
        volatile struct{
            unsigned CD_bit0: 1;
            unsigned CD_bit1: 1;
            unsigned CD_bit2: 1;
            unsigned CD_bit3: 1;

            unsigned CD_bit4: 1;
            unsigned CD_bit5: 1;
            unsigned CD_bit6: 1;
            unsigned CD_bit7: 1;

            unsigned CD_bit8: 1;
            unsigned CD_bit9: 1;
            unsigned CD_bit10: 1;
            unsigned CD_bit11: 1;

            unsigned CD_bit12: 1;
            unsigned CD_bit13: 1;
            unsigned CD_bit14: 1;
            unsigned CD_bit15: 1;

            unsigned CD_bit16: 1;
            unsigned CD_bit17: 1;
            unsigned CD_bit18: 1;
            unsigned CD_bit19: 1;

            unsigned CD_bit20: 1;
            unsigned CD_bit21: 1;
            unsigned CD_bit22: 1;
            unsigned CD_bit23: 1;

            unsigned CD_bit24: 1;
            unsigned CD_bit25: 1;
            unsigned CD_bit26: 1;
            unsigned CD_bit27: 1;

            unsigned CD_bit28: 1;
            unsigned CD_bit29: 1;
            unsigned CD_bit30: 1;
            unsigned CD_bit31: 1;
        } CD_bit;
    }; //190h-193h

    union{
        volatile unsigned SET;
        volatile struct{
            unsigned SD_bit0: 1;
            unsigned SD_bit1: 1;
            unsigned SD_bit2: 1;
            unsigned SD_bit3: 1;

            unsigned SD_bit4: 1;
            unsigned SD_bit5: 1;
            unsigned SD_bit6: 1;
            unsigned SD_bit7: 1;

            unsigned SD_bit8: 1;
            unsigned SD_bit9: 1;
            unsigned SD_bit10: 1;
            unsigned SD_bit11: 1;

            unsigned SD_bit12: 1;
            unsigned SD_bit13: 1;
            unsigned SD_bit14: 1;
            unsigned SD_bit15: 1;

            unsigned SD_bit16: 1;
            unsigned SD_bit17: 1;
            unsigned SD_bit18: 1;
            unsigned SD_bit19: 1;

            unsigned SD_bit20: 1;
            unsigned SD_bit21: 1;
            unsigned SD_bit22: 1;
            unsigned SD_bit23: 1;

            unsigned SD_bit24: 1;
            unsigned SD_bit25: 1;
            unsigned SD_bit26: 1;
            unsigned SD_bit27: 1;

            unsigned SD_bit28: 1;
            unsigned SD_bit29: 1;
            unsigned SD_bit30: 1;
            unsigned SD_bit31: 1;
        } SD_bit;
    }; //194h-197h
} gpio_t; //size = 198h
#define GPIO0 (*((volatile gpio_t *)GPIO0_OFFSET))
#define GPIO1 (*((volatile gpio_t *)GPIO1_OFFSET))
#endif
#ifdef J721E_TDA4VM
    #define CLKFREQ                  250u
    #define SCOPE_DEBUG_FREQ         (CLKFREQ - 18)
    #define GPIO0_OFFSET              0x00600000
    #define GPIO1_OFFSET              0x00601000
    #define GPIO2_OFFSET              0x00610000
    #define GPIO3_OFFSET              0x00611000
#endif
#ifdef AM64X // SK-AM64B
    #define GPIO0_OFFSET              0x00600000
    #define GPIO1_OFFSET              0x00601000
    #define MCU_GPIO_OFFSET           0x04201000
#endif
typedef struct
{
    union
    {
        volatile unsigned GPIO_PID;
        volatile struct
        {
            unsigned MINOR : 6;
            unsigned CUSTOM: 2;
            unsigned MAJOR: 3;
            unsigned RTL: 5;
            unsigned FUNC: 12;
            unsigned rsrvd: 2;
            unsigned SCHEME: 2;
        } GPIO_PID_bit;
    } ; //0h - 3h - reset 44832905h
    union
    {
        volatile unsigned PCR;
        volatile struct
        {
            unsigned FREE: 1;
            unsigned SOFT: 1;
            unsigned rsrvd: 30;
        } PCR_bit;
    } ; //4h-7h, reset: Xh
    union
    {
        volatile unsigned BINTEN;
        volatile struct
        {
            unsigned EN: 16;
            unsigned rsrvd: 16;
        } BINTEN_bit;
    } ; //8h-bh, reset: 0h
    unsigned rsrvd0; //ch-fh
    union
    {
        volatile unsigned DIR01;
        volatile struct
        {
            unsigned OE_bit0: 1; // 0
            unsigned OE_bit1: 1; // 1
            unsigned OE_bit2: 1; // 2
            unsigned OE_bit3: 1; // 3

            unsigned OE_bit4: 1; // 2
            unsigned OE_bit5: 1; // 5
            unsigned OE_bit6: 1; // 6
            unsigned OE_bit7: 1; // 7

            unsigned OE_bit8: 1;
            unsigned OE_bit9: 1;
            unsigned OE_bit10: 1; // 10
            unsigned OE_bit11: 1; // 11

            unsigned OE_bit12: 1; // 12
            unsigned OE_bit13: 1; // 13
            unsigned OE_bit14: 1; // 14
            unsigned OE_bit15: 1; // 15

            unsigned OE_bit16: 1; // 16
            unsigned OE_bit17: 1; // 17
            unsigned OE_bit18: 1; // 18
            unsigned OE_bit19: 1; // 19

            unsigned OE_bit20: 1; // 20
            unsigned OE_bit21: 1; // 21
            unsigned OE_bit22: 1; // 22
            unsigned OE_bit23: 1; // 23

            unsigned OE_bit24: 1; // 24
            unsigned OE_bit25: 1; // 25
            unsigned OE_bit26: 1; // 26
            unsigned OE_bit27: 1; //27

            unsigned OE_bit28: 1; // 28
            unsigned OE_bit29: 1; // 29
            unsigned OE_bit30: 1; // 30
            unsigned OE_bit31: 1; // 31
        } DIR01_bit;
    }; //10h-13h, reset: ffffffffh
    union
    {
        volatile unsigned OUT_DATA01;
        volatile struct
        {
            unsigned DO_bit0: 1; // 0
            unsigned DO_bit1: 1; // 1
            unsigned DO_bit2: 1; // 2
            unsigned DO_bit3: 1; // 3

            unsigned DO_bit4: 1; // 4
            unsigned DO_bit5: 1; // 5
            unsigned DO_bit6: 1; // 6
            unsigned DO_bit7: 1; // 7

            unsigned DO_bit8: 1; // 8
            unsigned DO_bit9: 1; // 9
            unsigned DO_bit10: 1; // 10
            unsigned DO_bit11: 1; // 11

            unsigned DO_bit12: 1; // 12
            unsigned DO_bit13: 1; // 13
            unsigned DO_bit14: 1; // 14
            unsigned DO_bit15: 1; // 15

            unsigned DO_bit16: 1; // 16
            unsigned DO_bit17: 1; // 17
            unsigned DO_bit18: 1; // 18
            unsigned DO_bit19: 1; // 19

            unsigned DO_bit20: 1; // 20
            unsigned DO_bit21: 1; // 21
            unsigned DO_bit22: 1; // 22
            unsigned DO_bit23: 1; // 23

            unsigned DO_bit24: 1; // 24
            unsigned DO_bit25: 1; // 25
            unsigned DO_bit26: 1; // 26
            unsigned DO_bit27: 1; // 27
      
            unsigned DO_bit28: 1; // 28
            unsigned DO_bit29: 1; // 29
            unsigned DO_bit30: 1; // 30
            unsigned DO_bit31: 1; // 31
        } OUT_DATA01_bit;
    };//14h-17h, reset: 0h
    union
    {
        volatile unsigned SET_DATA01;
        volatile struct
        {
            unsigned SD_bit0: 1; // 0
            unsigned SD_bit1: 1; // 1
            unsigned SD_bit2: 1; // 2
            unsigned SD_bit3: 1; // 3

            unsigned SD_bit4: 1; // 4
            unsigned SD_bit5: 1; // 5
            unsigned SD_bit6: 1; // 6
            unsigned SD_bit7: 1; // 7

            unsigned SD_bit8: 1; // 8
            unsigned SD_bit9: 1; // 9
            unsigned SD_bit10: 1; // 10
            unsigned SD_bit11: 1; // 11

            unsigned SD_bit12: 1; // 12
            unsigned SD_bit13: 1; // 13
            unsigned SD_bit14: 1; // 14
            unsigned SD_bit15: 1; // 15

            unsigned SD_bit16: 1; // 16
            unsigned SD_bit17: 1; // 17
            unsigned SD_bit18: 1; // 18
            unsigned SD_bit19: 1; // 19

            unsigned SD_bit20: 1; // 20
            unsigned SD_bit21: 1; // 21
            unsigned SD_bit22: 1; // 22
            unsigned SD_bit23: 1; // 23

            unsigned SD_bit24: 1; // 24
            unsigned SD_bit25: 1; // 25
            unsigned SD_bit26: 1; // 26
            unsigned SD_bit27: 1; // 27

            unsigned SD_bit28: 1; // 28
            unsigned SD_bit29: 1; // 29
            unsigned SD_bit30: 1; // 30
            unsigned SD_bit31: 1; // 31
        } SET_DATA01_bit;
    }; //18h-1Bh
    union
    {
        volatile unsigned CLR_DATA01;
        volatile struct
        {
            unsigned CD_bit0: 1; // 0
            unsigned CD_bit1: 1; // 1
            unsigned CD_bit2: 1; // 2
            unsigned CD_bit3: 1; // 3

            unsigned CD_bit4: 1; // 4
            unsigned CD_bit5: 1; // 5
            unsigned CD_bit6: 1; // 6
            unsigned CD_bit7: 1; // 7

            unsigned CD_bit8: 1; // 9
            unsigned CD_bit9: 1; // 9
            unsigned CD_bit10: 1; // 10
            unsigned CD_bit11: 1; // 11

            unsigned CD_bit12: 1; // 12
            unsigned CD_bit13: 1; // 13
            unsigned CD_bit14: 1; // 14
            unsigned CD_bit15: 1; // 15

            unsigned CD_bit16: 1; // 16
            unsigned CD_bit17: 1; // 17
            unsigned CD_bit18: 1; // 18
            unsigned CD_bit19: 1; // 19

            unsigned CD_bit20: 1; // 20
            unsigned CD_bit21: 1; // 21
            unsigned CD_bit22: 1; // 22
            unsigned CD_bit23: 1; // 23

            unsigned CD_bit24: 1; // 24
            unsigned CD_bit25: 1; // 25
            unsigned CD_bit26: 1; // 26
            unsigned CD_bit27: 1; // 27

            unsigned CD_bit28: 1; // 28
            unsigned CD_bit29: 1; // 29
            unsigned CD_bit30: 1; // 30
            unsigned CD_bit31: 1; // 31
        } CLR_DATA01_bit;
    }; //1Ch-1Fh
    union
    {
        volatile unsigned IN_DATA01;
        volatile struct
        {
            unsigned DI_bit0: 1; // 0
            unsigned DI_bit1: 1; // 1
            unsigned DI_bit2: 1; // 2
            unsigned DI_bit3: 1; // 3

            unsigned DI_bit4: 1; // 4
            unsigned DI_bit5: 1; // 5
            unsigned DI_bit6: 1; // 6
            unsigned DI_bit7: 1; // 7
            unsigned DI_bit8: 1; // 8
            unsigned DI_bit9: 1; // 9
            unsigned DI_bit10: 1; // 10
            unsigned DI_bit11: 1; // 11

            unsigned DI_bit12: 1; // 12
            unsigned DI_bit13: 1; // 13
            unsigned DI_bit14: 1; // 14
            unsigned DI_bit15: 1; // 15

            unsigned DI_bit16: 1; // 16
            unsigned DI_bit17: 1; // 17
            unsigned DI_bit18: 1; // 18
            unsigned DI_bit19: 1; // 19

            unsigned DI_bit20: 1; // 20
            unsigned DI_bit21: 1; // 21
            unsigned DI_bit22: 1; // 22
            unsigned DI_bit23: 1; // 23

            unsigned DI_bit24: 1; // 24
            unsigned DI_bit25: 1; // 25
            unsigned DI_bit26: 1; // 26
            unsigned DI_bit27: 1; // 27

            unsigned DI_bit28: 1; // 28
            unsigned DI_bit29: 1; // 29
            unsigned DI_bit30: 1; // 30
            unsigned DI_bit31: 1; // 31
        } IN_DATA01_bit;
    };//20h-23h, reset: 0h
    volatile unsigned SET_RIS_TRIG01; //24h-27h, reset: 0h
    volatile unsigned CLR_RIS_TRIG01; //28ch-2Bh, reset: 0h
    volatile unsigned SET_FALL_TRIG01; //2Ch-2Fh, reset: 0h
    volatile unsigned CLR_FALL_TRIG01; //30h-33fh, reset: 0h
    volatile unsigned INTSTAT01; // 34h-37h
    union
    {
        volatile unsigned DIR23;
        volatile struct
        {
            unsigned OE_bit0: 1;  // 32
            unsigned OE_bit1: 1;  // 33
            unsigned OE_bit2: 1;  // 34
            unsigned OE_bit3: 1;  // 35

            unsigned OE_bit4: 1;  // 36
            unsigned OE_bit5: 1;  // 37
            unsigned OE_bit6: 1;  // 38
            unsigned OE_bit7: 1;  // 39

            unsigned OE_bit8: 1;  // 40
            unsigned OE_bit9: 1;  // 41
            unsigned OE_bit10: 1; // 42
            unsigned OE_bit11: 1; // 43

            unsigned OE_bit12: 1; // 44
            unsigned OE_bit13: 1; // 45
            unsigned OE_bit14: 1; // 46
            unsigned OE_bit15: 1; // 47

            unsigned OE_bit16: 1; // 48
            unsigned OE_bit17: 1; // 49
            unsigned OE_bit18: 1; // 50
            unsigned OE_bit19: 1; // 51

            unsigned OE_bit20: 1; // 52
            unsigned OE_bit21: 1; // 53
            unsigned OE_bit22: 1; // 54
            unsigned OE_bit23: 1; // 55

            unsigned OE_bit24: 1; // 56
            unsigned OE_bit25: 1; // 57
            unsigned OE_bit26: 1; // 58
            unsigned OE_bit27: 1; // 59

            unsigned OE_bit28: 1; // 60
            unsigned OE_bit29: 1; // 61
            unsigned OE_bit30: 1; // 62
            unsigned OE_bit31: 1; // 63
        } DIR23_bit;
    }; //38h-3bh, reset: ffffffffh
    union
    {
        volatile unsigned OUT_DATA23;
        volatile struct
        {
            unsigned DO_bit0: 1; // 32
            unsigned DO_bit1: 1; // 33
            unsigned DO_bit2: 1; // 34
            unsigned DO_bit3: 1; // 35

            unsigned DO_bit4: 1; // 36
            unsigned DO_bit5: 1; // 37
            unsigned DO_bit6: 1; // 38
            unsigned DO_bit7: 1; // 39

            unsigned DO_bit8: 1; // 40
            unsigned DO_bit9: 1; // 41
            unsigned DO_bit10: 1; // 42
            unsigned DO_bit11: 1; // 43

            unsigned DO_bit12: 1; // 44
            unsigned DO_bit13: 1; // 45
            unsigned DO_bit14: 1; // 46
            unsigned DO_bit15: 1; // 47

            unsigned DO_bit16: 1; // 48
            unsigned DO_bit17: 1; // 49
            unsigned DO_bit18: 1; // 50
            unsigned DO_bit19: 1; // 51

            unsigned DO_bit20: 1; // 52
            unsigned DO_bit21: 1; // 53
            unsigned DO_bit22: 1; // 54
            unsigned DO_bit23: 1; // 55

            unsigned DO_bit24: 1; // 56
            unsigned DO_bit25: 1; // 57
            unsigned DO_bit26: 1; // 58
            unsigned DO_bit27: 1; // 59

            unsigned DO_bit28: 1; // 60
            unsigned DO_bit29: 1; // 61
            unsigned DO_bit30: 1; // 62
            unsigned DO_bit31: 1; // 63
        } OUT_DATA23_bit;
    };// 3ch-3fh, reset: 0h
    union
    {
        volatile unsigned SET_DATA23;
        volatile struct
        {
            unsigned SD_bit0: 1; // 32
            unsigned SD_bit1: 1; // 33
            unsigned SD_bit2: 1; // 34
            unsigned SD_bit3: 1; // 35

            unsigned SD_bit4: 1; // 36
            unsigned SD_bit5: 1; // 37
            unsigned SD_bit6: 1; // 38
            unsigned SD_bit7: 1; // 39

            unsigned SD_bit8: 1; // 40
            unsigned SD_bit9: 1; // 41
            unsigned SD_bit10: 1; // 42
            unsigned SD_bit11: 1; // 43

            unsigned SD_bit12: 1; // 44
            unsigned SD_bit13: 1; // 45
            unsigned SD_bit14: 1; // 46
            unsigned SD_bit15: 1; // 47

            unsigned SD_bit16: 1; //48
            unsigned SD_bit17: 1; // 49
            unsigned SD_bit18: 1; // 50
            unsigned SD_bit19: 1; // 51

            unsigned SD_bit20: 1; // 52
            unsigned SD_bit21: 1; // 53
            unsigned SD_bit22: 1; // 54
            unsigned SD_bit23: 1; // 55

            unsigned SD_bit24: 1; // 56
            unsigned SD_bit25: 1; // 57
            unsigned SD_bit26: 1; // 58
            unsigned SD_bit27: 1; // 59

            unsigned SD_bit28: 1; // 60
            unsigned SD_bit29: 1; // 61
            unsigned SD_bit30: 1; // 62
            unsigned SD_bit31: 1; // 63
        } SET_DATA23_bit;
    }; // 40h-43h
    union
    {
        volatile unsigned CLR_DATA23;
        volatile struct
        {
            unsigned CD_bit0: 1; // 32
            unsigned CD_bit1: 1; // 33
            unsigned CD_bit2: 1; // 34
            unsigned CD_bit3: 1; // 35

            unsigned CD_bit4: 1;
            unsigned CD_bit5: 1;
            unsigned CD_bit6: 1;
            unsigned CD_bit7: 1;

            unsigned CD_bit8: 1;
            unsigned CD_bit9: 1;
            unsigned CD_bit10: 1;
            unsigned CD_bit11: 1;

            unsigned CD_bit12: 1;
            unsigned CD_bit13: 1;
            unsigned CD_bit14: 1;
            unsigned CD_bit15: 1;

            unsigned CD_bit16: 1;
            unsigned CD_bit17: 1;
            unsigned CD_bit18: 1;
            unsigned CD_bit19: 1;

            unsigned CD_bit20: 1;
            unsigned CD_bit21: 1;
            unsigned CD_bit22: 1;
            unsigned CD_bit23: 1;

            unsigned CD_bit24: 1;
            unsigned CD_bit25: 1;
            unsigned CD_bit26: 1;
            unsigned CD_bit27: 1;

            unsigned CD_bit28: 1;
            unsigned CD_bit29: 1;
            unsigned CD_bit30: 1;
            unsigned CD_bit31: 1;
        } CLR_DATA23_bit;
    }; // 44h-47h
    union
    {
        volatile unsigned IN_DATA23;
        volatile struct
        {
            unsigned DI_bit0: 1; // 32
            unsigned DI_bit1: 1; // 33
            unsigned DI_bit2: 1; // 34
            unsigned DI_bit3: 1; // 35

            unsigned DI_bit4: 1;
            unsigned DI_bit5: 1;
            unsigned DI_bit6: 1;
            unsigned DI_bit7: 1;

            unsigned DI_bit8: 1;
            unsigned DI_bit9: 1;
            unsigned DI_bit10: 1;
            unsigned DI_bit11: 1;

            unsigned DI_bit12: 1;
            unsigned DI_bit13: 1;
            unsigned DI_bit14: 1;
            unsigned DI_bit15: 1;

            unsigned DI_bit16: 1;
            unsigned DI_bit17: 1;
            unsigned DI_bit18: 1;
            unsigned DI_bit19: 1;

            unsigned DI_bit20: 1;
            unsigned DI_bit21: 1;
            unsigned DI_bit22: 1;
            unsigned DI_bit23: 1;

            unsigned DI_bit24: 1;
            unsigned DI_bit25: 1;
            unsigned DI_bit26: 1;
            unsigned DI_bit27: 1;

            unsigned DI_bit28: 1;
            unsigned DI_bit29: 1;
            unsigned DI_bit30: 1;
            unsigned DI_bit31: 1;
        } IN_DATA23_bit;
    };// 48h-4bh, reset: 0h
    volatile unsigned SET_RIS_TRIG23; // 4ch-4fh, reset: 0h
    volatile unsigned CLR_RIS_TRIG23; // 50h-53h, reset: 0h
    volatile unsigned SET_FALL_TRIG23; //54h-57h, reset: 0h
    volatile unsigned CLR_FALL_TRIG23; //58h-5bh, reset: 0h
    volatile unsigned INTSTAT23; // 5ch-5fh
    union
    {
        volatile unsigned DIR45;
        volatile struct
        {
            unsigned OE_bit0: 1; // 64
            unsigned OE_bit1: 1; // 65
            unsigned OE_bit2: 1; // 66
            unsigned OE_bit3: 1; // 67

            unsigned OE_bit4: 1; // 68
            unsigned OE_bit5: 1; // 69
            unsigned OE_bit6: 1; // 70
            unsigned OE_bit7: 1; // 71

            unsigned OE_bit8: 1; // 72
            unsigned OE_bit9: 1; // 73
            unsigned OE_bit10: 1; // 74
            unsigned OE_bit11: 1; // 75

            unsigned OE_bit12: 1; // 76
            unsigned OE_bit13: 1; // 77
            unsigned OE_bit14: 1; // 78
            unsigned OE_bit15: 1; // 79

            unsigned OE_bit16: 1; // 80
            unsigned OE_bit17: 1; // 81
            unsigned OE_bit18: 1; // 82
            unsigned OE_bit19: 1; // 83

            unsigned OE_bit20: 1; // 84
            unsigned OE_bit21: 1; // 85
            unsigned OE_bit22: 1; // 86
            unsigned OE_bit23: 1; // 87

            unsigned OE_bit24: 1; // 88
            unsigned OE_bit25: 1; // 89
            unsigned OE_bit26: 1; // 90
            unsigned OE_bit27: 1; // 91

            unsigned OE_bit28: 1; // 92
            unsigned OE_bit29: 1; // 93
            unsigned OE_bit30: 1; // 94
            unsigned OE_bit31: 1; // 95
        } DIR45_bit;
    }; //38h-3bh, reset: ffffffffh
    union
    {
        volatile unsigned OUT_DATA45;
        volatile struct
        {
            unsigned DO_bit0: 1;
            unsigned DO_bit1: 1;
            unsigned DO_bit2: 1;
            unsigned DO_bit3: 1;

            unsigned DO_bit4: 1;
            unsigned DO_bit5: 1;
            unsigned DO_bit6: 1;
            unsigned DO_bit7: 1;

            unsigned DO_bit8: 1;
            unsigned DO_bit9: 1;
            unsigned DO_bit10: 1;
            unsigned DO_bit11: 1;

            unsigned DO_bit12: 1;
            unsigned DO_bit13: 1;
            unsigned DO_bit14: 1;
            unsigned DO_bit15: 1;

            unsigned DO_bit16: 1;
            unsigned DO_bit17: 1;
            unsigned DO_bit18: 1;
            unsigned DO_bit19: 1;

            unsigned DO_bit20: 1;
            unsigned DO_bit21: 1;
            unsigned DO_bit22: 1;
            unsigned DO_bit23: 1;

            unsigned DO_bit24: 1;
            unsigned DO_bit25: 1;
            unsigned DO_bit26: 1;
            unsigned DO_bit27: 1;

            unsigned DO_bit28: 1;
            unsigned DO_bit29: 1;
            unsigned DO_bit30: 1;
            unsigned DO_bit31: 1;
        } OUT_DATA45_bit;
    };// 3ch-3fh, reset: 0h
    union
    {
        volatile unsigned SET_DATA45;
        volatile struct
        {
            unsigned SD_bit0: 1;
            unsigned SD_bit1: 1;
            unsigned SD_bit2: 1;
            unsigned SD_bit3: 1;

            unsigned SD_bit4: 1;
            unsigned SD_bit5: 1;
            unsigned SD_bit6: 1;
            unsigned SD_bit7: 1;

            unsigned SD_bit8: 1;
            unsigned SD_bit9: 1;
            unsigned SD_bit10: 1;
            unsigned SD_bit11: 1;

            unsigned SD_bit12: 1;
            unsigned SD_bit13: 1;
            unsigned SD_bit14: 1;
            unsigned SD_bit15: 1;

            unsigned SD_bit16: 1;
            unsigned SD_bit17: 1;
            unsigned SD_bit18: 1;
            unsigned SD_bit19: 1;

            unsigned SD_bit20: 1;
            unsigned SD_bit21: 1;
            unsigned SD_bit22: 1;
            unsigned SD_bit23: 1;

            unsigned SD_bit24: 1;
            unsigned SD_bit25: 1;
            unsigned SD_bit26: 1;
            unsigned SD_bit27: 1;

            unsigned SD_bit28: 1;
            unsigned SD_bit29: 1;
            unsigned SD_bit30: 1;
            unsigned SD_bit31: 1;
        } SET_DATA45_bit;
    }; // 40h-43h
    union
    {
        volatile unsigned CLR_DATA45;
        volatile struct
        {
            unsigned CD_bit0: 1;
            unsigned CD_bit1: 1;
            unsigned CD_bit2: 1;
            unsigned CD_bit3: 1;

            unsigned CD_bit4: 1;
            unsigned CD_bit5: 1;
            unsigned CD_bit6: 1;
            unsigned CD_bit7: 1;

            unsigned CD_bit8: 1;
            unsigned CD_bit9: 1;
            unsigned CD_bit10: 1;
            unsigned CD_bit11: 1;

            unsigned CD_bit12: 1;
            unsigned CD_bit13: 1;
            unsigned CD_bit14: 1;
            unsigned CD_bit15: 1;

            unsigned CD_bit16: 1;
            unsigned CD_bit17: 1;
            unsigned CD_bit18: 1;
            unsigned CD_bit19: 1;

            unsigned CD_bit20: 1;
            unsigned CD_bit21: 1;
            unsigned CD_bit22: 1;
            unsigned CD_bit23: 1;

            unsigned CD_bit24: 1;
            unsigned CD_bit25: 1;
            unsigned CD_bit26: 1;
            unsigned CD_bit27: 1;

            unsigned CD_bit28: 1;
            unsigned CD_bit29: 1;
            unsigned CD_bit30: 1;
           unsigned CD_bit31: 1;
        } CLR_DATA45_bit;
    }; // 44h-47h
    union
    {
        volatile unsigned IN_DATA45;
        volatile struct
        {
            unsigned DI_bit0: 1;
            unsigned DI_bit1: 1;
            unsigned DI_bit2: 1;
            unsigned DI_bit3: 1;

            unsigned DI_bit4: 1;
            unsigned DI_bit5: 1;
            unsigned DI_bit6: 1;
            unsigned DI_bit7: 1;

            unsigned DI_bit8: 1;
            unsigned DI_bit9: 1;
            unsigned DI_bit10: 1;
            unsigned DI_bit11: 1;

            unsigned DI_bit12: 1;
            unsigned DI_bit13: 1;
            unsigned DI_bit14: 1;
            unsigned DI_bit15: 1;

            unsigned DI_bit16: 1;
            unsigned DI_bit17: 1;
            unsigned DI_bit18: 1;
            unsigned DI_bit19: 1;

            unsigned DI_bit20: 1;
            unsigned DI_bit21: 1;
            unsigned DI_bit22: 1;
            unsigned DI_bit23: 1;

            unsigned DI_bit24: 1;
            unsigned DI_bit25: 1;
            unsigned DI_bit26: 1;
            unsigned DI_bit27: 1;

            unsigned DI_bit28: 1;
            unsigned DI_bit29: 1;
            unsigned DI_bit30: 1;
            unsigned DI_bit31: 1;
        } IN_DATA45_bit;
    };// 48h-4bh, reset: 0h
    volatile unsigned SET_RIS_TRIG45; // 4ch-4fh, reset: 0h
    volatile unsigned CLR_RIS_TRIG45; // 50h-53h, reset: 0h
    volatile unsigned SET_FALL_TRIG45; //54h-57h, reset: 0h
    volatile unsigned CLR_FALL_TRIG45; //58h-5bh, reset: 0h
    volatile unsigned INTSTAT45; // 5ch-5fh
    union
    {
        volatile unsigned DIR67;
        volatile struct
        {
            unsigned OE_bit0: 1; // 96
            unsigned OE_bit1: 1; // 97
            unsigned OE_bit2: 1; // 98
            unsigned OE_bit3: 1; // 99

            unsigned OE_bit4: 1; // 100
            unsigned OE_bit5: 1; // 101
            unsigned OE_bit6: 1; // 102
            unsigned OE_bit7: 1; // 103

            unsigned OE_bit8: 1; // 104
            unsigned OE_bit9: 1; // 105
            unsigned OE_bit10: 1; // 106
            unsigned OE_bit11: 1; // 107

            unsigned OE_bit12: 1; // 108
            unsigned OE_bit13: 1; // 109
            unsigned OE_bit14: 1; // 110
            unsigned OE_bit15: 1; // 111

            unsigned OE_bit16: 1; // 112
            unsigned OE_bit17: 1; // 113
            unsigned OE_bit18: 1; // 114
            unsigned OE_bit19: 1; // 115

            unsigned OE_bit20: 1; // 116
            unsigned OE_bit21: 1; // 117
            unsigned OE_bit22: 1; // 118
            unsigned OE_bit23: 1; // 119

            unsigned OE_bit24: 1; // 120
            unsigned OE_bit25: 1; // 121
            unsigned OE_bit26: 1; // 122
            unsigned OE_bit27: 1; // 123

            unsigned OE_bit28: 1; // 124
            unsigned OE_bit29: 1; // 125
            unsigned OE_bit30: 1; // 126
            unsigned OE_bit31: 1; // 127
        } DIR67_bit;
    }; //38h-3bh, reset: ffffffffh
    union
    {
        volatile unsigned OUT_DATA67;
        volatile struct
        {
            unsigned DO_bit0: 1;
            unsigned DO_bit1: 1;
            unsigned DO_bit2: 1;
            unsigned DO_bit3: 1;

            unsigned DO_bit4: 1;
            unsigned DO_bit5: 1;
            unsigned DO_bit6: 1;
            unsigned DO_bit7: 1;

            unsigned DO_bit8: 1;
            unsigned DO_bit9: 1;
            unsigned DO_bit10: 1;
            unsigned DO_bit11: 1;

            unsigned DO_bit12: 1;
            unsigned DO_bit13: 1;
            unsigned DO_bit14: 1;
            unsigned DO_bit15: 1;

            unsigned DO_bit16: 1;
            unsigned DO_bit17: 1;
            unsigned DO_bit18: 1;
            unsigned DO_bit19: 1;

            unsigned DO_bit20: 1;
            unsigned DO_bit21: 1;
            unsigned DO_bit22: 1;
            unsigned DO_bit23: 1;

            unsigned DO_bit24: 1;
            unsigned DO_bit25: 1;
            unsigned DO_bit26: 1;
            unsigned DO_bit27: 1;

            unsigned DO_bit28: 1;
            unsigned DO_bit29: 1;
            unsigned DO_bit30: 1;
            unsigned DO_bit31: 1;
        } OUT_DATA67_bit;
    };// 3ch-3fh, reset: 0h
    union
    {
        volatile unsigned SET_DATA67;
        volatile struct
        {
            unsigned SD_bit0: 1;
            unsigned SD_bit1: 1;
            unsigned SD_bit2: 1;
            unsigned SD_bit3: 1;

            unsigned SD_bit4: 1;
            unsigned SD_bit5: 1;
            unsigned SD_bit6: 1;
            unsigned SD_bit7: 1;

            unsigned SD_bit8: 1;
            unsigned SD_bit9: 1;
            unsigned SD_bit10: 1;
            unsigned SD_bit11: 1;

            unsigned SD_bit12: 1;
            unsigned SD_bit13: 1;
            unsigned SD_bit14: 1;
            unsigned SD_bit15: 1;

            unsigned SD_bit16: 1;
            unsigned SD_bit17: 1;
            unsigned SD_bit18: 1;
            unsigned SD_bit19: 1;

            unsigned SD_bit20: 1;
            unsigned SD_bit21: 1;
            unsigned SD_bit22: 1;
            unsigned SD_bit23: 1;

            unsigned SD_bit24: 1;
            unsigned SD_bit25: 1;
            unsigned SD_bit26: 1;
            unsigned SD_bit27: 1;

            unsigned SD_bit28: 1;
            unsigned SD_bit29: 1;
            unsigned SD_bit30: 1;
            unsigned SD_bit31: 1;
        } SET_DATA67_bit;
    }; // 40h-43h
    union
    {
        volatile unsigned CLR_DATA67;
        volatile struct
        {
            unsigned CD_bit0: 1;
            unsigned CD_bit1: 1;
            unsigned CD_bit2: 1;
            unsigned CD_bit3: 1;

            unsigned CD_bit4: 1;
            unsigned CD_bit5: 1;
            unsigned CD_bit6: 1;
            unsigned CD_bit7: 1;

            unsigned CD_bit8: 1;
            unsigned CD_bit9: 1;
            unsigned CD_bit10: 1;
            unsigned CD_bit11: 1;

            unsigned CD_bit12: 1;
            unsigned CD_bit13: 1;
            unsigned CD_bit14: 1;
            unsigned CD_bit15: 1;

            unsigned CD_bit16: 1;
            unsigned CD_bit17: 1;
            unsigned CD_bit18: 1;
            unsigned CD_bit19: 1;

            unsigned CD_bit20: 1;
            unsigned CD_bit21: 1;
            unsigned CD_bit22: 1;
            unsigned CD_bit23: 1;

            unsigned CD_bit24: 1;
            unsigned CD_bit25: 1;
            unsigned CD_bit26: 1;
            unsigned CD_bit27: 1;

            unsigned CD_bit28: 1;
            unsigned CD_bit29: 1;
            unsigned CD_bit30: 1;
            unsigned CD_bit31: 1;
        } CLR_DATA67_bit;
    }; // 44h-47h
    union
    {
        volatile unsigned IN_DATA67;
        volatile struct
        {
            unsigned DI_bit0: 1;
            unsigned DI_bit1: 1;
            unsigned DI_bit2: 1;
            unsigned DI_bit3: 1;

            unsigned DI_bit4: 1;
            unsigned DI_bit5: 1;
            unsigned DI_bit6: 1;
            unsigned DI_bit7: 1;

            unsigned DI_bit8: 1;
            unsigned DI_bit9: 1;
            unsigned DI_bit10: 1;
            unsigned DI_bit11: 1;

            unsigned DI_bit12: 1;
            unsigned DI_bit13: 1;
            unsigned DI_bit14: 1;
            unsigned DI_bit15: 1;

            unsigned DI_bit16: 1;
            unsigned DI_bit17: 1;
            unsigned DI_bit18: 1;
            unsigned DI_bit19: 1;

            unsigned DI_bit20: 1;
            unsigned DI_bit21: 1;
            unsigned DI_bit22: 1;
            unsigned DI_bit23: 1;

            unsigned DI_bit24: 1;
            unsigned DI_bit25: 1;
            unsigned DI_bit26: 1;
            unsigned DI_bit27: 1;

            unsigned DI_bit28: 1;
            unsigned DI_bit29: 1;
            unsigned DI_bit30: 1;
            unsigned DI_bit31: 1;
        } IN_DATA67_bit;
        volatile unsigned SET_RIS_TRIG67; // 4ch-4fh, reset: 0h
        volatile unsigned CLR_RIS_TRIG67; // 50h-53h, reset: 0h
        volatile unsigned SET_FALL_TRIG67; //54h-57h, reset: 0h
        volatile unsigned CLR_FALL_TRIG67; //58h-5bh, reset: 0h
        volatile unsigned INTSTAT67; // 5ch-5fh
    };
} gpio_t;

// From memory mapped GPIO Registers Table 12-127 am64x_tech_ref.pdf page 4981
#define MCU_GPIO (*((volatile gpio_t *)MCU_GPIO0_OFFSET))
#define GPIO0 (*((volatile gpio_t *)GPIO0_OFFSET))
#define GPIO1 (*((volatile gpio_t *)GPIO1_OFFSET))

#endif // SYS_GPIO_H

