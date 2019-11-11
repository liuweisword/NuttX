/************************************************************************************
 * arch/arm/src/artosyn/ar_tim.h
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With modifications and updates by:
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_ARTOSYN_AR_TIM_H
#define __ARCH_ARM_SRC_ARTOSYN_AR_TIM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
//#include "chip/ar_tim.h"


#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#define TIME_ENABLE         1
#define TIEM_DISABLE        0

#define TIME_INT_MASK       (0x1 << 2)
#define TIME_PWM_ENABLE     (0x1 << 3)
#define USER_DEFINED        (0x1 << 1)

#define BASE_ADDR_TIMER0    0x40000000
#define BASE_ADDR_TIMER1    0x40040000
#define BASE_ADDR_TIMER2    0x40080000

#define TMREOI              0xa4
#define TMRNEOI_0           0x0C
#define TMRNEOI_1           0x20
#define TMRNEOI_2           0x34
#define TMRNEOI_3           0x48
#define TMRNEOI_4           0x5C
#define TMRNEOI_5           0x70
#define TMRNEOI_6           0x84
#define TMRNEOI_7           0x98

#define CTRL_0              0x08
#define CTRL_1              0x1C
#define CTRL_2              0x30
#define CTRL_3              0x44
#define CTRL_4              0x58
#define CTRL_5              0x6C
#define CTRL_6              0x80
#define CTRL_7              0x94


#define TIMER0_TNT_STATUS   0x10
#define TIMER1_TNT_STATUS   0x24
#define TIMER2_TNT_STATUS   0x38
#define TIMER3_TNT_STATUS   0x4C
#define TIMER4_TNT_STATUS   0x60
#define TIMER5_TNT_STATUS   0x74
#define TIMER6_TNT_STATUS   0x88
#define TIMER7_TNT_STATUS   0x9C

#define CNT1_0              0x00
#define CNT1_1              0x14
#define CNT1_2              0x28
#define CNT1_3              0x3C
#define CNT1_4              0x50
#define CNT1_5              0x64
#define CNT1_6              0x78
#define CNT1_7              0x8C

#define CNT2_0              0xB0
#define CNT2_1              0xB4
#define CNT2_2              0xB8
#define CNT2_3              0xBC
#define CNT2_4              0xC0
#define CNT2_5              0xC4
#define CNT2_6              0xC8
#define CNT2_7              0xCC

typedef enum
{
    TIMER_NUM0=0,
    TIMER_NUM1,
    TIMER_NUM2,
    TIMER_NUM3,
    TIMER_NUM4,
    TIMER_NUM5,
    TIMER_NUM6,
    TIMER_NUM7,
    TIMER_NUM8,
    TIMER_NUM9,
    TIMER_NUM10,
    TIMER_NUM11,
    TIMER_NUM12,
    TIMER_NUM13,
    TIMER_NUM14,
    TIMER_NUM15,
    TIMER_NUM16,
    TIMER_NUM17,
    TIMER_NUM18,
    TIMER_NUM19,
    TIMER_NUM20,
    TIMER_NUM21,
    TIMER_NUM22,
    TIMER_NUM23
} ENUM_AR_TIMER_NUM;

int ar_timer_initialize(FAR const char *devpath, int timer);



#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_ARTOSYN_AR8020_TIM_H */
