/****************************************************************************
 * arch/arm/src/artosyn/ar_rcc.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nvic.h>

#include "chip/ar_define.h"
#include "chip/ar_config.h"
#include "chip/ar_memorymap.h"

#include "ar_uart.h"
#include "ar_rcc.h"
#include "ar_spi.h"
#include "cache.h"


uint32_t Reg_Read32(uint32_t regAddr)
{
    return *((volatile uint32_t*)regAddr);
}

void Reg_Write32(uint32_t regAddr, uint32_t regData)
{
    *((volatile uint32_t*)regAddr) = regData;
}

void Reg_Write32_Mask(uint32_t regAddr, uint32_t regData, uint32_t regDataMask)
{
    uint32_t u32_regDataTmp;

    u32_regDataTmp = *((volatile uint32_t*)regAddr);
    u32_regDataTmp &= ~regDataMask;
    u32_regDataTmp |= regData & regDataMask;

    *((volatile uint32_t*)regAddr) = u32_regDataTmp;
}

uint32_t read_reg32(uint32_t *addr)
{
    return *((volatile uint32_t*)addr);
}


uint64_t Reg_Read64(uint32_t regAddr)
{
    volatile uint64_t* ptr_regAddr = (uint64_t*)regAddr;
    return *ptr_regAddr;
}


static inline void local_irq_restore(unsigned long flags)

{

    __asm volatile(

                   "msr     primask, %0      @ local_irq_restore"

                   :

                   : "r" (flags)

                   : "memory", "cc");



    __asm volatile("dmb\n\t""dsb\n\t""isb\n\t");

}

static inline unsigned long local_irq_disable_save_flags(void)

{

    unsigned long flags;



    __asm volatile("mrs     %0, primask        @ local_irq_disable_save_flags\n"

                   "cpsid   i"

                   : "=r" (flags) : : "memory", "cc");



    __asm volatile("dmb\n\t""dsb\n\t""isb\n\t");



   return flags;

}

_EXT_ITCM void ar_fpuconfig_enable(void)
{
    Reg_Write32_Mask(0xE000ED88, 0xF << 20, 0xF << 20);
}

_EXT_ITCM void ar_fpuconfig_disable(void)
{
    Reg_Write32_Mask(0xE000ED88, 0, 0xF << 20);
}

_EXT_ITCM static int ar_timerisr(int irq, uint32_t *regs, void *arg)
{
    /* Process timer interrupt */
    sched_process_timer();
    return 0;
}

_EXT_ITCM void arm_timer_initialize(void)
{
    uint32_t regval;

    /* Configure SysTick to interrupt at the requested rate */

    putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);
    putreg32(0, NVIC_SYSTICK_CURRENT);

    /* Attach the timer interrupt vector */

    (void)irq_attach(AR_IRQ_SYSTICK, (xcpt_t)ar_timerisr, NULL);


    regval = (NVIC_SYSTICK_CTRL_TICKINT | NVIC_SYSTICK_CTRL_ENABLE);
    putreg32(regval, NVIC_SYSTICK_CTRL);

    /* And enable the timer interrupt */

    up_enable_irq(AR_IRQ_SYSTICK);

}