/************************************************************************************
 * arch/arm/src/artosyn/chip/ar_define.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/
//#include <nuttx/config.h>

#ifndef __ARCH_ARM_SRC_ARTOSYN_CHIP_AR_DEFINE_H
#define __ARCH_ARM_SRC_ARTOSYN_CHIP_AR_DEFINE_H


#define ARGREG1_ADC_DATA                  0x40B000F4
#define ARGREG1_ADC_CHANNEL               0x40B000EC

#ifndef BIT
#define BIT(n)    ((uint32_t)1 << (n))
#endif/* BIT */

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

#define __IO    volatile             /*!< Defines 'read / write' permissions */

#define m7_malloc             pvPortMalloc
#define m7_free               vPortFree

typedef uint32_t HAL_RET_T;
typedef uint8_t HAL_BOOL_T;


#define HAL_OK                                      (0)

#define HAL_TIME_OUT                                (0xFF)
#define HAL_BUSY                                    (0xFE)
#define HAL_OCCUPIED                                (0xFD)
#define HAL_NOT_INITED                              (0xFC)


#define HAL_GPIO_ERR_MASK                           (0x30000)
#define HAL_GPIO_ERR_UNKNOWN                        (HAL_GPIO_ERR_MASK | 0x1)

#endif /* __ARCH_ARM_SRC_ARTOSYN_CHIP_AR_DEFINE_H */
