/****************************************************************************************************
 * arch/arm/include/artosyn/ar8020_irq.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

/* This file should never be included directed but, rather, only indirectly through arch/irq.h */

#ifndef __ARCH_ARM_INCLUDE_ARTOSYN_AR8020_IRQ_H
#define __ARCH_ARM_INCLUDE_ARTOSYN_AR8020_IRQ_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to bits in the
 * NVIC.  This does, however, waste several words of memory in the IRQ to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15).  These common definitions can be found in the file
 * nuttx/arch/arm/include/stm32f7/irq.h which includes this file
 *
 * External interrupts (vectors >= 16)
 */

#define AR_IRQ_FIRST         (16) /* Vector number of the first external interrupt */


#define AR_IRQ_UART0        (AR_IRQ_FIRST + 0) /* 0: UART0 global interrupt */
#define AR_IRQ_UART1        (AR_IRQ_FIRST + 1) /* 1: UART1 global interrupt */
#define AR_IRQ_UART2        (AR_IRQ_FIRST + 2) /* 2: UART2 global interrupt */
#define AR_IRQ_UART3        (AR_IRQ_FIRST + 3) /* 3: UART3 global interrupt */
#define AR_IRQ_UART4        (AR_IRQ_FIRST + 4) /* 4: UART4 global interrupt */
#define AR_IRQ_UART5        (AR_IRQ_FIRST + 5) /* 5: UART5 global interrupt */
#define AR_IRQ_UART6        (AR_IRQ_FIRST + 6) /* 6: UART6 global interrupt */
#define AR_IRQ_UART7        (AR_IRQ_FIRST + 7) /* 7: UART7 global interrupt */
#define AR_IRQ_UART8        (AR_IRQ_FIRST + 8) /* 8: UART8 global interrupt */    
#define AR_IRQ_TIM0_CH0     (AR_IRQ_FIRST + 9) /* 9: TIM0_CH0 global interrupt */
#define AR_IRQ_TIM0_CH1     (AR_IRQ_FIRST + 10) /* 10: TIM0_CH1 global interrupt */
#define AR_IRQ_TIM0_CH2     (AR_IRQ_FIRST + 11) /* 11: TIM0_CH2 global interrupt */
#define AR_IRQ_TIM0_CH3     (AR_IRQ_FIRST + 12) /* 12: TIM0_CH3 global interrupt */
#define AR_IRQ_TIM0_CH4     (AR_IRQ_FIRST + 13) /* 13: TIM0_CH4 global interrupt */
#define AR_IRQ_TIM0_CH5     (AR_IRQ_FIRST + 14) /* 14: TIM0_CH5 global interrupt */
#define AR_IRQ_TIM0_CH6     (AR_IRQ_FIRST + 15) /* 15: TIM0_CH6 global interrupt */
#define AR_IRQ_TIM0_CH7     (AR_IRQ_FIRST + 16) /* 16: TIM0_CH7 global interrupt */
#define AR_IRQ_TIM1_CH0     (AR_IRQ_FIRST + 17) /* 17: TIM1_CH0 global interrupt */
#define AR_IRQ_TIM1_CH1     (AR_IRQ_FIRST + 18) /* 18: TIM1_CH1 global interrupt */
#define AR_IRQ_TIM1_CH2     (AR_IRQ_FIRST + 19) /* 19: TIM1_CH2 global interrupt */
#define AR_IRQ_TIM1_CH3     (AR_IRQ_FIRST + 20) /* 20: TIM1_CH3 global interrupt */
#define AR_IRQ_TIM1_CH4     (AR_IRQ_FIRST + 21) /* 21: TIM1_CH4 global interrupt */
#define AR_IRQ_TIM1_CH5     (AR_IRQ_FIRST + 22) /* 22: TIM1_CH5 global interrupt */
#define AR_IRQ_TIM1_CH6     (AR_IRQ_FIRST + 23) /* 23: TIM1_CH6 global interrupt */
#define AR_IRQ_TIM1_CH7     (AR_IRQ_FIRST + 24) /* 24: TIM1_CH7 global interrupt */
#define AR_IRQ_TIM2_CH0     (AR_IRQ_FIRST + 25) /* 25: TIM2_CH0 global interrupt */
#define AR_IRQ_TIM2_CH1     (AR_IRQ_FIRST + 26) /* 26: TIM2_CH1 global interrupt */
#define AR_IRQ_TIM2_CH2     (AR_IRQ_FIRST + 27) /* 27: TIM2_CH2 global interrupt */
#define AR_IRQ_TIM2_CH3     (AR_IRQ_FIRST + 28) /* 28: TIM2_CH3 global interrupt */
#define AR_IRQ_TIM2_CH4     (AR_IRQ_FIRST + 29) /* 29: TIM2_CH4 global interrupt */
#define AR_IRQ_TIM2_CH5     (AR_IRQ_FIRST + 30) /* 30: TIM2_CH5 global interrupt */
#define AR_IRQ_TIM2_CH6     (AR_IRQ_FIRST + 31) /* 31: TIM2_CH6 global interrupt */
#define AR_IRQ_TIM2_CH7     (AR_IRQ_FIRST + 32) /* 32: TIM2_CH7 global interrupt */  
#define AR_IRQ_SPI0         (AR_IRQ_FIRST + 33) /* 33: SPI0 global interrupt */
#define AR_IRQ_SPI1         (AR_IRQ_FIRST + 34) /* 34: SPI1 global interrupt */
#define AR_IRQ_SPI2         (AR_IRQ_FIRST + 35) /* 35: SPI2 global interrupt */
#define AR_IRQ_SPI3         (AR_IRQ_FIRST + 36) /* 36: SPI3 global interrupt */
#define AR_IRQ_SPI4         (AR_IRQ_FIRST + 37) /* 37: SPI4 global interrupt */
#define AR_IRQ_SPI5         (AR_IRQ_FIRST + 38) /* 38: SPI5 global interrupt */
#define AR_IRQ_SPI6         (AR_IRQ_FIRST + 39) /* 39: SPI6 global interrupt */   
#define AR_IRQ_I2C0         (AR_IRQ_FIRST + 40) /* 40: I2C0 global interrupt */
#define AR_IRQ_I2C1         (AR_IRQ_FIRST + 41) /* 41: I2C1 global interrupt */
#define AR_IRQ_I2C2         (AR_IRQ_FIRST + 42) /* 42: I2C2 global interrupt */
#define AR_IRQ_I2C3         (AR_IRQ_FIRST + 43) /* 43: I2C3 global interrupt */   
#define AR_IRQ_CAN0         (AR_IRQ_FIRST + 44) /* 44: CAN0 global interrupts */
#define AR_IRQ_CAN1         (AR_IRQ_FIRST + 45) /* 45: CAN1 global interrupts */
#define AR_IRQ_CAN2         (AR_IRQ_FIRST + 46) /* 46: CAN2 global interrupts */
#define AR_IRQ_CAN3         (AR_IRQ_FIRST + 47) /* 47: CAN3 global interrupts */  
#define AR_IRQ_WDT0         (AR_IRQ_FIRST + 48) /* 48: Window Watchdog interrupt */
#define AR_IRQ_WDT1         (AR_IRQ_FIRST + 49) /* 49: Window Watchdog interrupt */   
#define AR_IRQ_EXTI0_7      (AR_IRQ_FIRST + 50) /* 50:  EXTI Line 0 interrupt */
#define AR_IRQ_EXTI32_39    (AR_IRQ_FIRST + 51) /* 51:  EXTI Line 1 interrupt */
#define AR_IRQ_EXTI64_71    (AR_IRQ_FIRST + 52) /* 52:  EXTI Line 2 interrupt */
#define AR_IRQ_EXTI96_103   (AR_IRQ_FIRST + 53) /* 53:  EXTI Line 3 interrupt */  
#define AR_IRQ_I2C4         (AR_IRQ_FIRST + 54) /* 54: I2C4 global interrupt */   
#define AR_IRQ_RTC          (AR_IRQ_FIRST + 55) /* 55:  RTC global interrupt */   
#define AR_IRQ_USB0         (AR_IRQ_FIRST + 56) /* 56:  USB0 global interrupt */
#define AR_IRQ_USB1         (AR_IRQ_FIRST + 57) /* 57:  USB1 global interrupt */  
#define AR_IRQ_SDMMC        (AR_IRQ_FIRST + 58) /* 58:  SDMMC global interrupt */  
#define AR_IRQ_DMA          (AR_IRQ_FIRST + 59) /* 59: DMA global interrupt */    
#define AR_IRQ_UART9        (AR_IRQ_FIRST + 60) /* 60: UART9 global interrupt */  
#define AR_IRQ_ENCODER      (AR_IRQ_FIRST + 61) /* 61: Video Encoder global interrupt */  
#define AR_IRQ_UART10       (AR_IRQ_FIRST + 62) /* 62: UART10 global interrupt */ 
#define AR_IRQ_I2C4ENC      (AR_IRQ_FIRST + 63) /* 63: SLAVE I2C global interrupt */  
#define AR_IRQ_SPIBBENC     (AR_IRQ_FIRST + 64) /* 64: SLAVE SPI BB global interrupt */   
#define AR_IRQ_WDT2         (AR_IRQ_FIRST + 65) /* 65: Window Watchdog interrupt */   
#define AR_IRQ_SPIBB        (AR_IRQ_FIRST + 66) /* 66: SLAVE SPI BB global interrupt */   
#define AR_IRQ_VWFD0        (AR_IRQ_FIRST + 67) /* 67: Video write frame done CH0 */
#define AR_IRQ_VWFD1        (AR_IRQ_FIRST + 68) /* 68: Video write frame done CH1 */  
#define AR_IRQ_RESVSOC0     (AR_IRQ_FIRST + 69) /* 69: RES VSOC interrupt 0 */
#define AR_IRQ_RESVSOC1     (AR_IRQ_FIRST + 70) /* 70: RES VSOC interrupt 1 */    
#define AR_IRQ_SRAM0        (AR_IRQ_FIRST + 71) /* 71: SRAM READY IRQ 0 */
#define AR_IRQ_SRAM1        (AR_IRQ_FIRST + 72) /* 72: SRAM READY IRQ 1 */    
#define AR_IRQ_BBTXEN       (AR_IRQ_FIRST + 73) /* 73: BB TX ENABLE */
#define AR_IRQ_BBRXEN       (AR_IRQ_FIRST + 74) /* 74: BB RX ENABLE */    
#define AR_IRQ_FPU0         (AR_IRQ_FIRST + 75) /* 75: FPU0 global interrupt */
#define AR_IRQ_FPU1         (AR_IRQ_FIRST + 76) /* 76: FPU1 global interrupt */
#define AR_IRQ_FPU2         (AR_IRQ_FIRST + 77) /* 77: FPU2 global interrupt */
#define AR_IRQ_FPU3         (AR_IRQ_FIRST + 78) /* 78: FPU3 global interrupt */
#define AR_IRQ_FPU4         (AR_IRQ_FIRST + 79) /* 79: FPU4 global interrupt */
#define AR_IRQ_FPU5         (AR_IRQ_FIRST + 80) /* 80: FPU5 global interrupt */   
#define AR_IRQ_CTI0         (AR_IRQ_FIRST + 81) /* 81: CTI0 global interrupt */
#define AR_IRQ_CTI1         (AR_IRQ_FIRST + 82) /* 82: CTI1 global interrupt */

#define NR_INTERRUPTS         83
#define NR_VECTORS          (AR_IRQ_FIRST+NR_INTERRUPTS)

/* EXTI interrupts (Do not use IRQ numbers) */

#define NR_IRQS               NR_VECTORS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_ARTOSYN_AR8020_IRQ_H */

