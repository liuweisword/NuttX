/************************************************************************************
 * arch/arm/src/artosyn/chip/ar8020_memorymap.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *            liuwei  <wei.liu@cecooleye.cn>
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

#ifndef __ARCH_ARM_SRC_ARTOSYN_CHIP_AR8020_MEMORYMAP_H
#define __ARCH_ARM_SRC_ARTOSYN_CHIP_AR8020_MEMORYMAP_H



#define AR_SDMMC_BASE       0x42000000      /* 0x42000000-0x42ffffff: sdcard ctrl */

#define AR_USB_BASE         0x43000000      /* 0x43000000-0x43ffffff: usb ctrl */
#define AR_USB1_BASE        0x43100000      /* 0x43000000-0x43ffffff: usb ctrl */

#define AR_SDRAM_BASE       0x80000000      /* 0x80000000-0x81ffffff: 16MB SDRAM */


#define AR_TIM0_BASE        0x40000000      /* 0x40000000-0x4003ffff: TIM0~7 */
#define AR_TIM1_BASE        0x40040000      /* 0x40040000-0x4007ffff: TIM10~17 */
#define AR_TIM2_BASE        0x40080000      /* 0x40080000-0x400fffff: TIM20~27 */

#define AR_SPI0_BASE        0x40100000      /* 0x40100000-0x4011ffff: SPI0 */
#define AR_SPI1_BASE        0x40120000      /* 0x40120000-0x4013ffff: SPI1 */
#define AR_SPI2_BASE        0x40140000      /* 0x40140000-0x4015ffff: SPI2 */
#define AR_SPI3_BASE        0x40160000      /* 0x40160000-0x4017ffff: SPI3 */
#define AR_SPI4_BASE        0x40180000      /* 0x40180000-0x4019ffff: SPI4 */
#define AR_SPI5_BASE        0x401A0000      /* 0x401a0000-0x401affff: SPI5 */
#define AR_SPI6_BASE        0x401C0000      /* 0x401c0000-0x401fffff: SPI6 */

#define AR_I2C0_BASE        0x40200000      /* 0x40200000-0x4023ffff: I2C0 */
#define AR_I2C1_BASE        0x40240000      /* 0x40240000-0x4027ffff: I2C1 */
#define AR_I2C2_BASE        0x40280000      /* 0x40280000-0x402bffff: I2C2 */
#define AR_I2C3_BASE        0x402C0000      /* 0x402c0000-0x402fffff: I2C3 */

#define AR_CAN0_BASE        0x40300000      /* 0x40300000-0x4033ffff: CAN0 */
#define AR_CAN1_BASE        0x40340000      /* 0x40340000-0x4037ffff: CAN1 */

#define AR_GPIOP0_BASE      0x40400000      /* 0x40400000-0x4043ffff: GPIOP0 */
#define AR_GPIOP1_BASE      0x40440000      /* 0x40440000-0x4047ffff: GPIOP1 */
#define AR_GPIOP2_BASE      0x40480000      /* 0x40480000-0x404bffff: GPIOP2 */
#define AR_GPIOP3_BASE      0x404C0000      /* 0x404C0000-0x404fffff: GPIOP3 */

#define AR_UART0_BASE       0x40500000      /* 0x40500000-0x4050ffff: UART0 */
#define AR_UART1_BASE       0x40510000      /* 0x40500000-0x4051ffff: UART1 */
#define AR_UART2_BASE       0x40520000      /* 0x40500000-0x4052ffff: UART2 */
#define AR_UART3_BASE       0x40530000      /* 0x40500000-0x4053ffff: UART3 */
#define AR_UART4_BASE       0x40540000      /* 0x40500000-0x4054ffff: UART4 */
#define AR_UART5_BASE       0x40550000      /* 0x40500000-0x4055ffff: UART5 */
#define AR_UART6_BASE       0x40560000      /* 0x40500000-0x4056ffff: UART6 */
#define AR_UART7_BASE       0x40570000      /* 0x40570000-0x4057ffff: UART6 */

#define AR_BKSRAM_BASE      0x21000000 + (60 * 1024)     /* 0x41100000-0x41100fff: 4K BACKUPRAM  */
#define AR_BKREG_BASE       0x41200000      /* 0x41200000-0x412fffff: 80Bit backup reg  */

#endif /* __ARCH_ARM_SRC_ARTOSYN_CHIP_AR8020_MEMORYMAP_H */
