/************************************************************************************
 * arch/arm/src/artosyn/chip/ar8020_uart.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_ARTOSYN_CHIP_AR8020_UART_H
#define __ARCH_ARM_SRC_ARTOSYN_CHIP_AR8020_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>


# if 1
/* Register offsets *****************************************************************/

#define AR_UART_RBR_OFFSET        0x0000
#define AR_UART_THR_OFFSET        0x0000
#define AR_UART_DLL_OFFSET        0x0000
#define AR_UART_DLH_OFFSET        0x0004
#define AR_UART_IER_OFFSET        0x0004
#define AR_UART_IIR_OFFSET        0x0008
#define AR_UART_FCR_OFFSET        0x0008
#define AR_UART_LCR_OFFSET        0x000c
#define AR_UART_MCR_OFFSET        0x0010
#define AR_UART_LSR_OFFSET        0x0014
#define AR_UART_MSR_OFFSET        0x0018
#define AR_UART_SCR_OFFSET        0x001c
#define AR_UART_LPDLL_OFFSET      0x0020
#define AR_UART_LPDLH_OFFSET      0x0024


#define AR_UART_USR_OFFSET        0x007c


/* Register addresses ***************************************************************/


#define AR_UART0_RBR       (AR_UART0_BASE+AR_UART_RBR_OFFSET)
#define AR_UART0_THR       (AR_UART0_BASE+AR_UART_THR_OFFSET)
#define AR_UART0_DLL       (AR_UART0_BASE+AR_UART_DLL_OFFSET)
#define AR_UART0_DLH       (AR_UART0_BASE+AR_UART_DLH_OFFSET)
#define AR_UART0_IER       (AR_UART0_BASE+AR_UART_IER_OFFSET)
#define AR_UART0_IIR       (AR_UART0_BASE+AR_UART_IIR_OFFSET)
#define AR_UART0_FCR       (AR_UART0_BASE+AR_UART_FCR_OFFSET)
#define AR_UART0_LCR       (AR_UART0_BASE+AR_UART_LCR_OFFSET)
#define AR_UART0_MCR       (AR_UART0_BASE+AR_UART_MCR_OFFSET)
#define AR_UART0_LSR       (AR_UART0_BASE+AR_UART_LSR_OFFSET)
#define AR_UART0_MSR       (AR_UART0_BASE+AR_UART_MSR_OFFSET)
#define AR_UART0_SCR       (AR_UART0_BASE+AR_UART_SCR_OFFSET)
#define AR_UART0_LPDLL     (AR_UART0_BASE+AR_UART_LPDLL_OFFSET)
#define AR_UART0_LPDLH     (AR_UART0_BASE+AR_UART_LPDLH_OFFSET)



#define AR_UART1_RBR       (AR_UART1_BASE+AR_UART_RBR_OFFSET)
#define AR_UART1_THR       (AR_UART1_BASE+AR_UART_THR_OFFSET)
#define AR_UART1_DLL       (AR_UART1_BASE+AR_UART_DLL_OFFSET)
#define AR_UART1_DLH       (AR_UART1_BASE+AR_UART_DLH_OFFSET)
#define AR_UART1_IER       (AR_UART1_BASE+AR_UART_IER_OFFSET)
#define AR_UART1_IIR       (AR_UART1_BASE+AR_UART_IIR_OFFSET)
#define AR_UART1_FCR       (AR_UART1_BASE+AR_UART_FCR_OFFSET)
#define AR_UART1_LCR       (AR_UART1_BASE+AR_UART_LCR_OFFSET)
#define AR_UART1_MCR       (AR_UART1_BASE+AR_UART_MCR_OFFSET)
#define AR_UART1_LSR       (AR_UART1_BASE+AR_UART_LSR_OFFSET)
#define AR_UART1_MSR       (AR_UART1_BASE+AR_UART_MSR_OFFSET)
#define AR_UART1_SCR       (AR_UART1_BASE+AR_UART_SCR_OFFSET)
#define AR_UART1_LPDLL     (AR_UART1_BASE+AR_UART_LPDLL_OFFSET)
#define AR_UART1_LPDLH     (AR_UART1_BASE+AR_UART_LPDLH_OFFSET)



#define AR_UART2_RBR       (AR_UART2_BASE+AR_UART_RBR_OFFSET)
#define AR_UART2_THR       (AR_UART2_BASE+AR_UART_THR_OFFSET)
#define AR_UART2_DLL       (AR_UART2_BASE+AR_UART_DLL_OFFSET)
#define AR_UART2_DLH       (AR_UART2_BASE+AR_UART_DLH_OFFSET)
#define AR_UART2_IER       (AR_UART2_BASE+AR_UART_IER_OFFSET)
#define AR_UART2_IIR       (AR_UART2_BASE+AR_UART_IIR_OFFSET)
#define AR_UART2_FCR       (AR_UART2_BASE+AR_UART_FCR_OFFSET)
#define AR_UART2_LCR       (AR_UART2_BASE+AR_UART_LCR_OFFSET)
#define AR_UART2_MCR       (AR_UART2_BASE+AR_UART_MCR_OFFSET)
#define AR_UART2_LSR       (AR_UART2_BASE+AR_UART_LSR_OFFSET)
#define AR_UART2_MSR       (AR_UART2_BASE+AR_UART_MSR_OFFSET)
#define AR_UART2_SCR       (AR_UART2_BASE+AR_UART_SCR_OFFSET)
#define AR_UART2_LPDLL     (AR_UART2_BASE+AR_UART_LPDLL_OFFSET)
#define AR_UART2_LPDLH     (AR_UART2_BASE+AR_UART_LPDLH_OFFSET)



#define AR_UART3_RBR       (AR_UART3_BASE+AR_UART_RBR_OFFSET)
#define AR_UART3_THR       (AR_UART3_BASE+AR_UART_THR_OFFSET)
#define AR_UART3_DLL       (AR_UART3_BASE+AR_UART_DLL_OFFSET)
#define AR_UART3_DLH       (AR_UART3_BASE+AR_UART_DLH_OFFSET)
#define AR_UART3_IER       (AR_UART3_BASE+AR_UART_IER_OFFSET)
#define AR_UART3_IIR       (AR_UART3_BASE+AR_UART_IIR_OFFSET)
#define AR_UART3_FCR       (AR_UART3_BASE+AR_UART_FCR_OFFSET)
#define AR_UART3_LCR       (AR_UART3_BASE+AR_UART_LCR_OFFSET)
#define AR_UART3_MCR       (AR_UART3_BASE+AR_UART_MCR_OFFSET)
#define AR_UART3_LSR       (AR_UART3_BASE+AR_UART_LSR_OFFSET)
#define AR_UART3_MSR       (AR_UART3_BASE+AR_UART_MSR_OFFSET)
#define AR_UART3_SCR       (AR_UART3_BASE+AR_UART_SCR_OFFSET)
#define AR_UART3_LPDLL     (AR_UART3_BASE+AR_UART_LPDLL_OFFSET)
#define AR_UART3_LPDLH     (AR_UART3_BASE+AR_UART_LPDLH_OFFSET)


#define AR_UART4_RBR       (AR_UART4_BASE+AR_UART_RBR_OFFSET)
#define AR_UART4_THR       (AR_UART4_BASE+AR_UART_THR_OFFSET)
#define AR_UART4_DLL       (AR_UART4_BASE+AR_UART_DLL_OFFSET)
#define AR_UART4_DLH       (AR_UART4_BASE+AR_UART_DLH_OFFSET)
#define AR_UART4_IER       (AR_UART4_BASE+AR_UART_IER_OFFSET)
#define AR_UART4_IIR       (AR_UART4_BASE+AR_UART_IIR_OFFSET)
#define AR_UART4_FCR       (AR_UART4_BASE+AR_UART_FCR_OFFSET)
#define AR_UART4_LCR       (AR_UART4_BASE+AR_UART_LCR_OFFSET)
#define AR_UART4_MCR       (AR_UART4_BASE+AR_UART_MCR_OFFSET)
#define AR_UART4_LSR       (AR_UART4_BASE+AR_UART_LSR_OFFSET)
#define AR_UART4_MSR       (AR_UART4_BASE+AR_UART_MSR_OFFSET)
#define AR_UART4_SCR       (AR_UART4_BASE+AR_UART_SCR_OFFSET)
#define AR_UART4_LPDLL     (AR_UART4_BASE+AR_UART_LPDLL_OFFSET)
#define AR_UART4_LPDLH     (AR_UART4_BASE+AR_UART_LPDLH_OFFSET)


#define AR_UART5_RBR       (AR_UART5_BASE+AR_UART_RBR_OFFSET)
#define AR_UART5_THR       (AR_UART5_BASE+AR_UART_THR_OFFSET)
#define AR_UART5_DLL       (AR_UART5_BASE+AR_UART_DLL_OFFSET)
#define AR_UART5_DLH       (AR_UART5_BASE+AR_UART_DLH_OFFSET)
#define AR_UART5_IER       (AR_UART5_BASE+AR_UART_IER_OFFSET)
#define AR_UART5_IIR       (AR_UART5_BASE+AR_UART_IIR_OFFSET)
#define AR_UART5_FCR       (AR_UART5_BASE+AR_UART_FCR_OFFSET)
#define AR_UART5_LCR       (AR_UART5_BASE+AR_UART_LCR_OFFSET)
#define AR_UART5_MCR       (AR_UART5_BASE+AR_UART_MCR_OFFSET)
#define AR_UART5_LSR       (AR_UART5_BASE+AR_UART_LSR_OFFSET)
#define AR_UART5_MSR       (AR_UART5_BASE+AR_UART_MSR_OFFSET)
#define AR_UART5_SCR       (AR_UART5_BASE+AR_UART_SCR_OFFSET)
#define AR_UART5_LPDLL     (AR_UART5_BASE+AR_UART_LPDLL_OFFSET)
#define AR_UART5_LPDLH     (AR_UART5_BASE+AR_UART_LPDLH_OFFSET)


#define AR_UART6_RBR       (AR_UART6_BASE+AR_UART_RBR_OFFSET)
#define AR_UART6_THR       (AR_UART6_BASE+AR_UART_THR_OFFSET)
#define AR_UART6_DLL       (AR_UART6_BASE+AR_UART_DLL_OFFSET)
#define AR_UART6_DLH       (AR_UART6_BASE+AR_UART_DLH_OFFSET)
#define AR_UART6_IER       (AR_UART6_BASE+AR_UART_IER_OFFSET)
#define AR_UART6_IIR       (AR_UART6_BASE+AR_UART_IIR_OFFSET)
#define AR_UART6_FCR       (AR_UART6_BASE+AR_UART_FCR_OFFSET)
#define AR_UART6_LCR       (AR_UART6_BASE+AR_UART_LCR_OFFSET)
#define AR_UART6_MCR       (AR_UART6_BASE+AR_UART_MCR_OFFSET)
#define AR_UART6_LSR       (AR_UART6_BASE+AR_UART_LSR_OFFSET)
#define AR_UART6_MSR       (AR_UART6_BASE+AR_UART_MSR_OFFSET)
#define AR_UART6_SCR       (AR_UART6_BASE+AR_UART_SCR_OFFSET)
#define AR_UART6_LPDLL     (AR_UART6_BASE+AR_UART_LPDLL_OFFSET)
#define AR_UART6_LPDLH     (AR_UART6_BASE+AR_UART_LPDLH_OFFSET)


#define AR_UART7_RBR       (AR_UART7_BASE+AR_UART_RBR_OFFSET)
#define AR_UART7_THR       (AR_UART7_BASE+AR_UART_THR_OFFSET)
#define AR_UART7_DLL       (AR_UART7_BASE+AR_UART_DLL_OFFSET)
#define AR_UART7_DLH       (AR_UART7_BASE+AR_UART_DLH_OFFSET)
#define AR_UART7_IER       (AR_UART7_BASE+AR_UART_IER_OFFSET)
#define AR_UART7_IIR       (AR_UART7_BASE+AR_UART_IIR_OFFSET)
#define AR_UART7_FCR       (AR_UART7_BASE+AR_UART_FCR_OFFSET)
#define AR_UART7_LCR       (AR_UART7_BASE+AR_UART_LCR_OFFSET)
#define AR_UART7_MCR       (AR_UART7_BASE+AR_UART_MCR_OFFSET)
#define AR_UART7_LSR       (AR_UART7_BASE+AR_UART_LSR_OFFSET)
#define AR_UART7_MSR       (AR_UART7_BASE+AR_UART_MSR_OFFSET)
#define AR_UART7_SCR       (AR_UART7_BASE+AR_UART_SCR_OFFSET)
#define AR_UART7_LPDLL     (AR_UART7_BASE+AR_UART_LPDLL_OFFSET)
#define AR_UART7_LPDLH     (AR_UART7_BASE+AR_UART_LPDLH_OFFSET)


#define AR_UART8_RBR       (AR_UART8_BASE+AR_UART_RBR_OFFSET)
#define AR_UART8_THR       (AR_UART8_BASE+AR_UART_THR_OFFSET)
#define AR_UART8_DLL       (AR_UART8_BASE+AR_UART_DLL_OFFSET)
#define AR_UART8_DLH       (AR_UART8_BASE+AR_UART_DLH_OFFSET)
#define AR_UART8_IER       (AR_UART8_BASE+AR_UART_IER_OFFSET)
#define AR_UART8_IIR       (AR_UART8_BASE+AR_UART_IIR_OFFSET)
#define AR_UART8_FCR       (AR_UART8_BASE+AR_UART_FCR_OFFSET)
#define AR_UART8_LCR       (AR_UART8_BASE+AR_UART_LCR_OFFSET)
#define AR_UART8_MCR       (AR_UART8_BASE+AR_UART_MCR_OFFSET)
#define AR_UART8_LSR       (AR_UART8_BASE+AR_UART_LSR_OFFSET)
#define AR_UART8_MSR       (AR_UART8_BASE+AR_UART_MSR_OFFSET)
#define AR_UART8_SCR       (AR_UART8_BASE+AR_UART_SCR_OFFSET)
#define AR_UART8_LPDLL     (AR_UART8_BASE+AR_UART_LPDLL_OFFSET)
#define AR_UART8_LPDLH     (AR_UART8_BASE+AR_UART_LPDLH_OFFSET)

/* Register bit definitions *********************************************************/
/* RBR (DLAB =0) Receiver Buffer Register (all) */

#define UART_RBR_MASK                (0xff)    /* Bits 0-7: Oldest received byte in RX FIFO */
                                               /* Bits 8-31: Reserved */

/* THR (DLAB =0) Transmit Holding Register (all) */

#define UART_THR_MASK                (0xff)    /* Bits 0-7: Adds byte to TX FIFO */
                                               /* Bits 8-31: Reserved */

/* DLL (DLAB =1) Divisor Latch LSB (all) */

#define UART_DLL_MASK                (0xff)    /* Bits 0-7: DLL */
                                               /* Bits 8-31: Reserved */

/* DLM (DLAB =1) Divisor Latch MSB (all) */

#define UART_DLH_MASK                (0xff)    /* Bits 0-7: DLH */
                                               /* Bits 8-31: Reserved */


/* IER (DLAB =0) Interrupt Enable Register (all) */

#define UART_IER_ERBFI               (1 << 0)  /* Bit 0: RBR Interrupt Enable */
#define UART_IER_ETBEI               (1 << 1)  /* Bit 1: THRE Interrupt Enable */
#define UART_IER_ELSI                (1 << 2)  /* Bit 2: RX Line Status Interrupt Enable */
#define UART_IER_EDSSI               (1 << 3)  /* Bit 3: Modem Status Interrupt Enable */
                                               /* Bits 4-6: Reserved */
#define UART_IER_PTIME               (1 << 7)  /* Bit 7: Programmable THRE Interrupt Mode Enable */

#define UART_IER_ALLIE               (0x008f)



/* IIR Interrupt ID Register (all) */

#define UART_IIR_INTID_SHIFT         (0)       /* Bits 0-3: Interrupt identification */
#define UART_IIR_INTID_MASK          (15 << UART_IIR_INTID_SHIFT)

#define UART_IIR_INTID_MSI           (0 << UART_IIR_INTID_SHIFT) /* Modem Status  */
#define UART_IIR_INTID_NIP           (1 << UART_IIR_INTID_SHIFT) /* no interrupt pending */
#define UART_IIR_INTID_THRE          (2 << UART_IIR_INTID_SHIFT) /* THR empty */
#define UART_IIR_INTID_RDA           (4 << UART_IIR_INTID_SHIFT) /* Receive Data Available (RDA) */
#define UART_IIR_INTID_RLS           (6 << UART_IIR_INTID_SHIFT) /* Receive Line Status (RLS) */
#define UART_IIR_INTID_BUSY          (7 << UART_IIR_INTID_SHIFT) /* busy detect */
#define UART_IIR_INTID_CTI           (12 << UART_IIR_INTID_SHIFT) /* 2b - Character Time-out Indicator (CTI) */
                                               /* Bits 4-5: Reserved */

#define UART_IIR_FIFOEN_SHIFT        (6)       /* Bits 6-7: Copies of FCR bit 0 */
#define UART_IIR_FIFOEN_MASK         (3 << UART_IIR_FIFOEN_SHIFT)

#define UART_IIR_FIFOEN_ENABLE       (0 << UART_IIR_FIFOEN_SHIFT) /* ENABLE Status  */
#define UART_IIR_FIFOEN_DISABLE      (3 << UART_IIR_FIFOEN_SHIFT) /* DISABLE Status  */
                                               /* Bits 8-31: Reserved */


/* FCR FIFO Control Register (all) */

#define UART_FCR_FIFOEN              (1 << 0)  /* Bit 0:  Enable FIFOs */

#define UART_FCR_RXRST               (1 << 1)  /* Bit 1:  RX FIFO Reset */
#define UART_FCR_TXRST               (1 << 2)  /* Bit 2:  TX FIFO Reset */

#define UART_FCR_DMAMODE             (1 << 3)  /* Bit 3:  DMA Mode Select */


#define UART_FCR_TXTRIGGER_SHIFT     (4)       /* Bits 4-5:TX Empty Trigger */
#define UART_FCR_TXTRIGGER_MASK      (3 << UART_FCR_TXTRIGGER_SHIFT)
#define UART_FCR_TXTRIGGER_0         (0 << UART_FCR_TXTRIGGER_SHIFT) /* Trigger level 0 (empty) */
#define UART_FCR_TXTRIGGER_4         (1 << UART_FCR_TXTRIGGER_SHIFT) /* Trigger level 1 (2 characters) */
#define UART_FCR_TXTRIGGER_8         (2 << UART_FCR_TXTRIGGER_SHIFT) /* Trigger level 2 (1/4) */
#define UART_FCR_TXTRIGGER_14        (3 << UART_FCR_TXTRIGGER_SHIFT) /* Trigger level 3 (1/2) */

#define UART_FCR_RXTRIGGER_SHIFT     (6)       /* Bits 6-7: RX Trigger Level */
#define UART_FCR_RXTRIGGER_MASK      (3 << UART_FCR_RXTRIGGER_SHIFT)
#define UART_FCR_RXTRIGGER_0         (0 << UART_FCR_RXTRIGGER_SHIFT) /* Trigger level 0 (1 character) */
#define UART_FCR_RXTRIGGER_4         (1 << UART_FCR_RXTRIGGER_SHIFT) /* Trigger level 1 (4 characters) */
#define UART_FCR_RXTRIGGER_8         (2 << UART_FCR_RXTRIGGER_SHIFT) /* Trigger level 2 (8 characters) */
#define UART_FCR_RXTRIGGER_14        (3 << UART_FCR_RXTRIGGER_SHIFT) /* Trigger level 3 (14 characters) */
                                               /* Bits 8-31: Reserved */

/* LCR Line Control Register (all) */

#define UART_LCR_DLS_SHIFT           (0)       /* Bit 0-1: Word Length Select */
#define UART_LCR_DLS_MASK            (3 << UART_LCR_DLS_SHIFT)

#define UART_LCR_DLS_5BIT            (0 << UART_LCR_DLS_SHIFT)
#define UART_LCR_DLS_6BIT            (1 << UART_LCR_DLS_SHIFT)
#define UART_LCR_DLS_7BIT            (2 << UART_LCR_DLS_SHIFT)
#define UART_LCR_DLS_8BIT            (3 << UART_LCR_DLS_SHIFT)

#define UART_LCR_STOP                (1 << 2)  /* Bit 2:  Stop Bit Select */

#define UART_LCR_PE                  (1 << 3)  /* Bit 3:  Parity Enable */

#define UART_LCR_PS_SHIFT            (4)       /* Bits 4-5: Parity Select */
#define UART_LCR_PS_MASK             (3 << UART_LCR_PS_SHIFT)
#define UART_LCR_PS_ODD              (0 << UART_LCR_PS_SHIFT) /* Odd parity */
#define UART_LCR_PS_EVEN             (1 << UART_LCR_PS_SHIFT) /* Even Parity */
#define UART_LCR_PS_STICK1           (2 << UART_LCR_PS_SHIFT) /* Forced "1" stick parity */
#define UART_LCR_PS_STICK0           (3 << UART_LCR_PS_SHIFT) /* Forced "0" stick parity */


#define UART_LCR_BRK                 (1 << 6)  /* Bit 6: Break Control */

#define UART_LCR_DLAB                (1 << 7)  /* Bit 7: Divisor Latch Access Bit (DLAB) */
                                               /* Bits 8-31: Reserved */

/* MCR Modem Control Register (UART1 only) */

#define UART_MCR_DTR                 (1 << 0)  /* Bit 0:  DTR Control Source for DTR output */

#define UART_MCR_RTS                 (1 << 1)  /* Bit 1:  Control Source for  RTS output */

#define UART_MCR_OUT1                (1 << 2)  /* Bit 2:  OUT1 */
#define UART_MCR_OUT2                (1 << 3)  /* Bit 3:  OUT3 */

#define UART_MCR_LPBK                (1 << 4)  /* Bit 4:  Loopback Mode Select */

#define UART_MCR_AFCE                (1 << 5)  /* Bit 5:  Enable auto flow control */
#define UART_MCR_SIRE                (1 << 6)  /* Bit 6:  SIR Mode Enable */
                                               /* Bits 7-31: Reserved */

/* LSR Line Status Register (all) */

#define UART_LSR_RDR                 (1 << 0)  /* Bit 0:  Receiver Data Ready */
#define UART_LSR_OE                  (1 << 1)  /* Bit 1:  Overrun Error */
#define UART_LSR_PE                  (1 << 2)  /* Bit 2:  Parity Error */
#define UART_LSR_FE                  (1 << 3)  /* Bit 3:  Framing Error */
#define UART_LSR_BI                  (1 << 4)  /* Bit 4:  Break Interrupt */
#define UART_LSR_THRE                (1 << 5)  /* Bit 5:  Transmitter Holding Register Empty */
#define UART_LSR_TEMT                (1 << 6)  /* Bit 6:  Transmitter Empty */
#define UART_LSR_RXFE                (1 << 7)  /* Bit 7:  Error in RX FIFO (RXFE) */
                                               /* Bits 8-31: Reserved */

/* MSR Modem Status Register (UART1 only) */

#define UART_MSR_DCTS                (1 << 0)  /* Bit 0:  CTS state change */
#define UART_MSR_DDSR                (1 << 1)  /* Bit 1:  DSR state change */
#define UART_MSR_TERI                (1 << 2)  /* Bit 2:  RI ow to high transition */
#define UART_MSR_DDCD                (1 << 3)  /* Bit 3:  DCD state change */
#define UART_MSR_CTS                 (1 << 4)  /* Bit 4:  CTS State */
#define UART_MSR_DSR                 (1 << 5)  /* Bit 5:  DSR State */
#define UART_MSR_RI                  (1 << 6)  /* Bit 6:  Ring Indicator State */
#define UART_MSR_DCD                 (1 << 7)  /* Bit 7:  Data Carrier Detect State */
                                               /* Bits 8-31: Reserved */


/* SCR Scratch Pad Register (all) */

#define UART_SCR_MASK                (0xff)    /* Bits 0-7: SCR data */
                                               /* Bits 8-31: Reserved */

///////////////////////////////////////////////////////////////////////////

#endif

#endif /* __ARCH_ARM_SRC_AR_CHIP_AR8020_UART_H */
