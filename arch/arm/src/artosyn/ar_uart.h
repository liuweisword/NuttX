/************************************************************************************
 * arch/arm/src/artosyn/ar_uart.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_ARTOSYN_AR_UART_H
#define __ARCH_ARM_SRC_ARTOSYN_AR_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/serial/serial.h>
#include "chip/ar_uart.h"


#define AR_UART_BAUD_RATE   115200

#define AR_UART_CONSOLE_LOG

typedef struct {
  unsigned int RBR_THR_DLL;
  unsigned int DLH_IER;
  unsigned int IIR_FCR;
  unsigned int LCR;
  unsigned int MCR;
  unsigned int LSR;
  unsigned int MSR;
  unsigned int SCR;
  unsigned int LPDLL;
  unsigned int LPDLH;
  unsigned int reserv[2];
  unsigned int SRBR_STHR[16];
  unsigned int UARTFAR;
  unsigned int TFR;
  unsigned int RFW;
  unsigned int USR;
  unsigned int TFL;
  unsigned int RFL;
  unsigned int SRR;
  unsigned int SRTS;
  unsigned int SBCR;
  unsigned int SDMAM;
  unsigned int SFE;
  unsigned int SRT;
  unsigned int STET;
  unsigned int HTX;
  unsigned int DMASA;
  unsigned int reserv1[14];
  unsigned int CPR;
  unsigned int UCV;
  unsigned int CTR;
} uart_type;

#define HAVE_CONSOLE 1

#define UART_MINDL 32

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif


#undef EXTERN
#if defined(__cplusplus)
}
#endif


#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_ARTOSYN_AR_UART_H */
