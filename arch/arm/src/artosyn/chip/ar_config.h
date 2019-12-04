/****************************************************************************************************
 * arch/arm/src/artosyn/chip/ar_config.h
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_ARTOSYN_CHIP_AR_CONFIG_H
#define __ARCH_ARM_SRC_ARTOSYN_CHIP_AR_CONFIG_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/ar_memorymap.h"

#define CPU0_CPU1_CORE_PLL_CLK       200000000                                       /* unit: MHz */
#define CPU2_CORE_PLL_CLK            166000000                                       /* unit: MHz */

#define AR_I2C0_3_CLK_MAX           (CPU0_CPU1_CORE_PLL_CLK/2)
#define AR_BUS_CLK                  (CPU0_CPU1_CORE_PLL_CLK/2)


#define ARCFG_UART_BAUD_DEF         115200
#define ARCFG_UART_PARITY_DEF       0
#define ARCFG_UART_BITS_DEF         8
#define ARCFG_UART_STOPBITS_DEF     0
#define ARCFG_UART_RXBUFF_SIZE      600
#define ARCFG_UART_TXBUFF_SIZE      1000

#define ARCFG_UART0_BAUD        ARCFG_UART_BAUD_DEF
#define ARCFG_UART0_PARITY      ARCFG_UART_PARITY_DEF
#define ARCFG_UART0_BITS        ARCFG_UART_BITS_DEF
#define ARCFG_UART0_STOPBITS    ARCFG_UART_STOPBITS_DEF
#define ARCFG_UART0_RXBUFF      ARCFG_UART_RXBUFF_SIZE
#define ARCFG_UART0_TXBUFF      ARCFG_UART_TXBUFF_SIZE

#define ARCFG_UART1_BAUD        ARCFG_UART_BAUD_DEF
#define ARCFG_UART1_PARITY      ARCFG_UART_PARITY_DEF
#define ARCFG_UART1_BITS        ARCFG_UART_BITS_DEF
#define ARCFG_UART1_STOPBITS    ARCFG_UART_STOPBITS_DEF
#define ARCFG_UART1_RXBUFF      ARCFG_UART_RXBUFF_SIZE
#define ARCFG_UART1_TXBUFF      ARCFG_UART_TXBUFF_SIZE

#define ARCFG_UART2_BAUD        ARCFG_UART_BAUD_DEF
#define ARCFG_UART2_PARITY      ARCFG_UART_PARITY_DEF
#define ARCFG_UART2_BITS        ARCFG_UART_BITS_DEF
#define ARCFG_UART2_STOPBITS    ARCFG_UART_STOPBITS_DEF
#define ARCFG_UART2_RXBUFF      ARCFG_UART_RXBUFF_SIZE
#define ARCFG_UART2_TXBUFF      ARCFG_UART_TXBUFF_SIZE

#define ARCFG_UART3_BAUD        ARCFG_UART_BAUD_DEF
#define ARCFG_UART3_PARITY      ARCFG_UART_PARITY_DEF
#define ARCFG_UART3_BITS        ARCFG_UART_BITS_DEF
#define ARCFG_UART3_STOPBITS    ARCFG_UART_STOPBITS_DEF
#define ARCFG_UART3_RXBUFF      ARCFG_UART_RXBUFF_SIZE
#define ARCFG_UART3_TXBUFF      ARCFG_UART_TXBUFF_SIZE

#define ARCFG_UART4_BAUD        ARCFG_UART_BAUD_DEF
#define ARCFG_UART4_PARITY      ARCFG_UART_PARITY_DEF
#define ARCFG_UART4_BITS        ARCFG_UART_BITS_DEF
#define ARCFG_UART4_STOPBITS    ARCFG_UART_STOPBITS_DEF
#define ARCFG_UART4_RXBUFF      ARCFG_UART_RXBUFF_SIZE
#define ARCFG_UART4_TXBUFF      ARCFG_UART_TXBUFF_SIZE

#define ARCFG_UART5_BAUD        ARCFG_UART_BAUD_DEF
#define ARCFG_UART5_PARITY      ARCFG_UART_PARITY_DEF
#define ARCFG_UART5_BITS        ARCFG_UART_BITS_DEF
#define ARCFG_UART5_STOPBITS    ARCFG_UART_STOPBITS_DEF
#define ARCFG_UART5_RXBUFF      ARCFG_UART_RXBUFF_SIZE
#define ARCFG_UART5_TXBUFF      ARCFG_UART_TXBUFF_SIZE

#define ARCFG_UART6_BAUD        ARCFG_UART_BAUD_DEF
#define ARCFG_UART6_PARITY      ARCFG_UART_PARITY_DEF
#define ARCFG_UART6_BITS        ARCFG_UART_BITS_DEF
#define ARCFG_UART6_STOPBITS    ARCFG_UART_STOPBITS_DEF
#define ARCFG_UART6_RXBUFF      ARCFG_UART_RXBUFF_SIZE
#define ARCFG_UART6_TXBUFF      ARCFG_UART_TXBUFF_SIZE

#define ARCFG_UART7_BAUD        ARCFG_UART_BAUD_DEF
#define ARCFG_UART7_PARITY      ARCFG_UART_PARITY_DEF
#define ARCFG_UART7_BITS        ARCFG_UART_BITS_DEF
#define ARCFG_UART7_STOPBITS    ARCFG_UART_STOPBITS_DEF
#define ARCFG_UART7_RXBUFF      ARCFG_UART_RXBUFF_SIZE
#define ARCFG_UART7_TXBUFF      ARCFG_UART_TXBUFF_SIZE

#define CONSOLE_DEV     g_uart0port     /* UART1=console */
// #define TTYS0_DEV       g_uart0port     /* UART1=ttyS1 */
#define TTYS1_DEV       g_uart1port     /* UART1=ttyS1 */
#define TTYS2_DEV       g_uart2port     /* UART2=ttyS2 */
#define TTYS3_DEV       g_uart3port     /* UART3=ttyS3 */
#define TTYS4_DEV       g_uart4port     /* UART4=ttyS4 */
#define TTYS5_DEV       g_uart5port     /* UART5=ttyS5 */
#define TTYS6_DEV       g_uart6port     /* UART6=ttyS6 */
#define TTYS7_DEV       g_uart7port     /* UART7=ttyS7 */


#define SYSTICK_RELOAD ((CPU0_CPU1_CORE_PLL_CLK / CLK_TCK) - 1)

#if SYSTICK_RELOAD > 0x00ffffff
#  error SYSTICK_RELOAD exceeds the range of the RELOAD register
#endif


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define SRAM_BASE_ADDRESS                             0x21000000     /* start address of SRAM */
#define SRAM_SIZE                                     (60 * 1024)    /* size of SRAM */

/* 8K video0  */
#define SRAM_BB_VIDEO_BUFFER_0_ST_ADDRESS             SRAM_BASE_ADDRESS
#define SRAM_BB_VIDEO_BUFFER_0_SIZE                   0x2000

/* 8K video1*/
#define SRAM_BB_VIDEO_BUFFER_1_ST_ADDRESS             (SRAM_BB_VIDEO_BUFFER_0_ST_ADDRESS + SRAM_BB_VIDEO_BUFFER_0_SIZE)
#define SRAM_BB_VIDEO_BUFFER_1_SIZE                   0x2000

/* 16K non-cache start, initialized by inter core module.*/

// 256 inter core message
#define SRAM_INTER_CORE_MSG_SHARE_MEMORY_ST_ADDRESS   (SRAM_BB_VIDEO_BUFFER_1_ST_ADDRESS + SRAM_BB_VIDEO_BUFFER_1_SIZE)
#define SRAM_INTER_CORE_MSG_SHARE_MEMORY_SIZE         0x2B0

// 256 module lock
#define SRAM_MODULE_LOCK_ST_ADDRESS                   (SRAM_INTER_CORE_MSG_SHARE_MEMORY_ST_ADDRESS + SRAM_INTER_CORE_MSG_SHARE_MEMORY_SIZE)
#define SRAM_MODULE_LOCK_SIZE                         0x20
#define SRAM_MODULE_LOCK_BB_UART_MUTEX_FLAG           (SRAM_MODULE_LOCK_ST_ADDRESS)
#define SRAM_MODULE_LOCK_BB_UART_INIT_FLAG            (SRAM_MODULE_LOCK_ST_ADDRESS + 4)

// 256 module share
#define SRAM_MODULE_SHARE_ST_ADDRESS                  (SRAM_MODULE_LOCK_ST_ADDRESS + SRAM_MODULE_LOCK_SIZE)
#define SRAM_MODULE_SHARE_SIZE                        0x30
#define SRAM_MODULE_SHARE_PLL_INIT_FLAG               (SRAM_MODULE_SHARE_ST_ADDRESS)
#define SRAM_MODULE_SHARE_PLL_CPU0CPU1                (SRAM_MODULE_SHARE_ST_ADDRESS + 4)
#define SRAM_MODULE_SHARE_PLL_CPU2                    (SRAM_MODULE_SHARE_ST_ADDRESS + 8)
#define SRAM_MODULE_SHARE_AUDIO_PCM                   (SRAM_MODULE_SHARE_ST_ADDRESS + 12)
#define SRAM_MODULE_SHARE_AUDIO_RATE                  (SRAM_MODULE_SHARE_ST_ADDRESS + 16)
#define SRAM_MODULE_SHARE_AVSYNC_TICK                 (SRAM_MODULE_SHARE_ST_ADDRESS + 20)
#define SRAM_MODULE_WATCHDOG_BASE_TICK                (SRAM_MODULE_SHARE_ST_ADDRESS + 24)
#define SRAM_MODULE_WATCHDOG_CPU1_TICK                (SRAM_MODULE_SHARE_ST_ADDRESS + 28)
#define SRAM_MODULE_WATCHDOG_CPU2_TICK                (SRAM_MODULE_SHARE_ST_ADDRESS + 32)

// 512 bb status
#define SRAM_BB_STATUS_SHARE_MEMORY_ST_ADDR           (SRAM_MODULE_SHARE_ST_ADDRESS + SRAM_MODULE_SHARE_SIZE)
#define SRAM_BB_STATUS_SHARE_MEMORY_SIZE              0x200

// 1K bb uart com for each session RX buffer
#define SRAM_BB_UART_COM_SESSION_1_SHARE_MEMORY_ST_ADDR    (SRAM_BB_STATUS_SHARE_MEMORY_ST_ADDR + SRAM_BB_STATUS_SHARE_MEMORY_SIZE)
#define SRAM_BB_UART_COM_SESSION_1_SHARE_MEMORY_SIZE       0x400
#define SRAM_BB_UART_COM_SESSION_2_SHARE_MEMORY_ST_ADDR    (SRAM_BB_UART_COM_SESSION_1_SHARE_MEMORY_ST_ADDR + SRAM_BB_UART_COM_SESSION_1_SHARE_MEMORY_SIZE)
#define SRAM_BB_UART_COM_SESSION_2_SHARE_MEMORY_SIZE       0x400
#define SRAM_BB_UART_COM_SESSION_3_SHARE_MEMORY_ST_ADDR    (SRAM_BB_UART_COM_SESSION_2_SHARE_MEMORY_ST_ADDR + SRAM_BB_UART_COM_SESSION_2_SHARE_MEMORY_SIZE)
#define SRAM_BB_UART_COM_SESSION_3_SHARE_MEMORY_SIZE       0x400
#define SRAM_BB_UART_COM_SESSION_4_SHARE_MEMORY_ST_ADDR    (SRAM_BB_UART_COM_SESSION_3_SHARE_MEMORY_ST_ADDR + SRAM_BB_UART_COM_SESSION_3_SHARE_MEMORY_SIZE)
#define SRAM_BB_UART_COM_SESSION_4_SHARE_MEMORY_SIZE       0x400

//4K bb com TX buffer with High priority
#define SRAM_BB_COM_TX_HIGH_PRIO_SHARE_MEMORY_ST_ADDR  (SRAM_BB_UART_COM_SESSION_4_SHARE_MEMORY_ST_ADDR + SRAM_BB_UART_COM_SESSION_4_SHARE_MEMORY_SIZE)
#define SRAM_BB_COM_TX_HIGH_PRIO_SHARE_MEMORY_SIZE     0x1000

//4K bb com TX buffer with Low priority
#define SRAM_BB_COM_TX_LOW_PRIO_SHARE_MEMORY_ST_ADDR   (SRAM_BB_COM_TX_HIGH_PRIO_SHARE_MEMORY_ST_ADDR + SRAM_BB_COM_TX_HIGH_PRIO_SHARE_MEMORY_SIZE)
#define SRAM_BB_COM_TX_LOW_PRIO_SHARE_MEMORY_SIZE      0x1000

// 256 bytes, nonvolatile variable,stored in flash
#define SRAM_NV_MEMORY_ST_ADDR       (SRAM_BB_COM_TX_LOW_PRIO_SHARE_MEMORY_ST_ADDR + SRAM_BB_COM_TX_LOW_PRIO_SHARE_MEMORY_SIZE)
#define SRAM_NV_MEMORY_SIZE          (0x100)

// 256 byte for periperial mutex reserved
#define SRAM_PERIPERIAL_MUTEX_ADDR       (SRAM_NV_MEMORY_ST_ADDR + SRAM_NV_MEMORY_SIZE)
#define SRAM_PERIPERIAL_MUTEX_SIZE       64

// 64 byte for bb data transmission buffer(grd -> sky)

#define SRAM_BB_DT_ST_ADDR          (SRAM_PERIPERIAL_MUTEX_ADDR + SRAM_PERIPERIAL_MUTEX_SIZE)
#define SRAM_BB_DT_SIZE             (64)

// 32 byte for share flag
#define SRAM_SHARE_FLAG_ST_ADDR    (SRAM_BB_DT_ST_ADDR + SRAM_BB_DT_SIZE)
#define SRAM_SHARE_FLAG_SIZE       (32)
#define SHARE_FLAG_RC_ID_OFFSET    (0)
#define SHARE_FLAG_CHIP_ID_OFFSET  (5)

// 32 byte for spi data trans
#define SRAM_SPI_DATA_TRANS_ST_ADDR    (SRAM_SHARE_FLAG_ST_ADDR + SRAM_SHARE_FLAG_SIZE)
#define SRAM_SPI_DATA_TRANS_SIZE       (32)

// 2K usr nv data buffer
#define SRAM_USR_NV_MEMORY_ST_ADDR  (SRAM_SPI_DATA_TRANS_ST_ADDR + SRAM_SPI_DATA_TRANS_SIZE)
#define SRAM_USR_NV_MEMORY_SIZE     (0x800)

// 32 byte for SKY or Grd select
#define SRAM_USR_GRD_SKY_SELECT_ST_ADDR  (SRAM_USR_NV_MEMORY_ST_ADDR + SRAM_USR_NV_MEMORY_SIZE)
#define SRAM_USR_GRD_SKY_SELECT_SIZE     (32)

// 4 byte for log event
#define SRAM_INTERCORE_EVENT_CPU2T0_COUNT_FLAG (SRAM_USR_GRD_SKY_SELECT_ST_ADDR + SRAM_USR_GRD_SKY_SELECT_SIZE)
#define SRAM_INTERCORE_EVENT_CPU2T0_COUNT_SIZE     (4)

// 4 byte for cpu0 to cpu2 event
#define SRAM_INTERCORE_EVENT_CPU0T2_COUNT_FLAG (SRAM_INTERCORE_EVENT_CPU2T0_COUNT_FLAG + SRAM_INTERCORE_EVENT_CPU2T0_COUNT_SIZE)
#define SRAM_INTERCORE_EVENT_CPU0T2_COUNT_SIZE     (4)


#define SRAM_CONFIGURE_MEMORY_ST_ADDR    (SRAM_BASE_ADDRESS + 0x8700)
#define SRAM_CONFIGURE_MEMORY_SIZE       (0x1000)
#define CONFIGURE_INIT_FLAG_VALUE        SRAM_CONFIGURE_MEMORY_ST_ADDR

#define GET_WORD_FROM_ANY_ADDR(any_addr) ((uint32_t)(*any_addr) | \
                                         (((uint32_t)(*(any_addr+1))) << 8) | \
                                         (((uint32_t)(*(any_addr+2))) << 16) | \
                                         ((uint32_t)((*(any_addr+3))) << 24))

// 256 for rc info 
#define SRAM_MAVLINK_RC_MSG_ST_ADDR     (SRAM_CONFIGURE_MEMORY_ST_ADDR + SRAM_CONFIGURE_MEMORY_SIZE)
#define SRAM_MAVLINK_RC_MSG_SIZE        (256)
#define SRAM_MAVLINK_RC_MSG_END_ADDR    (SRAM_MAVLINK_RC_MSG_ST_ADDR + SRAM_MAVLINK_RC_MSG_SIZE - 1)


// 1k for sesion1 data to uart
#define SRAM_SESSION1_TO_UART_DATA_ST_ADDR          (SRAM_MAVLINK_RC_MSG_ST_ADDR + SRAM_MAVLINK_RC_MSG_SIZE)
#define SRAM_SESSION1_TO_UART_DATA_SIZE             (0x400) 
#define SRAM_SESSION1_TO_UART_DATA_END_ADDR         (SRAM_SESSION1_TO_UART_DATA_ST_ADDR + SRAM_SESSION1_TO_UART_DATA_SIZE - 1)

// 1k for uart data to sesion1
#define SRAM_UART_TO_SESSION1_DATA_ST_ADDR          (SRAM_SESSION1_TO_UART_DATA_ST_ADDR + SRAM_SESSION1_TO_UART_DATA_SIZE)
#define SRAM_UART_TO_SESSION1_DATA_SIZE             (0x400) 
#define SRAM_UART_TO_SESSION1_DATA_END_ADDR         (SRAM_UART_TO_SESSION1_DATA_ST_ADDR + SRAM_UART_TO_SESSION1_DATA_SIZE - 1)

// 4 bytes for reboot signal
#define SRAM_REBOOT_SIGNAL_ST_ADDR      (SRAM_UART_TO_SESSION1_DATA_ST_ADDR + SRAM_UART_TO_SESSION1_DATA_SIZE)
#define SRAM_REBOOT_SIGNAL_SIZE         (4)
#define SRAM_REBOOT_SIGNAL_END_ADDR     (SRAM_REBOOT_SIGNAL_ST_ADDR + SRAM_REBOOT_SIGNAL_SIZE - 1)

// nonvolatile variable management struct
typedef struct
{
    uint8_t u8_nvChg;    // TRUE: some nv changed,FALSE: nv not change.
    uint8_t u8_nvPrc;    // TRUE: nv in writing flash,FLASE: not being write flash
    uint8_t u8_nvUpd;    // TRUE: nv in updating,FLASE: not being update
    uint8_t u8_nvVld;    // TRUE: nv is valid,FALSE: nv is invalid,and set to default value.
    uint32_t u32_nvInitFlag; //0x23178546 nv have inited.
}STRU_NV_MNG;

// nonvolatile variable data struct
typedef struct
{
    uint8_t u8_nvChk            __attribute__ ((aligned (4)));  //RC id data checksum,in bytes.
    uint8_t u8_nvBbRcId[5]      __attribute__ ((aligned (4)));  //RC id
    uint8_t u8_nvBbVtId[2]      __attribute__ ((aligned (4)));  //Vt id, use 2bytes
    uint8_t u8_nvChipId[5]      __attribute__ ((aligned (4)));  //chip id
    uint8_t u8_reserve[3]       __attribute__ ((aligned (4)));  //reserve
}STRU_NV_DATA;


typedef struct
{
    STRU_NV_MNG  st_nvMng;
    STRU_NV_DATA st_nvDataUpd; // use to update nv in sram
    STRU_NV_DATA st_nvDataPrc; // use to write nv to flash
}STRU_NV;

typedef struct
{
#define     USR_NV_UNIT_MAX_DATA_LEN    (130)

    uint32_t    addr;     //
    uint8_t     valid;    // 1:valid,0:invalid
    uint8_t     len;      //
    uint8_t     data[USR_NV_UNIT_MAX_DATA_LEN];
}STRU_USR_NV_UNIT;


typedef struct
{
    volatile uint32_t buf_wr_pos;
    volatile uint32_t buf_rd_pos;
    volatile uint32_t buf_init_flag;
} STRU_SramBufferHeader;

typedef struct
{
    volatile STRU_SramBufferHeader header           __attribute__ ((aligned (4)));
    volatile char buf[1]                            __attribute__ ((aligned (4)));
} STRU_SramBuffer;


// 100 for write header
#define SRAM_MAVLINK_INTERCORE_WR_HEADER_ST_ADDR        SRAM_MAVLINK_INTERCORE_BUFFER_ST_ADDR
#define SRAM_MAVLINK_INTERCORE_WR_HEADER_SIZE           100

// 2k for write buffer 
#define SRAM_MAVLINK_INTERCORE_WR_BUFFER                (SRAM_MAVLINK_INTERCORE_WR_HEADER_ST_ADDR +　SRAM_MAVLINK_INTERCORE_WR_HEADER_SIZE)
#define SRAM_MAVLINK_INTERCORE_WR_SIZE                  0x800

// 100 for read header 
#define SRAM_MAVLINK_INTERCORE_RD_HEADER_ST_ADDR        (SRAM_MAVLINK_INTERCORE_WR_BUFFER + SRAM_MAVLINK_INTERCORE_WR_SIZE)
#define SRAM_MAVLINK_INTERCORE_RD_HEADER_SIZE           100

// 2k for read buffer
#define SRAM_MAVLINK_INTERCORE_RD_BUFFER                (SRAM_MAVLINK_INTERCORE_RD_HEADER_ST_ADDR + SRAM_MAVLINK_INTERCORE_RD_HEADER_SIZE)
#define SRAM_MAVLINK_INTERCORE_RD_SIZE                  0x800

/*  3k for cpu0 to cpu2 log **/
#define SRAM_INTERCORE_EVENT_ST_SIZE    0xC00

#define SRAM_INTERCORE_EVENT_CPU0T2_ST_STARTADDR              (SRAM_BASE_ADDRESS + SRAM_SIZE - (14 * 1024))
#define SRAM_INTERCORE_EVENT_CPU0T2_ST_SIZE                   (SRAM_INTERCORE_EVENT_ST_SIZE)

/*  3k for cpu2 to cpu0 log **/
#define SRAM_INTERCORE_EVENT_CPU2T0_ST_STARTADDR              (SRAM_BASE_ADDRESS + SRAM_SIZE - (11 * 1024)) 
#define SRAM_INTERCORE_EVENT_CPU2T0_ST_SIZE                   (SRAM_INTERCORE_EVENT_ST_SIZE)

// Intercore buffer
#define SRAM_MAVLINK_INTERCORE_BUFFER_ST_ADDR   (SRAM_BASE_ADDRESS + SRAM_SIZE - (19 * 1024)) // 0x2100E000
#define SRAM_MAVLINK_INTERCORE_BUFFER_SIZE      (5 * 1024)

/* 3.5K debug log buffer at the end of the SRAM. */
#define SRAM_DEBUG_LOG_BUFFER_ST_ADDR              (SRAM_BASE_ADDRESS + SRAM_SIZE - (8 * 1024))

// 512 debug log input buffer
#define SRAM_DEBUG_LOG_INPUT_BUFFER_ST_ADDR        SRAM_DEBUG_LOG_BUFFER_ST_ADDR
#define SRAM_DEBUG_LOG_INPUT_BUFFER_SIZE           0x200

// 1K * 3 debug log output buffer
#define SRAM_DEBUG_LOG_OUTPUT_BUFFER_ST_ADDR_0  (SRAM_DEBUG_LOG_INPUT_BUFFER_ST_ADDR + SRAM_DEBUG_LOG_INPUT_BUFFER_SIZE)
#define SRAM_DEBUG_LOG_OUTPUT_BUFFER_END_ADDR_0 (SRAM_DEBUG_LOG_OUTPUT_BUFFER_ST_ADDR_0 + 0xA00 - 1)

#define SRAM_DEBUG_LOG_OUTPUT_BUFFER_ST_ADDR_1  (SRAM_DEBUG_LOG_OUTPUT_BUFFER_END_ADDR_0 + 1)
#define SRAM_DEBUG_LOG_OUTPUT_BUFFER_END_ADDR_1 (SRAM_DEBUG_LOG_OUTPUT_BUFFER_ST_ADDR_1 + 0xA00 - 1)

#define SRAM_DEBUG_LOG_OUTPUT_BUFFER_ST_ADDR_2  (SRAM_DEBUG_LOG_OUTPUT_BUFFER_END_ADDR_1 + 1)
#define SRAM_DEBUG_LOG_OUTPUT_BUFFER_END_ADDR_2 (SRAM_DEBUG_LOG_OUTPUT_BUFFER_ST_ADDR_2 + 0xA00 - 1)


/****************************************************************************
 * Inline Functions
 ****************************************************************************/

# define ar_getreg8(a)           (*(volatile uint8_t *)(a))
# define ar_putreg8(v,a)         (*(volatile uint8_t *)(a) = (v))
# define ar_getreg16(a)          (*(volatile uint16_t *)(a))
# define ar_putreg16(v,a)        (*(volatile uint16_t *)(a) = (v))
# define ar_getreg32(a)          (*(volatile uint32_t *)(a))
# define ar_putreg32(v,a)        (*(volatile uint32_t *)(a) = (v))




#ifdef __cplusplus
}
#endif /* __cplusplus */



#endif /* __ARCH_ARM_SRC_ARTOSYN_CHIP_AR_CONFIG_H */
