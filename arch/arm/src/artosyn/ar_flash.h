/****************************************************************************
 * arch/arm/src/artosyn/ar_exti.h
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#pragma once

#ifndef __ARCH_ARM_SRC_ARTOSYN_AR_FLASH_H
#define __ARCH_ARM_SRC_ARTOSYN_AR_FLASH_H



/****************************************************************************
 * Public Data
 ****************************************************************************/

__BEGIN_DECLS

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define SPI_FLASH_MAX_ID_LEN                3
#define MIN_PROTECT_SIZE                    (16 * 1024 *1024)

#ifndef CONFIG_SPI_FLASH_GIGADEVICE
#define CONFIG_SPI_FLASH_GIGADEVICE
#define SPI_FLASH_MFR_GIGADEVICE            0xC8
#endif


#ifndef CONFIG_SPI_FLASH_MACRONIX
#define CONFIG_SPI_FLASH_MACRONIX
#define SPI_FLASH_MFR_MACRONIX              0xc2
#endif

#ifndef CONFIG_SPI_FLASH_WINBOND
#define CONFIG_SPI_FLASH_WINBOND
#define SPI_FLASH_MFR_WINBOND               0xef
#endif


#define RC_ID_SIZE                      (5)
#define VT_ID_SIZE                      (2)

#define RDWR_SECTOR_SIZE                    (0x1000)
#define RDWR_BLOCK_SIZE                     (0x10000)

#define SECT_4K                             1<<0  /* CMD_ERASE_4K works uniformly */
#define E_FSR                               1<<1  /* use flag status register for */
#define SST_WR                              1<<2  /* use SST byte/word programming */
#define WR_QPP                              1<<3  /* use Quad Page Program */
#define RD_QUAD                             1<<4  /* use Quad Read */
#define RD_DUAL                             1<<5  /* use Dual Read */
#define RD_QUADIO                           1<<6  /* use Quad IO Read */
#define RD_DUALIO                           1<<7  /* use Dual IO Read */
#define RD_FULL                             (RD_QUAD | RD_DUAL | RD_QUADIO | RD_DUALIO)

#define BP0                                 1<<2  /* BP0 */
#define BP1                                 1<<3  /* BP1 */
#define BP2                                 1<<4  /* BP2 */
#define BP3                                 1<<5  /* BP3 */
#define BP4                                 1<<6  /* BP4 */
#define TB                                  1<<5  /* TB */
#define TB_1                                1<<6  /* TB_1 */
#define SEC                                 1<<6  /* SEC */

#define CMP                                 1<<6  /* CMP */

struct spi_flash_info
{
    /* Device name ([MANUFLETTER][DEVTYPE][DENSITY][EXTRAINFO]) */
    const char    *name;

    /*
     * This array stores the ID bytes.
     * The first three bytes are the JEDIC ID.
     * JEDEC ID zero means "no ID" (mostly older chips).
     */
    uint8_t         id[SPI_FLASH_MAX_ID_LEN];
    uint8_t         id_len;

    /*
     * The size listed here is what works with SPINOR_OP_SE, which isn't
     * necessarily called a "sector" by the vendor.
     */
    uint32_t        sector_size;
    uint32_t        n_sectors;
    uint32_t        protect_start;// start address of protect
    uint32_t        protect_end;// end address of protect

    uint16_t        page_size;

    uint16_t        flags;

    uint8_t         s1_map;// Status register1
    uint8_t         s2_map;// Status register2

    uint8_t         s1_wp_map;// Status register1
    uint8_t         s2_wp_map;// Status register2
};

struct spi_flash
{
    const char *  name;
    uint8_t         id[SPI_FLASH_MAX_ID_LEN];
    uint8_t         dual_flash;
    uint8_t         shift;
    uint16_t        flags;

    uint32_t        size;
    uint32_t        page_size;
    uint32_t        sector_size;
    uint32_t        erase_size;
    uint32_t        protect_start;
    uint32_t        protect_end;
    uint8_t         s1_map;
    uint8_t         s2_map;
    uint8_t         s1_wp_map;
    uint8_t         s2_wp_map;
};

typedef enum 
{
    ENUM_ID_TYPE_RCID = 0,
    ENUM_ID_TYPE_VTID = 1,
    // ENUM_ID_TYPE_CHIPID = 2,
    ENUM_ID_TYPE_ALL  = 2,

} ENUM_ID_TYPE;

typedef enum
{
    NV_NUM_RCID = 1,
    NV_NUM_CCHIPID,
    NV_NUM_NOTICE = 0xFF,
} ENUM_NV_NUM;

typedef struct
{
    uint8_t u8_nvSrc  : 4;
    uint8_t u8_nvDst  : 4; // cpu2
    ENUM_NV_NUM e_nvNum;
    // uint8_t u8_nvPar[SYS_EVENT_HANDLER_PARAMETER_LENGTH - 2];
    uint8_t u8_nvPar[14];
} STRU_SysEvent_NvMsg;


typedef enum
{
    INTER_CORE_CPU0_ID = 1,
    INTER_CORE_CPU1_ID = 2,
    INTER_CORE_CPU2_ID = 4,

}INTER_CORE_CPU_ID;

void ar_flash_init(void);

void ar_flash_update_id(uint8_t *buffer, uint8_t length, int type);


__END_DECLS

#endif
