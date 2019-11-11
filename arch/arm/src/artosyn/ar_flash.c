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

#include <stdlib.h>
#include <string.h>
#include <nuttx/config.h>
#include <sys/types.h>
#include <stdio.h>
#include <debug.h>

#include "chip/ar8020_memorymap.h"
#include "ar_flash.h"
#include "chip/ar_config.h"
#include "ar_qspi.h"
#include "chip/ar8020_core_m7.h"


struct spi_flash    g_norflash;
uint32_t            g_protect_size = MIN_PROTECT_SIZE;
uint8_t             u8_WP_Disable_Flag = 0;

#define NV_FLASH_RCVTCHIP_ADDR     (0x8000)
#define NV_FLASH_RCVTCHIP_SIZE     (64)

#define NV_FLASH_RCVTCHIP_ADDR1     NV_FLASH_RCVTCHIP_ADDR
#define NV_FLASH_RCVTCHIP_SIZE1    NV_FLASH_RCVTCHIP_SIZE

#define NV_FLASH_RCVTCHIP_ADDR2     (NV_FLASH_RCVTCHIP_ADDR1 + NV_FLASH_RCVTCHIP_SIZE1)
#define NV_FLASH_RCVTCHIP_SIZE2     NV_FLASH_RCVTCHIP_SIZE





#define INFO(_jedec_id, _sector_size, _n_sectors, _flags, _pro_start, _pro_end, _s1, _s2, _s1_wp, _s2_wp)    \
        .id = {                                    \
            ((_jedec_id) >> 16) & 0xff,            \
            ((_jedec_id) >> 8) & 0xff,             \
            (_jedec_id) & 0xff,                    \
            },                                     \
        .id_len = (!(_jedec_id) ? 0 : 3),          \
        .sector_size = (_sector_size),             \
        .n_sectors = (_n_sectors),                 \
        .protect_start = (_pro_start),             \
        .protect_end = (_pro_end),                 \
        .page_size = 256,                          \
        .s1_map = _s1,                             \
        .s2_map = _s2,                             \
        .s1_wp_map = _s1_wp,                       \
        .s2_wp_map = _s2_wp,                       \
        .flags = (_flags),


const struct spi_flash_info spi_flash_ids[] = {
#ifdef CONFIG_SPI_FLASH_ATMEL        /* ATMEL */

#endif
#ifdef CONFIG_SPI_FLASH_EON          /* EON */

#endif
#ifdef CONFIG_SPI_FLASH_GIGADEVICE   /* GIGADEVICE */
    {"gd25q127c",       INFO(0xc84018,   64 * 1024,   256, SECT_4K, 0, 0x3fffff, BP3 | BP2 | BP0, 0, BP4 | BP3 | BP2 | BP1 | BP0, CMP) },
#endif
#ifdef CONFIG_SPI_FLASH_ISSI         /* ISSI */

#endif
#ifdef CONFIG_SPI_FLASH_MACRONIX     /* MACRONIX */
    {"mx25l3205d",      INFO(0xc22016,   64 * 1024,    64, RD_FULL | WR_QPP | SECT_4K, 0, 0x37ffff, BP3 | BP1 | BP0, 0, BP3 | BP2 | BP1 | BP0, 0) },
#endif
#ifdef CONFIG_SPI_FLASH_SPANSION     /* SPANSION */

#endif
#ifdef CONFIG_SPI_FLASH_STMICRO      /* STMICRO */

#endif
#ifdef CONFIG_SPI_FLASH_SST          /* SST */

#endif
#ifdef CONFIG_SPI_FLASH_WINBOND      /* WINBOND */
    {"w25q32bv",        INFO(0xef4016,   64 * 1024,    64, RD_FULL | WR_QPP | SECT_4K, 0, 0x37ffff, BP2, CMP, BP0 | BP1 | BP2 | TB | SEC, CMP) },
    {"w25q32jv",        INFO(0xef7016,   64 * 1024,    64, RD_FULL | WR_QPP | SECT_4K, 0, 0x37ffff, BP2, CMP, BP0 | BP1 | BP2 | TB | SEC, CMP) },
    {"w25q128bv",       INFO(0xef4018,   64 * 1024,   256, RD_FULL | WR_QPP | SECT_4K, 0, 0x7ffffff, BP2| BP1 | BP0, 0, BP0 | BP1 | BP2 | TB | SEC, CMP) },    
    {"w25q128jv",       INFO(0xef7018,   64 * 1024,   256, RD_FULL | WR_QPP | SECT_4K, 0, 0x7ffffff, BP2| BP1 | BP0, 0, BP0 | BP1 | BP2 | TB | SEC, CMP) },
#endif
    {},    /* Empty entry to terminate the list */
};


_EXT_ITCM void NOR_FLASH_SetFlashDisableFlag(uint8_t flag)
{
    u8_WP_Disable_Flag = flag;
}

_EXT_ITCM uint8_t NOR_FLASH_GetFlashDisableFlag(void)
{
    return u8_WP_Disable_Flag;
}

/*Winbond, Macronix, GIGADEVICE*/
_EXT_ITCM static uint8_t NOR_FLASH_Common_WPEnable(struct spi_flash flash_status, uint8_t enable_flag)
{
    uint32_t u32_reg = 0;

    if(0 != flash_status.s1_map)
    {
        ar_qspi_write_enable();
        ar_qspi_check_busy();

        u32_reg =  ar_qspi_read_reg1_status();
        if(1 == enable_flag)
        {
            u32_reg |= flash_status.s1_map;
        }
        else
        {
            u32_reg &= (~flash_status.s1_wp_map);
        }
        ar_qspi_set_reg1_status(u32_reg);

        ar_qspi_check_busy();
    }
    
    if(0 != flash_status.s2_map)
    {
        ar_qspi_write_enable();
        ar_qspi_check_busy();

        u32_reg =  ar_qspi_read_reg2_status();
        if(1 == enable_flag)
        {
            u32_reg |= flash_status.s2_map;
        }
        else
        {
            u32_reg &= (~flash_status.s2_wp_map);
        }
        ar_qspi_set_reg2_status(u32_reg);

        ar_qspi_check_busy();
    }
    return 0;
}


_EXT_ITCM uint8_t NOR_FLASH_WP_Enable(struct spi_flash flash_status, uint8_t enable_flag, uint32_t start_addr, uint32_t size)
{
    if(1 == enable_flag && 1 == u8_WP_Disable_Flag)
    {
        flashinfo("u8_WP_Disable_Flag = 1 \n");
        return 1;// Can't enable WP
    }

    if((NULL == flash_status.name) || (0 == g_norflash.protect_end))
    {
        return 0;
    }

    switch(flash_status.id[0])
    {
        case SPI_FLASH_MFR_GIGADEVICE:
        case SPI_FLASH_MFR_MACRONIX:
        case SPI_FLASH_MFR_WINBOND:
            NOR_FLASH_Common_WPEnable(flash_status, enable_flag);
            break;
        default:
            return 1;
    }

    return 0;
}

_EXT_ITCM static int32_t NV_CalChk(STRU_NV_DATA *pst_nvData, uint8_t *u8_chk)
{
    uint32_t u32_nvDataLen;
    uint32_t u32_i;
    uint8_t u8_checkSum = 0;
    uint8_t *p_u8Addr;

    u32_nvDataLen = sizeof(STRU_NV_DATA);
    u32_i = sizeof(pst_nvData->u8_nvChk);
    p_u8Addr = (uint8_t *)pst_nvData;

    //calculate checkSum
    for (; u32_i < u32_nvDataLen; u32_i++)
    {
        u8_checkSum += *(p_u8Addr + u32_i);
    }  

    *u8_chk = u8_checkSum;

    return 0;
}

_EXT_ITCM static int32_t NV_CheckValidity(STRU_NV_DATA *pst_nvData)
{
    uint8_t u8_checkSum;
    uint32_t u32_nvDataLen;
    uint32_t u32_i = 0;
    uint8_t *p_u8Addr;

    u32_nvDataLen = sizeof(STRU_NV_DATA);
    p_u8Addr = (uint8_t *)pst_nvData;
    
    while (u32_i < u32_nvDataLen)
    {
        if (0xFF != p_u8Addr[u32_i])
        {
            break;
        }
        u32_i += 1;
    }

    NV_CalChk(pst_nvData, &u8_checkSum);

    if (((pst_nvData->u8_nvChk) == u8_checkSum) && (u32_i < u32_nvDataLen))
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

_EXT_ITCM static int32_t ar_nv_read_from_flash(uint32_t u32_nvFlashAddr, STRU_NV_DATA *pst_nvDataPrc)
{
    ar_qspi_read_block_by_byte(u32_nvFlashAddr, (uint8_t *)pst_nvDataPrc, sizeof(STRU_NV_DATA));

    return 0;
}

_EXT_ITCM static int32_t ar_flash_copyRcIdVtId_to_sram(void)
{
    int32_t result = -1;
    STRU_NV *pst_nv = (STRU_NV *)SRAM_NV_MEMORY_ST_ADDR;

    ar_nv_read_from_flash(NV_FLASH_RCVTCHIP_ADDR1, &(pst_nv->st_nvDataPrc));

    if (0 == NV_CheckValidity(&(pst_nv->st_nvDataPrc)))
    {
        memcpy((uint8_t *)(&(pst_nv->st_nvDataUpd)), 
               (uint8_t *)(&(pst_nv->st_nvDataPrc)), 
               sizeof(STRU_NV_DATA));
        result = 0;
    }


    flashinfo("rc id:%x %x %x %x %x, vt id:%x %x, chip id:%x %x %x %x %x \n",
        pst_nv->st_nvDataPrc.u8_nvBbRcId[0],pst_nv->st_nvDataPrc.u8_nvBbRcId[1],pst_nv->st_nvDataPrc.u8_nvBbRcId[2],pst_nv->st_nvDataPrc.u8_nvBbRcId[3],pst_nv->st_nvDataPrc.u8_nvBbRcId[4],
        pst_nv->st_nvDataPrc.u8_nvBbVtId[0],pst_nv->st_nvDataPrc.u8_nvBbVtId[1],
        pst_nv->st_nvDataPrc.u8_nvChipId[0],pst_nv->st_nvDataPrc.u8_nvChipId[1],pst_nv->st_nvDataPrc.u8_nvChipId[2],pst_nv->st_nvDataPrc.u8_nvChipId[3],pst_nv->st_nvDataPrc.u8_nvChipId[4]);


    memcpy((uint8_t *)(SRAM_SHARE_FLAG_ST_ADDR + SHARE_FLAG_CHIP_ID_OFFSET), pst_nv->st_nvDataUpd.u8_nvChipId, 5);

    return result;
}

_EXT_ITCM static int32_t ar_flash_copyRcIdVtId_init(void)
{
    STRU_NV *pst_nv = (STRU_NV *)SRAM_NV_MEMORY_ST_ADDR;

    if (-1 == ar_flash_copyRcIdVtId_to_sram())
    {
        pst_nv->st_nvMng.u8_nvVld = FALSE;
    }    
    else
    {
        pst_nv->st_nvMng.u8_nvVld = TRUE;
    }
    return OK;
}


_EXT_ITCM void NOR_FLASH_WriteEnable(uint32_t start_addr, uint32_t size)
{

    NOR_FLASH_WP_Enable(g_norflash, 0, start_addr, size);
    
    ar_qspi_write_enable();
    
    ar_qspi_check_busy();
    return;
}

_EXT_ITCM void NOR_FLASH_WriteDisable(uint32_t start_addr, uint32_t size)
{
    ar_qspi_check_busy();

    NOR_FLASH_WP_Enable(g_norflash, 1, start_addr, size);

    ar_qspi_write_disable();

    ar_qspi_check_busy();

    return;
}

// 擦除
_EXT_ITCM uint8_t NOR_FLASH_EraseSector(uint32_t flash_start_addr)
{
    uint32_t sector_size = 0x1000;

#ifdef USE_WINBOND_SPI_NOR_FLASH
    sector_size = W25Q128_SECTOR_SIZE;
#endif

    if ((flash_start_addr % sector_size) != 0)
    {
        flashinfo("The w25q128 sector erase address is not sector aligned! \n");
        return FALSE;
    }
    
    NOR_FLASH_WriteEnable(flash_start_addr, sector_size);
    ar_qspi_erase_sector(flash_start_addr);
    NOR_FLASH_WriteDisable(flash_start_addr, sector_size);

    return TRUE;
}

_EXT_ITCM uint8_t NOR_FLASH_EraseBlock(uint32_t flash_start_addr)
{
    uint32_t sector_size = 0x10000;

#ifdef USE_WINBOND_SPI_NOR_FLASH
    sector_size = W25Q128_BLOCK_SIZE;
#endif

    if ((flash_start_addr % sector_size) != 0)
    {
        return FALSE;
    }

    NOR_FLASH_WriteEnable(flash_start_addr, sector_size);
    ar_qspi_erase_block(flash_start_addr);
    NOR_FLASH_WriteDisable(flash_start_addr, sector_size);

    return TRUE;
}


_EXT_ITCM void UPGRADE_EraseWriteFlash(uint32_t u32_flashAddress, uint32_t u32_imageSize)
{
    uint32_t u32_dataSize = u32_imageSize;
    uint32_t u32_flashAddr = u32_flashAddress;

    while(u32_flashAddr % RDWR_BLOCK_SIZE != 0)
    {
        NOR_FLASH_EraseSector(u32_flashAddr);

        if (u32_dataSize <= RDWR_SECTOR_SIZE)
        {
            return ;
        }

        u32_dataSize -= RDWR_SECTOR_SIZE;
        u32_flashAddr += RDWR_SECTOR_SIZE;

    }

    while(1)
    {

        NOR_FLASH_EraseBlock(u32_flashAddr);

        if (u32_dataSize <= RDWR_BLOCK_SIZE)
        {
            return ;
        }

        u32_dataSize -= RDWR_BLOCK_SIZE;
        u32_flashAddr += RDWR_BLOCK_SIZE;

    }
}

_EXT_ITCM void NOR_FLASH_WriteByteBuffer(uint32_t start_addr, uint8_t* data_buf, uint32_t size)
{

    NOR_FLASH_WriteEnable(start_addr, size);
    
    ar_qspi_write_block_by_byte(start_addr, data_buf, size);

    NOR_FLASH_WriteDisable(start_addr, size);
}


_EXT_ITCM void  ar_flash_readblock(uint32_t flash_blk_st_addr, uint8_t* blk_val_table, uint32_t byte_size)
{
    ar_qspi_read_block_by_byte(flash_blk_st_addr, (uint8_t *)blk_val_table, byte_size);
}


_EXT_ITCM static int32_t NV_WriteToFlash(uint32_t u32_nvFlashAddr, STRU_NV_DATA *pst_nvDataPrc)
{
    NV_CalChk(pst_nvDataPrc, &(pst_nvDataPrc->u8_nvChk));

    // write to flash
    NOR_FLASH_WriteByteBuffer(u32_nvFlashAddr, 
                             (uint8_t *)pst_nvDataPrc, 
                             sizeof(STRU_NV_DATA));

    return 0;
}

_EXT_ITCM static const struct spi_flash_info * NOR_FLASH_ReadId(void)
{
    uint8_t                         id[SPI_FLASH_MAX_ID_LEN];
    const struct spi_flash_info    *info;

    
    ar_qspi_read_jedecid(id, SPI_FLASH_MAX_ID_LEN);

    info = spi_flash_ids;
    for (; info->name != NULL; info++)
    {
        if (info->id_len)
        {
            if (!memcmp(info->id, id, info->id_len))
                return info;
        }
    }
    return NULL;
}


_EXT_ITCM static void ar_flash_write_RcVtChipID(STRU_NV_DATA *par) 
{
    NOR_FLASH_EraseSector(NV_FLASH_RCVTCHIP_ADDR1);
    NV_WriteToFlash(NV_FLASH_RCVTCHIP_ADDR1, par);
    // NOR_FLASH_EraseSector(NV_FLASH_RCVTCHIP_ADDR2);
    // NV_WriteToFlash(NV_FLASH_RCVTCHIP_ADDR2, par);

    return;
}


_EXT_ITCM void NV_Save(STRU_NV_DATA *par)
{   
    NV_CalChk(par, &(par->u8_nvChk));

    ar_flash_write_RcVtChipID(par);
}

_EXT_ITCM void ar_flash_update_id(uint8_t *buffer, uint8_t length, int type)
{   

    ar_flash_copyRcIdVtId_init();

    STRU_NV *pst_nv = (STRU_NV *)SRAM_NV_MEMORY_ST_ADDR;
    switch (type)
    {
    case ENUM_ID_TYPE_RCID:
        pst_nv->st_nvDataPrc.u8_nvBbRcId[0] = buffer[0];
        pst_nv->st_nvDataPrc.u8_nvBbRcId[1] = buffer[1];
        pst_nv->st_nvDataPrc.u8_nvBbRcId[2] = buffer[2];
        pst_nv->st_nvDataPrc.u8_nvBbRcId[3] = buffer[3];
        pst_nv->st_nvDataPrc.u8_nvBbRcId[4] = buffer[4];

        break;
    case ENUM_ID_TYPE_VTID:
        pst_nv->st_nvDataPrc.u8_nvBbVtId[0] = buffer[0];
        pst_nv->st_nvDataPrc.u8_nvBbVtId[1] = buffer[1];
        break;

    case ENUM_ID_TYPE_ALL:

        // rcid & ChipId same 
        pst_nv->st_nvDataPrc.u8_nvBbRcId[0] = buffer[0];
        pst_nv->st_nvDataPrc.u8_nvBbRcId[1] = buffer[1];
        pst_nv->st_nvDataPrc.u8_nvBbRcId[2] = buffer[2];
        pst_nv->st_nvDataPrc.u8_nvBbRcId[3] = buffer[3];
        pst_nv->st_nvDataPrc.u8_nvBbRcId[4] = buffer[4];

        pst_nv->st_nvDataPrc.u8_nvBbVtId[0] = buffer[5];
        pst_nv->st_nvDataPrc.u8_nvBbVtId[1] = buffer[6];

        pst_nv->st_nvDataPrc.u8_nvChipId[0] = buffer[0];
        pst_nv->st_nvDataPrc.u8_nvChipId[1] = buffer[1];
        pst_nv->st_nvDataPrc.u8_nvChipId[2] = buffer[2];
        pst_nv->st_nvDataPrc.u8_nvChipId[3] = buffer[3];
        pst_nv->st_nvDataPrc.u8_nvChipId[4] = buffer[4];

        break;
    
    default:
        return;
    }

    NV_Save(&pst_nv->st_nvDataPrc);

    ar_flash_copyRcIdVtId_init();

    // memset((uint8_t *)SRAM_USR_NV_MEMORY_ST_ADDR, 0x00, SRAM_USR_NV_MEMORY_SIZE);

    // pst_nv->st_nvMng.u32_nvInitFlag = 0x23178546;
} 

_EXT_ITCM void ar_flash_init(void)
{
    const struct spi_flash_info * info;

    memset(&g_norflash, 0, sizeof(struct spi_flash));

    // ar_qspi_update_instruct(QUAD_SPI_INSTR_0, 0x001c14, 0x700);
    // ar_qspi_update_instruct(QUAD_SPI_INSTR_1, 0x001cd4, 0x700);
    // ar_qspi_update_instruct(QUAD_SPI_INSTR_2, 0x001c54, 0x700);
    // ar_qspi_update_instruct(QUAD_SPI_INSTR_3, 0x609c04, 0x700);
    // ar_qspi_update_instruct(QUAD_SPI_INSTR_4, 0x609cc4, 0x700);
    // ar_qspi_update_instruct(QUAD_SPI_INSTR_5, 0x609c44, 0x700);
    // ar_qspi_update_instruct(QUAD_SPI_INSTR_6, 0x609c10, 0x0);
    // ar_qspi_update_instruct(QUAD_SPI_INSTR_7, 0x609c18, 0x0);
    // ar_qspi_update_instruct(QUAD_SPI_INSTR_8, 0x609d40, 0x0);
    // ar_qspi_update_instruct(QUAD_SPI_INSTR_9, 0x401e40, 0x3c1700);
    // ar_qspi_update_instruct(QUAD_SPI_INSTR_10, 0x401e7c, 0x1700);
    // ar_qspi_update_instruct(QUAD_SPI_INSTR_11, 0x409f1c, 0x0);
    // ar_qspi_update_instruct(QUAD_SPI_INSTR_16, 0x609c80, 0x17);
    // ar_qspi_update_instruct(QUAD_SPI_INSTR_17, 0x609f60, 0x17);

    info = NOR_FLASH_ReadId();

    if(info != NULL)
    {
        g_norflash.name = info->name;
        
        memcpy(g_norflash.id, info->id, SPI_FLASH_MAX_ID_LEN);
        
        g_norflash.size = info->sector_size * info->n_sectors;

        if(g_norflash.size >= g_protect_size)
        {
            uint32_t s1_tmp, s2_tmp;

            g_norflash.protect_start = info->protect_start;
            
            g_norflash.protect_end = info->protect_end;

            g_norflash.flags = info->flags;

            g_norflash.s1_map = info->s1_map;

            g_norflash.s2_map = info->s2_map;
            
            g_norflash.s1_wp_map = info->s1_wp_map;

            g_norflash.s2_wp_map = info->s2_wp_map;

            s1_tmp = ar_qspi_read_reg1_status();
            s2_tmp = ar_qspi_read_reg2_status();

            if((s1_tmp != 0 && (s1_tmp & g_norflash.s1_wp_map) != g_norflash.s1_map) ||
                (s2_tmp != 0 && (s2_tmp & g_norflash.s2_wp_map) != g_norflash.s2_map))
            {
                NOR_FLASH_WP_Enable(g_norflash, 0, g_norflash.protect_start, 0);
            }

            if(0 != NOR_FLASH_WP_Enable(g_norflash, 1, g_norflash.protect_start, 0))
            {
                NOR_FLASH_SetFlashDisableFlag(1);
            }
        }
    }
    else
    {
        NOR_FLASH_SetFlashDisableFlag(1);
    }
    
    volatile STRU_NV *pst_nv = (volatile STRU_NV *)SRAM_NV_MEMORY_ST_ADDR;
    memset((void *)pst_nv, 0, SRAM_NV_MEMORY_SIZE);

    ar_flash_copyRcIdVtId_init();

    if (pst_nv->st_nvDataPrc.u8_nvBbRcId[0] == 0x00 && 
        pst_nv->st_nvDataPrc.u8_nvBbRcId[1] == 0x00 && 
        pst_nv->st_nvDataPrc.u8_nvBbRcId[2] == 0x00 &&
        pst_nv->st_nvDataPrc.u8_nvBbRcId[3] == 0x00 &&
        pst_nv->st_nvDataPrc.u8_nvBbRcId[4] == 0x00)
    {
        STRU_NV_DATA msg = {};    
        memset(&msg, 0, sizeof(STRU_NV_DATA));
        msg.u8_nvBbRcId[0] = 0x11;
        msg.u8_nvBbRcId[1] = 0x22;
        msg.u8_nvBbRcId[2] = 0x33;
        msg.u8_nvBbRcId[3] = 0x44;
        msg.u8_nvBbRcId[4] = 0x55;

        msg.u8_nvBbVtId[0] = 0xaa;
        msg.u8_nvBbVtId[1] = 0xbb;

        msg.u8_nvChipId[0] = 0x11;
        msg.u8_nvChipId[1] = 0x22;
        msg.u8_nvChipId[2] = 0x33;
        msg.u8_nvChipId[3] = 0x44;
        msg.u8_nvChipId[4] = 0x55;

        NV_Save(&msg);

        ar_flash_copyRcIdVtId_init();
    }

    memset((uint8_t *)SRAM_USR_NV_MEMORY_ST_ADDR, 0x00, SRAM_USR_NV_MEMORY_SIZE);

    pst_nv->st_nvMng.u32_nvInitFlag = 0x23178546;
}