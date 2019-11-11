/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdint.h>
#include "ar_qspi.h"
#include <debug.h>
#include <sys/types.h>
#include <stdio.h>
#include "chip/ar_config.h"

#define CMD_ERASE_CHIP			            CMD11
#define CMD_ERASE_4K			            CMD16
#define CMD_ERASE_4K_ADDR			        CMD16_ADDR
#define CMD_ERASE_64K			            CMD17
#define CMD_ERASE_64K_ADDR			        CMD17_ADDR

#define CMD_WRITE_STATUS1		            CMD3
#define CMD_WRITE_STATUS2		            CMD4
#define CMD_WRITE_DISABLE		            CMD6
#define CMD_WRITE_ENABLE		            CMD7

#define CMD_READ_STATUS1		            CMD0
#define CMD_READ_STATUS2		            CMD1
#define CMD_READ_MFR_DEV_ID			        CMD9
#define CMD_READ_JEDEC_ID			        CMD10

_EXT_ITCM void QUAD_SPI_Delay(unsigned int delay)
{
    volatile int i = delay;
    while(i > 0)
    {
        i--;
    }
}

_EXT_ITCM uint8_t ar_qspi_enable_defaultInstruct(void)
{
    ar_putreg32(1<<0, HP_SPI_BASE_ADDR + HP_SPI_INIT_REG);
    return OK;
}

_EXT_ITCM uint8_t ar_qspi_update_instruct(ENUM_QUAD_SPI_INSTR_ID instr_id, uint32_t cmd_h, uint32_t cmd_l)
{
    if (instr_id > QUAD_SPI_INSTR_UNKNOWN)
    {
        return FALSE;
    }

    uint32_t instr_h_offset = INSTR0_H + (instr_id * 2 * 4);
    uint32_t instr_l_offset = INSTR0_L + (instr_id * 2 * 4);

    ar_putreg32(cmd_h, HP_SPI_BASE_ADDR + instr_h_offset);
    ar_putreg32(cmd_l, HP_SPI_BASE_ADDR + instr_l_offset);

    return TRUE;
}


_EXT_ITCM uint8_t ar_qspi_read_instruct(ENUM_QUAD_SPI_INSTR_ID instr_id, uint32_t * cmd_h, uint32_t * cmd_l)
{
    if (instr_id > QUAD_SPI_INSTR_UNKNOWN)
    {
        return FALSE;
    }

    uint32_t instr_h_offset = INSTR0_H + (instr_id * 2 * 4);
    uint32_t instr_l_offset = INSTR0_L + (instr_id * 2 * 4);
    
    (*cmd_h) = ar_getreg32(HP_SPI_BASE_ADDR + instr_h_offset);
    (*cmd_l) = ar_getreg32(HP_SPI_BASE_ADDR + instr_l_offset);

    return TRUE;
}



_EXT_ITCM uint8_t ar_qspi_write_enable(void)
{
    uint8_t ret = TRUE;

    ar_putreg32(0x0, HP_SPI_BASE_ADDR + CMD_WRITE_ENABLE);
    ar_putreg32(0x739c08, HP_SPI_BASE_ADDR + HP_SPI_WR_HW_REG);
    ar_putreg32(0xff17, HP_SPI_BASE_ADDR + HP_SPI_WR_LW_REG);
    ar_putreg32(0x0, HP_SPI_BASE_ADDR + HP_SPI_UPDATE_WR_REG);            
    return ret;
}


_EXT_ITCM uint8_t ar_qspi_write_disable(void)
{

    ar_putreg32(0x0, HP_SPI_BASE_ADDR + CMD_WRITE_DISABLE);
    ar_putreg32(0x0, HP_SPI_BASE_ADDR + HP_SPI_WR_HW_REG);
    ar_putreg32(0x0, HP_SPI_BASE_ADDR + HP_SPI_WR_LW_REG);
    ar_putreg32(0x0, HP_SPI_BASE_ADDR + HP_SPI_UPDATE_WR_REG);
        
    return OK;
}

_EXT_ITCM uint8_t ar_qspi_check_busy(void)
{
    unsigned int    rd_tmp;
    unsigned char   busy_flag;

    do
    {
        up_udelay(200);
        
        rd_tmp = ar_getreg32(HP_SPI_BASE_ADDR + CMD_READ_STATUS1); 
        busy_flag = rd_tmp & 0x1;

    } while(busy_flag);

    return OK;
}



_EXT_ITCM uint8_t ar_qspi_erase_block(uint32_t flash_blk_st_addr)
{
    uint8_t ret = TRUE;

    ar_putreg32(flash_blk_st_addr,  HP_SPI_BASE_ADDR + CMD_ERASE_64K_ADDR);
    ar_putreg32(0x0, HP_SPI_BASE_ADDR + CMD_ERASE_64K); 
    
    return ret;
}

_EXT_ITCM uint8_t ar_qspi_erase_sector(uint32_t flash_sect_st_addr)
{
    uint8_t ret = TRUE;

    ar_putreg32(flash_sect_st_addr, HP_SPI_BASE_ADDR + CMD_ERASE_4K_ADDR);
    ar_putreg32(0x0, HP_SPI_BASE_ADDR + CMD_ERASE_4K);
        
    return ret;
}


_EXT_ITCM uint8_t ar_qspi_erase_chip(void)
{
    uint8_t ret = TRUE;
    
    ar_putreg32(0x0, HP_SPI_BASE_ADDR + CMD_ERASE_CHIP);
       
    return ret;
}


_EXT_ITCM uint8_t ar_qspi_write_byte(uint32_t flash_addr, uint8_t value)
{
    uint8_t ret = TRUE;
    
    *((volatile uint8_t*)(FLASH_APB_BASE_ADDR + flash_addr)) = value;
            
    return ret;
}

_EXT_ITCM uint8_t ar_qspi_write_halfword(uint32_t flash_addr, uint16_t value)
{
    uint8_t ret = TRUE;

    *((volatile uint16_t*) (FLASH_APB_BASE_ADDR + flash_addr)) = value;  

    return ret;
}

_EXT_ITCM uint8_t ar_qspi_write_word(uint32_t flash_addr, uint32_t value)
{
    uint8_t ret = TRUE;

    *((volatile uint32_t*) (FLASH_APB_BASE_ADDR + flash_addr)) = value;     
            
    return ret;
}

_EXT_ITCM uint8_t ar_qspi_write_block_by_byte(uint32_t flash_blk_st_addr, uint8_t* blk_val_table, uint32_t byte_size)
{
    uint32_t i;
    uint8_t ret = TRUE;
    volatile uint8_t* write_addr = (uint8_t*)(FLASH_APB_BASE_ADDR + flash_blk_st_addr);

    if (blk_val_table == NULL)
    {
        return FALSE;
    }

    for (i = 0; i < byte_size; i++)
    {
        *write_addr = blk_val_table[i];
        write_addr++;
    }
    
    return ret;
}

_EXT_ITCM uint8_t ar_qspi_write_block_by_halfword(uint32_t flash_blk_st_addr, uint16_t* blk_val_table, uint32_t halfword_size)
{
    uint32_t i;
    uint8_t ret = TRUE;
    volatile uint16_t* write_addr = (uint16_t*)(FLASH_APB_BASE_ADDR + flash_blk_st_addr);

    if (blk_val_table == NULL)
    {
        return FALSE;
    }

    for (i = 0; i < halfword_size; i++)
    {
        *write_addr = blk_val_table[i];
        write_addr++;
    }
    
    return ret;
}

_EXT_ITCM uint8_t ar_qspi_write_block_by_word(uint32_t flash_blk_st_addr, uint32_t* blk_val_table, uint32_t word_size)
{
    uint32_t i;
    uint8_t ret = TRUE;
    volatile uint32_t* write_addr = (uint32_t*)(FLASH_APB_BASE_ADDR + flash_blk_st_addr);

    if (blk_val_table == NULL)
    {
        return FALSE;
    }

    for (i = 0; i < word_size; i++)
    {
        *write_addr = blk_val_table[i];
        write_addr++;
    }
    
    return ret;
}



_EXT_ITCM uint8_t ar_qspi_read_byte(uint32_t flash_addr, uint8_t* value_ptr)
{
    uint8_t ret = TRUE;

    volatile uint8_t* p = (uint8_t*)(FLASH_APB_BASE_ADDR + flash_addr);
    
    *value_ptr = *p;
            
    return ret;
}

_EXT_ITCM uint8_t ar_qspi_read_halfword(uint32_t flash_addr, uint16_t* value_ptr)
{
    uint8_t ret = TRUE;

    volatile uint16_t* p = (uint16_t*)(FLASH_APB_BASE_ADDR + flash_addr);
    
    *value_ptr = *p;

    return ret;
}

_EXT_ITCM uint8_t ar_qspi_read_word(uint32_t flash_addr, uint32_t* value_ptr)
{
    uint8_t ret = TRUE;

    volatile uint32_t* p = (uint32_t*)(FLASH_APB_BASE_ADDR + flash_addr);
    
    *value_ptr = *p;
            
    return ret;
}

_EXT_ITCM void ar_qspi_read_block_by_byte(uint32_t flash_blk_st_addr, uint8_t* blk_val_table, uint32_t byte_size)
{
    uint32_t i;
    volatile uint8_t* write_addr = (uint8_t*)(FLASH_APB_BASE_ADDR + flash_blk_st_addr);

    if (blk_val_table == NULL)
    {
        return ;
    }

    for (i = 0; i < byte_size; i++)
    {
        blk_val_table[i] = *write_addr;
        write_addr++;
    }
}

_EXT_ITCM uint8_t ar_qspi_set_speed(ENUM_QUAD_SPI_SPEED speed)
{
    uint8_t ret = TRUE;
   
    uint32_t mask_val = 0;
    uint32_t mask_bit = 1<<2 | 1<<3 | 1<<7;

    switch(speed)
    {
    case QUAD_SPI_SPEED_25M:
        mask_val = 0;
        break;
    case QUAD_SPI_SPEED_50M:
        mask_val = 1<<2;
        break;
    case QUAD_SPI_SPEED_100M:
        mask_val = 1<<3;
        break;
    case QUAD_SPI_SPEED_25M_ENCRYPT:
        mask_val = 1<<7;
        break;
    case QUAD_SPI_SPEED_50M_ENCRYPT:
        mask_val = 1<<2 | 1<<7;
        break;
    case QUAD_SPI_SPEED_100M_ENCRYPT:
        mask_val = 1<<3 | 1<<7;
        break;
    default:
        ret = FALSE;
        break;
     }

    Reg_Write32_Mask(HP_SPI_BASE_ADDR + HP_SPI_CONFIG_REG, mask_val, mask_bit);
    
    return ret;
}



_EXT_ITCM uint32_t ar_qspi_read_reg1_status(void)
{
    return ar_getreg32(HP_SPI_BASE_ADDR + CMD_READ_STATUS1);
}

_EXT_ITCM uint32_t ar_qspi_read_reg2_status(void)
{
    return ar_getreg32(HP_SPI_BASE_ADDR + CMD_READ_STATUS2);
}


_EXT_ITCM void ar_qspi_set_reg1_status(uint8_t flag)
{
    uint32_t u32_reg = flag;

    ar_putreg32(u32_reg, HP_SPI_BASE_ADDR + CMD_WRITE_STATUS1);
}

_EXT_ITCM void ar_qspi_set_reg2_status(uint8_t flag)
{
    uint32_t u32_reg = flag;

    ar_putreg32(u32_reg, HP_SPI_BASE_ADDR + CMD_WRITE_STATUS2);
}



_EXT_ITCM void ar_qspi_read_manudeviceid(uint8_t* data_buf, uint8_t buflen)
{
    uint32_t i = 0;
    uint32_t u32_id = 0;
    uint8_t len = (buflen < sizeof(u32_id) ? buflen : sizeof(u32_id));

    u32_id = ar_getreg32(HP_SPI_BASE_ADDR + CMD_READ_MFR_DEV_ID);
    for (i = 0; i < len; i++)
    {
        data_buf[i] = (u32_id>>i*8)&0xff;
    }
}

_EXT_ITCM void ar_qspi_read_jedecid(uint8_t* data_buf, uint8_t buflen)
{
    uint32_t i = 0;
    uint32_t u32_id = 0;
    uint8_t len = (buflen < sizeof(u32_id) ? buflen : sizeof(u32_id));

    u32_id = ar_getreg32(HP_SPI_BASE_ADDR + CMD_READ_JEDEC_ID);

    for (i = 0; i < len; i++)
    {
        data_buf[i] = (u32_id>>i*8)&0xff;
    }
}

