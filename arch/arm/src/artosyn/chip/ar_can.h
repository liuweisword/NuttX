/************************************************************************************
 * arch/arm/src/artosyn/chip/ar_can.h
 *
 *   Copyright (C) 2009, 2011, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_ARTOSYN_CHIP_AR_CAN_H
#define __ARCH_ARM_SRC_ARTOSYN_CHIP_AR_CAN_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>

/*******************Macro define**************************/
#define BASE_ADDR_CAN0 (0x40300000)
#define BASE_ADDR_CAN1 (0x40340000)
#define BASE_ADDR_CAN2 (0x40380000)
#define BASE_ADDR_CAN3 (0x403C0000)

//define register RTIE
#define CAN_RTIE_RIE    (1<<7)
#define CAN_RTIE_ROIE   (1<<6)
#define CAN_RTIE_RFIE   (1<<5)
#define CAN_RTIE_RAFIE  (1<<4)
#define CAN_RTIE_TPIE   (1<<3)
#define CAN_RTIE_TSIE   (1<<2)
#define CAN_RTIE_EIE    (1<<1)

//define register TCMD
#define CAN_TCMD_TBSEL  (1<<7)
#define CAN_TCMD_LOM    (1<<6)
#define CAN_TCMD_STBY   (1<<5)
#define CAN_TCMD_TPE    (1<<4)
#define CAN_TCMD_TPA    (1<<3)
#define CAN_TCMD_TSONE  (1<<2)
#define CAN_TCMD_TSALL  (1<<1)
#define CAN_TCMD_TSA    (1<<0)

//define register TBUF
#define CAN_TBUF_IDE    (1<<7)  //std or ext
#define CAN_TBUF_RTR    (1<<6)  //data or remote
#define CAN_TBUF_EDL    (1<<5)
#define CAN_TBUF_BRS    (1<<4)

//define register CFG_STAT
#define CFG_STAT_RESET     (1<<7) // 
#define CFG_STAT_LBME      (1<<6) // 
#define CFG_STAT_LBMI      (1<<5) // 
#define CFG_STAT_TPSS      (1<<4) // 
#define CFG_STAT_TSSS      (1<<3) // 
#define CFG_STAT_RACTIVE   (1<<2) // Reception ACTIVE (Receive Status bit)
#define CFG_STAT_TACTIVE   (1<<1) // Transmission ACTIVE (Transmit Status bit)
#define CFG_STAT_BUSOFF    (1<<0) // 


/*******************can register define**************************/
typedef struct{
    volatile uint32_t    u32_rxBuf[18];          // 0x00-0x47
    volatile uint32_t    u32_txBuf[18];          // 0x48-0x8f
    volatile uint32_t    u32_reg3;               // 0x90 --> RCTRL     TCTRL    TCMD      CFG_STAT
    volatile uint32_t    u32_reg4;               // 0x94 --> LIMIT     ERRINT   RTIF      RTIE
    volatile uint32_t    u32_reg5;               // 0x98 -->  --       BITTIME2 BITTIME1  BITTIME0
    volatile uint32_t    u32_reg6;               // 0x9c -->  --       TDC      F_PRESC   S_PRESC
    volatile uint32_t    u32_reg7;               // 0xa0 -->  TECNT    RECNT    --        EALCAP
    volatile uint32_t    u32_reg8;               // 0xa4 -->  ACF_EN1  ACF_EN0  --        ACFCTRL
    volatile uint32_t    u32_reg9;               // 0xa8 -->  ACF3     ACF2     ACF1      ACF0
    volatile uint32_t    u32_reg10;              // 0xac -->  --       --       VER1      VER0
    volatile uint32_t    u32_reg11;              // 0xb0 -->  REF_MSG3 REF_MSG2 REF_MSG1  REF_MSG0
    volatile uint32_t    u32_reg12;              // 0xb4 -->  TT_TRIG1 TT_TRIG0 TRIG_CFG1 TRIG_CFG0
    volatile uint32_t    u32_reg13;              // 0xb8 -->  --       --       TT_WTRIG1 TT_WTRIG0
} STRU_CAN_TYPE;




#define CAN_TOTAL_CHANNEL       (4)

#define CAN_AMASK_ID10_0        (0x7FF)
#define CAN_UNAMASK_ID10_0      (0x0)
#define CAN_FRAME_LEN_AMASK     (0xF)

#define CAN_AMASK_ID28_0        (0x1FFFFFFF)
#define CAN_UNAMASK_ID28_0      (0x0)


struct STRU_CAN_CONFIG;
struct STRU_CAN_MSG;

//can read msg function define.just read u8_canRxCnt frame from pst_canRxBuf
typedef uint32_t (*CAN_RcvMsgHandler)(struct STRU_CAN_MSG *pst_canRxBuf, \
                                      uint8_t u8_canRxCnt);

typedef enum
{
    CAN_COMPONENT_0 = 0,
    CAN_COMPONENT_1,
    CAN_COMPONENT_2,
    CAN_COMPONENT_3, 
} ENUM_CAN_COMPONENT;

typedef enum
{
    CAN_BAUDR_125K = 0,
    CAN_BAUDR_250K,
    CAN_BAUDR_500K,
    CAN_BAUDR_1M, 
} ENUM_CAN_BAUDR;

typedef enum
{
    CAN_FORMAT_STD= 0,
    CAN_FORMAT_EXT,
} ENUM_CAN_FORMAT;

typedef enum
{
    CAN_TYPE_DATA= 0,
    CAN_TYPE_RMT,
} ENUM_CAN_TYPE;

typedef struct STRU_CAN_CONFIG
{
    ENUM_CAN_COMPONENT     e_canComponent;   
    ENUM_CAN_BAUDR         e_canBaudr;       
    uint32_t               u32_canAcode;     /*std bit10~0 <-> ID10~0 
                                               ext bit28~0 <-> ID28~0*/
    uint32_t               u32_canAmask;     /*std bit10~0 <-> ID10~0 
                                               ext bit28~0 <-> ID28~0*/     
    ENUM_CAN_FORMAT        e_canFormat;      
    CAN_RcvMsgHandler      pfun_canRcvMsg;   
} STRU_CAN_CONFIG;

typedef struct STRU_CAN_MSG
{
    ENUM_CAN_COMPONENT     e_canComponent;       
    uint32_t               u32_canId;            
    uint8_t                u8_canDataArray[8];    
    uint8_t                u8_canDataLen;                    
    ENUM_CAN_FORMAT        e_canFormat;          
    ENUM_CAN_TYPE          e_canType;            
} STRU_CAN_MSG;

typedef enum{
        CAN_PAR_BR,     //baud rate
        CAN_PAR_ACODE,  //acceptance code
        CAN_PAR_AMASK,  //acceptance mask
        CAN_PAR_RTIE,   //receive and transmit interrupt
} ENUM_CAN_PAR_NO;

/****************************Function declaration*****************************/
/**
* @brief    can init 
* @param    e_canComponent        CAN_COMPONENT_0 ~ 3 
* @param    e_canBaudr            CAN_BAUDR_125K ~ 1M
* @param    u32_acode             std bit10~0 <-> ID10~0
*                                 ext bit28~0 <-> ID28~0
* @param    u32_amask             std bit10~0 <-> ID10~0
*                                 ext bit28~0 <-> ID28~0
* @param    u8_rtie               bit7~bit1 <---> RIE,ROIE,
*                                 RFIE,RAFIE,TPIE,TSIE,EIE 
* @param    e_canFormat           standard or extended format 
* @retval   0                     init successed.
*           other                 init failed. 
* @note     None.
*/
int32_t CAN_InitHw(ENUM_CAN_COMPONENT e_canComponent, 
                   ENUM_CAN_BAUDR e_canBaudr, 
                   uint32_t u32_acode, 
                   uint32_t u32_amask, 
                   uint8_t u8_rtie,
                   ENUM_CAN_FORMAT e_canFormat);

/**
* @brief    send can frame 
* @param    e_canComponent        CAN_COMPONENT_0 ~ 3
* @param    u32_id:               std bit10~0 <-> ID10~0
*                                 ext bit28~0 <-> ID28~0
* @param    u32_txBuf:            send data buf for data field.
* @param    u8_len:               data length for data field in byte.
* @param    e_canFormat           standard or extended format
* @param    e_canType             data or remote frame
* @retval   0                     send can frame sucessed.
*           other                 send can frame failed. 
* @note     None.
*/
int32_t CAN_Send(ENUM_CAN_COMPONENT e_canComponent, 
                 uint32_t u32_id, 
                 uint8_t *u32_txBuf, 
                 uint8_t u8_len, 
                 ENUM_CAN_FORMAT e_canFormat, 
                 ENUM_CAN_TYPE e_canType);

/**
* @brief    receive frame from can controller.
* @param    e_canComponent        CAN_COMPONENT_0 ~ 3
* @param    u32_id:               std bit10~0 <-> ID10~0
*                                 ext bit28~0 <-> ID28~0
* @param    u32_txBuf:            receive data buf for data field.
* @param    u8_len:               receive data length for data field in byte.
* @param    e_canFormat           standard or extended format
* @param    e_canType             data or remote frame
* @retval   0                     receive can frame sucessed.
*           other                 receive can frame failed. 
* @note     None.
*/
int32_t CAN_Rcv(ENUM_CAN_COMPONENT e_canComponent, 
                uint32_t *u32_id, 
                uint8_t *u8_rxBuf, 
                uint8_t *u8_len, 
                ENUM_CAN_FORMAT *pe_canFormat, 
                ENUM_CAN_TYPE *pe_canType);

/**
* @brief  can interrupt servive function.just handled data reception.  
* @param  u32_vectorNum           Interrupt number.
* @retval None.
* @note   None.
*/
void CAN_IntrSrvc(uint32_t u32_vectorNum);

/**
* @brief  register user function for can recevie data.called in interrupt
*         service function.
* @param  u8_canCh           can channel, 0 ~ 3.
* @param  userHandle         user function for can recevie data.
* @retval 
*         -1                  register user function failed.
*         0                   register user function sucessed.
* @note   None.
*/
int32_t CAN_RegisterUserRxHandler(uint8_t u8_canCh, CAN_RcvMsgHandler userHandler);

/**
* @brief  unregister user function for can recevie data.
* @param  u8_canCh            can channel, 0 ~ 3.
* @retval 
*         -1                  unregister user function failed.
*         0                   unregister user function sucessed.
* @note   None.
*/
int32_t CAN_UnRegisterUserRxHandler(uint8_t u8_canCh);

/**
* @brief  get can control tx busy status.
* @param  e_canComponent        CAN_COMPONENT_0 ~ 3
* @retval 
*         0                     can control idle
*         1                     can control busy
* @note   None.
*/
int32_t CAN_GetTxBusyStatus(ENUM_CAN_COMPONENT e_canComponent);





#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_CAN_H */
