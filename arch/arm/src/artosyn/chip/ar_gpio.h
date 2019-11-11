/************************************************************************************
 * arch/arm/src/artosyn/ar_gpio.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *          liuwei <wei.liu@cecooleye.cn>
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

#ifndef __ARCH_ARM_SRC_ARTOSYN_CHIP_AR_GPIO_H
#define __ARCH_ARM_SRC_ARTOSYN_CHIP_AR_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include <nuttx/config.h>

#include "ar_rcc.h"
#include "chip/ar_define.h"
/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif



////////////////////////////////////////////////////////////



#define GPIO_NUM_PINS      				(128)

#define MULTIPLE           				(0)

typedef enum
{
    HAL_GPIO_PIN_MODE0 = 0,     /* DEFAULT mode */
    HAL_GPIO_PIN_MODE1,         /* TEST mode */
    HAL_GPIO_PIN_MODE2          /* GPIO mode */

} ENUM_HAL_GPIO_PinMode;


typedef enum
{
    HAL_GPIO_PIN_RESET = 0, //gpio output 0
    HAL_GPIO_PIN_SET        //gpio output 1

} ENUM_HAL_GPIO_PinState;

typedef enum
{
    HAL_GPIO_ACTIVE_LOW = 0, //falling-edge or active-low sensitive

    HAL_GPIO_ACTIVE_HIGH     //rising-edge or active-high sensitive

} ENUM_HAL_GPIO_InterrputPolarity;

typedef enum
{
    HAL_GPIO_LEVEL_SENUMSITIVE = 0,//level-interrupt
    HAL_GPIO_EDGE_SENUMSITIVE      //edge-interrupt

} ENUM_HAL_GPIO_InterrputLevel;

typedef enum
{  
    HAL_GPIO_NUM0 = 0,
    HAL_GPIO_NUM1,
    HAL_GPIO_NUM2,
    HAL_GPIO_NUM3,
    HAL_GPIO_NUM4,
    HAL_GPIO_NUM5,
    HAL_GPIO_NUM6,
    HAL_GPIO_NUM7,
    HAL_GPIO_NUM8,
    HAL_GPIO_NUM9,
    HAL_GPIO_NUM10,
    HAL_GPIO_NUM11,
    HAL_GPIO_NUM12,
    HAL_GPIO_NUM13,
    HAL_GPIO_NUM14,
    HAL_GPIO_NUM15,
    HAL_GPIO_NUM16,
    HAL_GPIO_NUM17,
    HAL_GPIO_NUM18,
    HAL_GPIO_NUM19,
    HAL_GPIO_NUM20,
    HAL_GPIO_NUM21,
    HAL_GPIO_NUM22,
    HAL_GPIO_NUM23,
    HAL_GPIO_NUM24,
    HAL_GPIO_NUM25,
    HAL_GPIO_NUM26,
    HAL_GPIO_NUM27,
    HAL_GPIO_NUM28,
    HAL_GPIO_NUM29,
    HAL_GPIO_NUM30,
    HAL_GPIO_NUM31,
    HAL_GPIO_NUM32,
    HAL_GPIO_NUM33,
    HAL_GPIO_NUM34,
    HAL_GPIO_NUM35,
    HAL_GPIO_NUM36,
    HAL_GPIO_NUM37,
    HAL_GPIO_NUM38,
    HAL_GPIO_NUM39,
    HAL_GPIO_NUM40,
    HAL_GPIO_NUM41,
    HAL_GPIO_NUM42,
    HAL_GPIO_NUM43,
    HAL_GPIO_NUM44,
    HAL_GPIO_NUM45,
    HAL_GPIO_NUM46,
    HAL_GPIO_NUM47,
    HAL_GPIO_NUM48,
    HAL_GPIO_NUM49,
    HAL_GPIO_NUM50,
    HAL_GPIO_NUM51,
    HAL_GPIO_NUM52,
    HAL_GPIO_NUM53,
    HAL_GPIO_NUM54,
    HAL_GPIO_NUM55,
    HAL_GPIO_NUM56,
    HAL_GPIO_NUM57,
    HAL_GPIO_NUM58,
    HAL_GPIO_NUM59,
    HAL_GPIO_NUM60,
    HAL_GPIO_NUM61,
    HAL_GPIO_NUM62,
    HAL_GPIO_NUM63,
    HAL_GPIO_NUM64,
    HAL_GPIO_NUM65,
    HAL_GPIO_NUM66,
    HAL_GPIO_NUM67,
    HAL_GPIO_NUM68,
    HAL_GPIO_NUM69,
    HAL_GPIO_NUM70,
    HAL_GPIO_NUM71,
    HAL_GPIO_NUM72,
    HAL_GPIO_NUM73,
    HAL_GPIO_NUM74,
    HAL_GPIO_NUM75,
    HAL_GPIO_NUM76,
    HAL_GPIO_NUM77,
    HAL_GPIO_NUM78,
    HAL_GPIO_NUM79,
    HAL_GPIO_NUM80,
    HAL_GPIO_NUM81,
    HAL_GPIO_NUM82,
    HAL_GPIO_NUM83,
    HAL_GPIO_NUM84,
    HAL_GPIO_NUM85,
    HAL_GPIO_NUM86,
    HAL_GPIO_NUM87,
    HAL_GPIO_NUM88,
    HAL_GPIO_NUM89,
    HAL_GPIO_NUM90,
    HAL_GPIO_NUM91,
    HAL_GPIO_NUM92,
    HAL_GPIO_NUM93,
    HAL_GPIO_NUM94,
    HAL_GPIO_NUM95,
    HAL_GPIO_NUM96,
    HAL_GPIO_NUM97,
    HAL_GPIO_NUM98,
    HAL_GPIO_NUM99,
    HAL_GPIO_NUM100,
    HAL_GPIO_NUM101,
    HAL_GPIO_NUM102,
    HAL_GPIO_NUM103,
    HAL_GPIO_NUM104,
    HAL_GPIO_NUM105,
    HAL_GPIO_NUM106,
    HAL_GPIO_NUM107,
    HAL_GPIO_NUM108,
    HAL_GPIO_NUM109,
    HAL_GPIO_NUM110,
    HAL_GPIO_NUM111,
    HAL_GPIO_NUM112,
    HAL_GPIO_NUM113,
    HAL_GPIO_NUM114,
    HAL_GPIO_NUM115,
    HAL_GPIO_NUM116,
    HAL_GPIO_NUM117,
    HAL_GPIO_NUM118,
    HAL_GPIO_NUM119,
    HAL_GPIO_NUM120,
    HAL_GPIO_NUM121,
    HAL_GPIO_NUM122,
    HAL_GPIO_NUM123,
    HAL_GPIO_NUM124,
    HAL_GPIO_NUM125,
    HAL_GPIO_NUM126,
    HAL_GPIO_NUM127
} ENUM_HAL_GPIO_NUM;


#define GPIO0_BASE_ADDR                 0x40400000
#define GPIO1_BASE_ADDR                 0x40440000
#define GPIO2_BASE_ADDR                 0x40480000
#define GPIO3_BASE_ADDR                 0x404C0000


#define GPIO_MODE0_ADDR    			    0x40B0007C //0 -15
#define GPIO_MODE1_ADDR    			    0x40B00080 //16-31
#define GPIO_MODE2_ADDR    			    0x40B00084 //32-47
#define GPIO_MODE3_ADDR    			    0x40B00088 //48-63
#define GPIO_MODE4_ADDR    			    0x40B0008C //64-79
#define GPIO_MODE5_ADDR    			    0x40B00090 //80-95
#define GPIO_MODE6_ADDR    			    0x40B00094 //96-111
#define GPIO_MODE7_ADDR    			    0x40B00098 //112-127

#define GPIO_DATA_A_OFFSET              0x00
#define GPIO_DATA_B_OFFSET              0x0C
#define GPIO_DATA_C_OFFSET              0x18
#define GPIO_DATA_D_OFFSET              0x24

#define GPIO_DATA_DIRECT_A_OFFSET       0x04
#define GPIO_DATA_DIRECT_B_OFFSET       0x10
#define GPIO_DATA_DIRECT_C_OFFSET       0x1C
#define GPIO_DATA_DIRECT_D_OFFSET       0x28

#define GPIO_DATA_DIRECT_INPUT          (0)
#define GPIO_DATA_DIRECT_OUTPUT         (1)

#define GPIO_CTRL_A_OFFSET              0x08
#define GPIO_CTRL_B_OFFSET              0x14
#define GPIO_CTRL_C_OFFSET              0x20
#define GPIO_CTRL_D_OFFSET              0x2C

#define GPIO_CTRL_SOFTWARE				(0)
#define GPIO_CTRL_HARDWARE              (1)

#define GPIO_EXT_PORT_A_OFFSET          0x50
#define GPIO_EXT_PORT_B_OFFSET          0x54
#define GPIO_EXT_PORT_C_OFFSET          0x58
#define GPIO_EXT_PORT_D_OFFSET          0x5C

#define GPIO_INTEN_OFFSET               0x30
#define GPIO_INTEN_NORMAL               (0)
#define GPIO_INTEN_INTERRUPT            (1)

#define GPIO_MASK_OFFSET                0x34
#define GPIO_MASK_MASK                  (0)
#define GPIO_MASK_UNMASK                (1)

#define GPIO_INTTYPE_OFFSET             0x38
#define GPIO_INTTYPE_LEVEL              (0)
#define GPIO_INTTYPE_EDGE               (1)

#define GPIO_INTPOL_OFFSET              0x3C
#define GPIO_INTPOL_LOW                 (0)
#define GPIO_INTPOL_HIGH                (1)

#define GPIO_DEBOUNCE_OFFSET            0x48
#define GPIO_DEBOUNCE_OFF               (0)
#define GPIO_DEBOUNCE_ON                (1)

#define GPIO_INTSTATUS_OFFSET           0x40
#define GPIO_RAW_INTSTATUS_OFFSET       0x44
#define GPIO_CLEARINT_OFFSET            0x4C

void GPIO_SetPin(uint32_t gpioNum, uint32_t value);

uint32_t GPIO_GetPin(uint32_t gpioNum);

void GPIO_SetPinHi(uint32_t gpioNum);

void GPIO_SetPinLo(uint32_t gpioNum);

void GPIO_SetMode(uint32_t gpioNum, uint32_t mode);

#if MULTIPLE

void GPIO_SetPinListList(uint32_t *pList, uint32_t size, uint32_t *mode);

void GPIO_SetPinListVal(uint32_t *pList, uint32_t size, uint32_t mode);

void GPIO_SetPinRange(uint32_t gpioNum1, uint32_t gpioNum2, uint32_t mode);

void GPIO_ModeRange(uint32_t gpioNum1, uint32_t gpioNum2,uint32_t mode);

void GPIO_ModeListVal(uint32_t *pList, uint32_t size, uint32_t mode);

void GPIO_ModeListList(uint32_t *gpioList, uint32_t size, uint32_t *modeList);

#endif

void GPIO_SetPinDirect(uint32_t gpioNum, uint32_t mode);

void GPIO_SetPinCtrl(uint32_t gpioNum, uint32_t mode);

void GPIO_Intr_SetPinIntrEn(uint32_t gpioNum, uint32_t mode);

void GPIO_Intr_SetPinIntrMask(uint32_t gpioNum, uint32_t mode);

void GPIO_Intr_SetPinIntrType(uint32_t gpioNum, uint32_t mode);

void GPIO_Intr_SetPinIntrPol(uint32_t gpioNum, uint32_t mode);

uint32_t GPIO_Intr_GetIntrStatus(uint32_t gpioNum);

uint32_t GPIO_Intr_GetRawIntrStatus(uint32_t gpioNum);

void GPIO_Intr_ClearIntr(uint32_t gpioNum);

uint32_t GPIO_Intr_GetIntrGroupStatus(uint32_t u32_vectorNum);

void GPIO_Intr_ClearIntrGroup(uint32_t u32_vectorNum, uint8_t u8_flag);

HAL_RET_T HAL_GPIO_SetMode(ENUM_HAL_GPIO_NUM e_gpioPin,ENUM_HAL_GPIO_PinMode e_gpioMode);

HAL_RET_T HAL_GPIO_OutPut(ENUM_HAL_GPIO_NUM e_gpioPin);

HAL_RET_T HAL_GPIO_InPut(ENUM_HAL_GPIO_NUM e_gpioPin);


HAL_RET_T HAL_GPIO_GetPin(ENUM_HAL_GPIO_NUM e_gpioPin,uint32_t *p_retGpioState);

HAL_RET_T HAL_GPIO_SetPin(ENUM_HAL_GPIO_NUM e_gpioPin, ENUM_HAL_GPIO_PinState e_pinState);


HAL_RET_T HAL_GPIO_RegisterInterrupt(ENUM_HAL_GPIO_NUM e_gpioPin,

                                     ENUM_HAL_GPIO_InterrputLevel e_inttype,

                                     ENUM_HAL_GPIO_InterrputPolarity e_polarity,

                                     void *fun_callBack);


#undef EXTERN
#if defined(__cplusplus)
}
#endif


#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_ARTOSYN_AR_GPIO_H */
