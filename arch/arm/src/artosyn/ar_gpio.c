/****************************************************************************
 * arch/arm/src/artosyn/ar_gpio.c
 *
 *   Copyright (C) 2015-2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *              liuwei <wei.liu@cecooleye.cn>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <string.h>


#include <errno.h>
#include <termios.h>


#include "chip/ar_define.h"
#include "chip/ar_gpio.h"
#include "ar_gpio.h"


#include "ar_uart.h"
#include "ar_rcc.h"




_EXT_ITCM void GPIO_SetMode(uint32_t gpioNum, uint32_t mode)
{
    uint32_t u32_GroupNoAddr = 0;
    uint8_t u8_GroupNo = 0;

    u8_GroupNo = (gpioNum>>4);
    u32_GroupNoAddr = u8_GroupNo*0x04 + GPIO_MODE0_ADDR;
    Reg_Write32_Mask(u32_GroupNoAddr, (mode << ((gpioNum % 16)<<1)), (0x3 << ((gpioNum % 16)<<1)));
    //DLOG_INFO(" SetMode %x %x \r\n", u32_GroupNoAddr, Reg_Read32(u32_GroupNoAddr));
}

#if MULTIPLE

_EXT_ITCM void GPIO_ModeRange(uint32_t gpioNum1, uint32_t gpioNum2, uint32_t mode)
{
    uint32_t i;
    uint32_t GPIOregVal;
    //DLOG_INFO(" ModeRange \r\n");

    for (i = gpioNum1; i <= gpioNum2; ++i)
    {
		GPIO_SetMode(i, mode);
        GPIO_SetPinDirect(i, GPIO_DATA_DIRECT_OUTPUT);
        GPIO_SetPinCtrl(i, GPIO_CTRL_SOFTWARE);
    }
}



_EXT_ITCM void GPIO_ModeListVal(uint32_t *pList, uint32_t size, uint32_t mode)
{
    uint32_t i;
    //DLOG_INFO(" ModeListVal\r\n");

    for (i = 0; i <= size; ++i)
    {
		GPIO_SetMode((*pList), mode);
        GPIO_SetPinDirect((*pList), GPIO_DATA_DIRECT_OUTPUT);
        GPIO_SetPinCtrl((*pList), GPIO_CTRL_SOFTWARE);
	    ++pList;
    }
}


_EXT_ITCM void GPIO_ModeListList(uint32_t *pList, uint32_t size, uint32_t *mode)
{
    uint32_t i;
    //DLOG_INFO(" ModeListList\r\n");

    for (i = 0; i <= size; ++i)
    {
		GPIO_SetMode((*pList), (*mode));
        GPIO_SetPinDirect((*pList), GPIO_DATA_DIRECT_OUTPUT);
        GPIO_SetPinCtrl((*pList), GPIO_CTRL_SOFTWARE);
	    ++pList;
	    ++mode;
    }
}

#endif


_EXT_ITCM void GPIO_SetPin(uint32_t gpioNum, uint32_t value)
{
    //DLOG_INFO(" SetPin \r\n");
    uint32_t u32_RegNoAddr = 0;
    uint32_t u32_GroupNoAddr = 0;
    uint8_t u8_PinNo = ((gpioNum%32)%8);

    switch((gpioNum>>5))
    {
        case 0:
        {
            u32_GroupNoAddr = GPIO0_BASE_ADDR;
            break;
        }
        case 1:
        {
            u32_GroupNoAddr = GPIO1_BASE_ADDR;
            break;
        }
        case 2:
        {
            u32_GroupNoAddr = GPIO2_BASE_ADDR;
            break;
        }
        case 3:
        {
            u32_GroupNoAddr = GPIO3_BASE_ADDR;
            break;
        }
        default:
            return ;
    }

    switch(((gpioNum%32)>>3))
    {
        case 0:
        {
            u32_RegNoAddr = GPIO_DATA_A_OFFSET;
            break;
        }
        case 1:
        {
            u32_RegNoAddr = GPIO_DATA_B_OFFSET;
            break;
        }
        case 2:
        {
            u32_RegNoAddr = GPIO_DATA_C_OFFSET;
            break;
        }
        case 3:
        {
            u32_RegNoAddr = GPIO_DATA_D_OFFSET;
            break;
        }
        default:
            return ;
    }

    if (value == 0)
    {
        Reg_Write32_Mask(u32_GroupNoAddr + u32_RegNoAddr, 0, (1 << u8_PinNo));
    }
    else
    {
        Reg_Write32_Mask(u32_GroupNoAddr + u32_RegNoAddr, (1 << u8_PinNo), (1 << u8_PinNo));
    }

}




#if MULTIPLE

_EXT_ITCM void GPIO_SetPinRange(uint32_t gpioNum1, uint32_t gpioNum2, uint32_t mode)
{
    uint32_t i;
    uint32_t GPIOregVal;

    //DLOG_INFO(" SetPinRange \r\n");

    for (i = gpioNum1; i <= gpioNum2; ++i)
    {
        GPIO_SetPin(i, mode);
    }
}



_EXT_ITCM void GPIO_SetPinListVal(uint32_t *pList, uint32_t size, uint32_t mode)
{
    uint32_t i;
    //DLOG_INFO(" SetPinListVal \r\n");

    for (i = 0; i <= size; ++i)
    {
        GPIO_SetPin((*pList), mode);
        ++pList;
    }
}



_EXT_ITCM void GPIO_SetPinListList(uint32_t *pList, uint32_t size, uint32_t *mode)
{
    uint32_t i;
    //DLOG_INFO(" SetPinListList \r\n");

    for (i = 0; i <= size; ++i)
    {
        GPIO_SetPin((*pList), (*mode));
        ++pList;
        ++mode;
    }
}

#endif




_EXT_ITCM uint32_t GPIO_Get(uint32_t gpioNum)
{
    uint32_t u32_RegNoAddr = 0;
    //uint32_t u32_PinNoAddr = 0;
    uint32_t u32_GroupNoAddr = 0;
    uint8_t u8_RegNo = 0;
    uint8_t u8_PinNo = 0;
    uint8_t u8_GroupNo = 0;
    uint32_t u32_GpioRegVal = 0;
    u8_GroupNo = (gpioNum>>5);
    u8_RegNo = (gpioNum%32)>>3;
    u8_PinNo = (gpioNum%32)%8;
    u32_GroupNoAddr = u8_GroupNo*0x40000 + GPIO0_BASE_ADDR;
    u32_RegNoAddr = u8_RegNo*0x04 + GPIO_EXT_PORT_A_OFFSET;
    u32_GpioRegVal = Reg_Read32(u32_GroupNoAddr + u32_RegNoAddr);

    return ((u32_GpioRegVal >> u8_PinNo) & 1);
}


_EXT_ITCM uint32_t GPIO_GetPin(uint32_t gpioNum)
{
    return GPIO_Get(gpioNum);
}



_EXT_ITCM void GPIO_SetPinDirect(uint32_t gpioNum, uint32_t mode)
{
    uint32_t u32_GroupNoAddr = 0;
    uint32_t u32_RegNoAddr = 0;
    uint8_t  u8_RegNo = 0;
    uint8_t  u8_PinNo = 0;
    uint8_t  u8_GroupNo = 0;
    uint32_t u32_GpioRegVal = 0;

    u8_GroupNo = (gpioNum>>5);
    u8_RegNo = (gpioNum%32)>>3;
    u8_PinNo = (gpioNum%32)%8;

    u32_GroupNoAddr = u8_GroupNo*0x40000 + GPIO0_BASE_ADDR;
    u32_RegNoAddr = u8_RegNo*0x0C + GPIO_DATA_DIRECT_A_OFFSET;
    u32_GpioRegVal = Reg_Read32(u32_GroupNoAddr + u32_RegNoAddr);

    if(GPIO_DATA_DIRECT_OUTPUT == mode)
    {
        u32_GpioRegVal |= (mode << u8_PinNo);
    }
    else
    {
        u32_GpioRegVal &= ~(1 << u8_PinNo);
    }

    Reg_Write32(u32_GroupNoAddr + u32_RegNoAddr, u32_GpioRegVal);

    //DLOG_INFO(" DataDirect %x\r\n",Reg_Read32(u32_GroupNoAddr + u32_RegNoAddr));
}



_EXT_ITCM void GPIO_SetPinCtrl(uint32_t gpioNum, uint32_t mode)
{
    uint32_t u32_GroupNoAddr = 0;
    uint32_t u32_RegNoAddr = 0;
    uint8_t  u8_RegNo = 0;
    uint8_t  u8_PinNo = 0;
    uint8_t  u8_GroupNo = 0;
    uint32_t u32_GpioRegVal = 0;

    u8_GroupNo = (gpioNum>>5);
    u8_RegNo = (gpioNum%32)>>3;
    u8_PinNo = (gpioNum%32)%8;

    u32_GroupNoAddr = u8_GroupNo*0x40000 + GPIO0_BASE_ADDR;
    u32_RegNoAddr = u8_RegNo*0x0C + GPIO_CTRL_A_OFFSET;
    u32_GpioRegVal = Reg_Read32(u32_GroupNoAddr + u32_RegNoAddr);

    if(GPIO_CTRL_SOFTWARE == mode)
    {
        u32_GpioRegVal &= ~(1 << u8_PinNo);
    }
    else
    {
        u32_GpioRegVal |= (mode << u8_PinNo);
    }

    Reg_Write32(u32_GroupNoAddr + u32_RegNoAddr, u32_GpioRegVal);

    //DLOG_INFO(" SetPinCtrl %x\r\n",Reg_Read32(u32_GroupNoAddr + u32_RegNoAddr));
}



_EXT_ITCM void GPIO_Intr_SetPinIntrEn(uint32_t gpioNum, uint32_t mode)
{
    // uint32_t u32_PinNoAddr = 0;
    uint32_t u32_GroupNoAddr = 0;
    // uint8_t  u8_RegNo = 0;
    uint8_t  u8_PinNo = 0;
    uint8_t  u8_GroupNo = 0;
    uint32_t u32_GpioRegVal = 0;

    u8_GroupNo = (gpioNum>>5);
    // u8_RegNo = (gpioNum%32)>>3;
    u8_PinNo = (gpioNum%32)%8;
    u32_GroupNoAddr = u8_GroupNo*0x40000 + GPIO0_BASE_ADDR;
    u32_GpioRegVal = Reg_Read32(u32_GroupNoAddr + GPIO_INTEN_OFFSET);

    if(GPIO_INTEN_INTERRUPT == mode)
    {
        u32_GpioRegVal |= (mode << u8_PinNo);
    }
    else
    {
        u32_GpioRegVal &= ~(1 << u8_PinNo);
    }

    Reg_Write32(u32_GroupNoAddr + GPIO_INTEN_OFFSET, u32_GpioRegVal);
    //DLOG_INFO(" SetPinInten %x\r\n",Reg_Read32(u32_GroupNoAddr + GPIO_INTEN_OFFSET));

}

_EXT_ITCM void GPIO_Intr_SetPinIntrMask(uint32_t gpioNum, uint32_t mode)
{

    uint32_t u32_GroupNoAddr = 0;
    // uint8_t  u8_RegNo = 0;
    uint8_t  u8_PinNo = 0;
    uint8_t  u8_GroupNo = 0;
    uint32_t u32_GpioRegVal = 0;
    u8_GroupNo = (gpioNum>>5);
    // u8_RegNo = (gpioNum%32)>>3;
    u8_PinNo = (gpioNum%32)%8;
    u32_GroupNoAddr = u8_GroupNo*0x40000 + GPIO0_BASE_ADDR;
    u32_GpioRegVal = Reg_Read32(u32_GroupNoAddr + GPIO_MASK_OFFSET);

    if(GPIO_MASK_MASK == mode)
    {
        u32_GpioRegVal &= ~(1 << u8_PinNo);
    }
    else
    {
        u32_GpioRegVal |= (mode << u8_PinNo);
    }

    Reg_Write32(u32_GroupNoAddr + GPIO_MASK_OFFSET, u32_GpioRegVal);

    //DLOG_INFO(" SetPinMask %x\r\n",Reg_Read32(u32_GroupNoAddr + GPIO_MASK_OFFSET));

}


_EXT_ITCM void GPIO_Intr_SetPinIntrType(uint32_t gpioNum, uint32_t mode)
{
    uint32_t u32_GroupNoAddr = 0;
    // uint8_t  u8_RegNo = 0;
    uint8_t  u8_PinNo = 0;
    uint8_t  u8_GroupNo = 0;
    uint32_t u32_GpioRegVal = 0;
    u8_GroupNo = (gpioNum>>5);
    // u8_RegNo = (gpioNum%32)>>3;
    u8_PinNo = (gpioNum%32)%8;
    u32_GroupNoAddr = u8_GroupNo*0x40000 + GPIO0_BASE_ADDR;
    u32_GpioRegVal = Reg_Read32(u32_GroupNoAddr + GPIO_INTTYPE_OFFSET);

    if(GPIO_INTTYPE_EDGE == mode)
    {
        u32_GpioRegVal |= (mode << u8_PinNo);
    }
    else
    {
        u32_GpioRegVal &= ~(1 << u8_PinNo);
    }

    Reg_Write32(u32_GroupNoAddr + GPIO_INTTYPE_OFFSET, u32_GpioRegVal);

    //DLOG_INFO(" SetPinInttype %x\r\n",Reg_Read32(u32_GroupNoAddr + GPIO_INTTYPE_OFFSET));
}

_EXT_ITCM void GPIO_Intr_SetPinIntrPol(uint32_t gpioNum, uint32_t mode)
{

    uint32_t u32_GroupNoAddr = 0;
    // uint8_t  u8_RegNo = 0;
    uint8_t  u8_PinNo = 0;
    uint8_t  u8_GroupNo = 0;
    uint32_t u32_GpioRegVal = 0;
    u8_GroupNo = (gpioNum>>5);
    // u8_RegNo = (gpioNum%32)>>3;
    u8_PinNo = (gpioNum%32)%8;

    u32_GroupNoAddr = u8_GroupNo*0x40000 + GPIO0_BASE_ADDR;

    u32_GpioRegVal = Reg_Read32(u32_GroupNoAddr + GPIO_INTPOL_OFFSET);

    if(GPIO_INTPOL_HIGH == mode)
    {
        u32_GpioRegVal |= (mode << u8_PinNo);
    }
    else
    {
        u32_GpioRegVal &= ~(1 << u8_PinNo);
    }

    Reg_Write32(u32_GroupNoAddr + GPIO_INTPOL_OFFSET, u32_GpioRegVal);

    //DLOG_INFO(" SetPinIntpol %x\r\n",Reg_Read32(u32_GroupNoAddr + GPIO_INTPOL_OFFSET));
}




_EXT_ITCM uint32_t GPIO_Intr_GetIntrStatus(uint32_t gpioNum)
{
    uint32_t u32_GroupNoAddr = 0;
    // uint8_t  u8_RegNo = 0;
    uint8_t u8_PinNo = 0;
    uint8_t u8_GroupNo = 0;
    uint32_t u32_GpioRegVal = 0;
    u8_GroupNo = (gpioNum>>5);
    // u8_RegNo = (gpioNum%32)>>3;
    u8_PinNo = (gpioNum%32)%8;

    u32_GroupNoAddr = u8_GroupNo*0x40000 + GPIO0_BASE_ADDR;

    u32_GpioRegVal = Reg_Read32(u32_GroupNoAddr + GPIO_INTSTATUS_OFFSET);

    return ((u32_GpioRegVal >> u8_PinNo) & 1);
}



_EXT_ITCM uint32_t GPIO_Intr_GetIntrGroupStatus(uint32_t u32_vectorNum)
{
    uint32_t u32_GroupNoAddr = 0;

    switch(u32_vectorNum)
    {
        case GPIO_INTR_N0_VECTOR_NUM:
        {
            u32_GroupNoAddr = GPIO0_BASE_ADDR;
            break;
        }
        case GPIO_INTR_N1_VECTOR_NUM:
        {
            u32_GroupNoAddr = GPIO1_BASE_ADDR;
            break;
        }
        case GPIO_INTR_N2_VECTOR_NUM:
        {
            u32_GroupNoAddr = GPIO2_BASE_ADDR;
            break;
        }
        case GPIO_INTR_N3_VECTOR_NUM:
        {
            u32_GroupNoAddr = GPIO3_BASE_ADDR;
            break;
        }
        default:
            return 0;
    }

    return Reg_Read32(u32_GroupNoAddr + GPIO_INTSTATUS_OFFSET);
}



_EXT_ITCM uint32_t GPIO_Intr_GetRawIntrStatus(uint32_t gpioNum)
{
    uint32_t u32_GroupNoAddr = 0;
    // uint8_t  u8_RegNo = 0;
    uint8_t  u8_PinNo = 0;
    uint8_t  u8_GroupNo = 0;
    uint32_t u32_GpioRegVal = 0;

    u8_GroupNo = (gpioNum>>5);
    // u8_RegNo = (gpioNum%32)>>3;
    u8_PinNo = (gpioNum%32)%8;

    u32_GroupNoAddr = u8_GroupNo*0x40000 + GPIO0_BASE_ADDR;

    u32_GpioRegVal = Reg_Read32(u32_GroupNoAddr + GPIO_RAW_INTSTATUS_OFFSET);

    return ((u32_GpioRegVal >> u8_PinNo) & 1);
}



_EXT_ITCM void GPIO_Intr_ClearIntr(uint32_t gpioNum)
{
    uint32_t u32_GroupNoAddr = 0;
    // uint8_t u8_RegNo = 0;
    uint8_t u8_PinNo = 0;
    uint8_t u8_GroupNo = 0;
    uint32_t u32_GpioRegVal = 0;

    u8_GroupNo = (gpioNum>>5);

    u8_PinNo = (gpioNum%32)%8;

    u32_GroupNoAddr = u8_GroupNo*0x40000 + GPIO0_BASE_ADDR;

    //DLOG_INFO(" ClearInt %x %x \r\n",u32_GpioRegVal,GPIO_Intr_GetIntrStatus(gpioNum));

    u32_GpioRegVal |= 1 << u8_PinNo;

    Reg_Write32(u32_GroupNoAddr + GPIO_CLEARINT_OFFSET, u32_GpioRegVal);
}



_EXT_ITCM void GPIO_Intr_ClearIntrGroup(uint32_t u32_vectorNum, uint8_t u8_flag)
{
    uint32_t u32_GroupNoAddr = 0;

    switch(u32_vectorNum)
    {
        case GPIO_INTR_N0_VECTOR_NUM:
        {
            u32_GroupNoAddr = GPIO0_BASE_ADDR + GPIO_CLEARINT_OFFSET;
            break;
        }
        case GPIO_INTR_N1_VECTOR_NUM:
        {
            u32_GroupNoAddr = GPIO1_BASE_ADDR + GPIO_CLEARINT_OFFSET;
            break;
        }
        case GPIO_INTR_N2_VECTOR_NUM:
        {
            u32_GroupNoAddr = GPIO2_BASE_ADDR + GPIO_CLEARINT_OFFSET;
            break;
        }
        case GPIO_INTR_N3_VECTOR_NUM:
        {
            u32_GroupNoAddr = GPIO3_BASE_ADDR + GPIO_CLEARINT_OFFSET;
            break;
        }
        default:
            return ;
    }

    Reg_Write32(u32_GroupNoAddr, u8_flag);

}


// static void GPIO_VectorFunctionN0(uint32_t u32_vectorNum);

// static void GPIO_VectorFunctionN1(uint32_t u32_vectorNum);

// static void GPIO_VectorFunctionN2(uint32_t u32_vectorNum);

// static void GPIO_VectorFunctionN3(uint32_t u32_vectorNum);

//static void GPIO_VectorFunctionNULL(uint32_t u32_vectorNum);



// static void (*g_pv_GpioVectorNumArray[4])(uint32_t u32_vectorNum)={ GPIO_VectorFunctionN0,
//                                                                     GPIO_VectorFunctionN1,
//                                                                     GPIO_VectorFunctionN2,
//                                                                     GPIO_VectorFunctionN3};

//static void (*g_pv_GpioVectorListArray[4][8])(uint32_t u32_vectorNum);



//static void GPIO_ClearNvic(uint32_t u32_vectorNum);






_EXT_ITCM HAL_RET_T HAL_GPIO_SetMode(ENUM_HAL_GPIO_NUM e_gpioPin,ENUM_HAL_GPIO_PinMode e_gpioMode)
{
    if ((e_gpioPin > HAL_GPIO_NUM127) && (e_gpioMode > HAL_GPIO_PIN_MODE2))
    {
        return HAL_GPIO_ERR_UNKNOWN;
    }

    GPIO_SetMode(e_gpioPin, e_gpioMode);

    return HAL_OK;
}




_EXT_ITCM HAL_RET_T HAL_GPIO_OutPut(ENUM_HAL_GPIO_NUM e_gpioPin)
{
    if (e_gpioPin > HAL_GPIO_NUM127)
    {
        return HAL_GPIO_ERR_UNKNOWN;
    }

    GPIO_SetMode(e_gpioPin, HAL_GPIO_PIN_MODE2);
    GPIO_SetPinDirect(e_gpioPin, GPIO_DATA_DIRECT_OUTPUT);
    GPIO_SetPinCtrl(e_gpioPin, GPIO_CTRL_SOFTWARE);

    return HAL_OK;
}




_EXT_ITCM HAL_RET_T HAL_GPIO_InPut(ENUM_HAL_GPIO_NUM e_gpioPin)
{
    if (e_gpioPin > HAL_GPIO_NUM127)
    {
        return HAL_GPIO_ERR_UNKNOWN;
    }

    GPIO_SetMode(e_gpioPin, HAL_GPIO_PIN_MODE2);
    GPIO_SetPinDirect(e_gpioPin, GPIO_DATA_DIRECT_INPUT);
    GPIO_SetPinCtrl(e_gpioPin, GPIO_CTRL_SOFTWARE);
    
    return HAL_OK;
}




_EXT_ITCM HAL_RET_T HAL_GPIO_GetPin(ENUM_HAL_GPIO_NUM e_gpioPin,uint32_t *p_retGpioState)
{
    if (e_gpioPin > HAL_GPIO_NUM127)
    {
        return HAL_GPIO_ERR_UNKNOWN;
    }

    *p_retGpioState = GPIO_GetPin(e_gpioPin);

    return HAL_OK;
}




_EXT_ITCM HAL_RET_T HAL_GPIO_SetPin(ENUM_HAL_GPIO_NUM e_gpioPin, ENUM_HAL_GPIO_PinState e_pinState)
{
    if (e_gpioPin > HAL_GPIO_NUM127)
    {
        return HAL_GPIO_ERR_UNKNOWN;
    }

    GPIO_SetPin(e_gpioPin,e_pinState);

    return HAL_OK;
}




// HAL_RET_T HAL_GPIO_RegisterInterrupt(ENUM_HAL_GPIO_NUM e_gpioPin,
//                                      ENUM_HAL_GPIO_InterrputLevel e_inttype,
//                                      ENUM_HAL_GPIO_InterrputPolarity e_polarity,
//                                      void *fun_callBack)
// {
//     if ((e_gpioPin%32) > 7)
//     {
//         return HAL_GPIO_ERR_UNKNOWN;
//     }

//     g_pv_GpioVectorListArray[e_gpioPin>>5][(e_gpioPin%32)%8] = fun_callBack;

//     reg_IrqHandle(GPIO_INTR_N0_VECTOR_NUM + (e_gpioPin>>5), g_pv_GpioVectorNumArray[e_gpioPin>>5], NULL);

//     GPIO_SetPinDirect(e_gpioPin, GPIO_DATA_DIRECT_INPUT);
    
//     GPIO_SetPinCtrl(e_gpioPin, GPIO_CTRL_SOFTWARE);

//     GPIO_SetMode(e_gpioPin, HAL_GPIO_PIN_MODE2);

//     GPIO_Intr_SetPinIntrEn(e_gpioPin, GPIO_INTEN_INTERRUPT);

//     GPIO_Intr_SetPinIntrMask(e_gpioPin, GPIO_MASK_MASK);

//     GPIO_Intr_SetPinIntrType(e_gpioPin, e_inttype);

//     GPIO_Intr_SetPinIntrPol(e_gpioPin, e_polarity);

//     NVIC_SetPriority(GPIO_INTR_N0_VECTOR_NUM + (e_gpioPin>>5), NVIC_EncodePriority(NVIC_PRIORITYGROUP_5,INTR_NVIC_PRIORITY_GPIO_DEFAULT,0));

//     NVIC_EnableIRQ(GPIO_INTR_N0_VECTOR_NUM + (e_gpioPin>>5));

//     return HAL_OK;

// }




// static void GPIO_ClearNvic(uint32_t u32_vectorNum)

// {
//     uint32_t i = 0;

//     for (i=(u32_vectorNum-66)*32; i<8+(u32_vectorNum-66)*32; i++)
//     {

//         if(GPIO_Intr_GetIntrStatus(i))
//         {
//             GPIO_Intr_ClearIntr(i);
//         }
//     }
// }




// HAL_RET_T HAL_GPIO_DisableNvic(ENUM_HAL_GPIO_NUM e_gpioPin)
// {

//     uint8_t i = 0;
//     uint8_t u8_flg = 0;

//     if ((e_gpioPin%32) > 7)
//     {
//         return HAL_GPIO_ERR_UNKNOWN;
//     }

//     g_pv_GpioVectorListArray[e_gpioPin>>5][(e_gpioPin%32)%8] = NULL;

//     for (i=0;i<8;i++)
//     {
//        if ((g_pv_GpioVectorListArray[e_gpioPin>>5][i]) == NULL)
//        {
//             u8_flg++;
//        }
//     }

//     if(8 == u8_flg)
//     {
//         NVIC_DisableIRQ(GPIO_INTR_N0_VECTOR_NUM + (e_gpioPin>>5));

//         NVIC_ClearPendingIRQ(GPIO_INTR_N0_VECTOR_NUM + (e_gpioPin>>5));
//     }

//     return HAL_OK;

// }





// static void GPIO_VectorFunctionN0(uint32_t u32_vectorNum)
// {
//     uint8_t i = 0;
//     uint8_t u8_intrStattus = GPIO_Intr_GetIntrGroupStatus(u32_vectorNum);
//     GPIO_Intr_ClearIntrGroup(u32_vectorNum,u8_intrStattus);

//     for (i=0; i<8; i++)
//     {

//         if((u8_intrStattus & (0x01 << i)))
//         {
//             if (g_pv_GpioVectorListArray[0][i] != NULL)
//             {
//                 (*(g_pv_GpioVectorListArray[0][i]))(u32_vectorNum);
//             }
//         }
//     }
// }



// static void GPIO_VectorFunctionN1(uint32_t u32_vectorNum)
// {

//     uint8_t i = 0;
//     uint8_t u8_intrStattus = GPIO_Intr_GetIntrGroupStatus(u32_vectorNum);
//     GPIO_Intr_ClearIntrGroup(u32_vectorNum,u8_intrStattus);
//     for (i=0; i<8; i++)
//     {
//         if((u8_intrStattus & (0x01 << i)))
//         {
//             if (g_pv_GpioVectorListArray[1][i] != NULL)
//             {
//                 (*(g_pv_GpioVectorListArray[1][i]))(u32_vectorNum);
//             }
//         }
//     }
// }



// static void GPIO_VectorFunctionN2(uint32_t u32_vectorNum)
// {

//     uint8_t i = 0;

//     uint8_t u8_intrStattus = GPIO_Intr_GetIntrGroupStatus(u32_vectorNum);

//     GPIO_Intr_ClearIntrGroup(u32_vectorNum,u8_intrStattus);

//     for (i=0; i<8; i++)
//     {
//         if((u8_intrStattus & (0x01 << i)))
//         {
//             if (g_pv_GpioVectorListArray[2][i] != NULL)
//             {
//                 (*(g_pv_GpioVectorListArray[2][i]))(u32_vectorNum);
//             }
//         }
//     }
// }



// static void GPIO_VectorFunctionN3(uint32_t u32_vectorNum)
// {
//     uint8_t i = 0;
//     uint8_t u8_intrStattus = GPIO_Intr_GetIntrGroupStatus(u32_vectorNum);
//     GPIO_Intr_ClearIntrGroup(u32_vectorNum,u8_intrStattus);

//     for (i=0; i<8; i++)
//     {
//         if((u8_intrStattus & (0x01 << i)))
//         {
//             if (g_pv_GpioVectorListArray[3][i] != NULL)
//             {
//                 (*(g_pv_GpioVectorListArray[3][i]))(u32_vectorNum);
//             }
//         }
//     }
// }



// static void GPIO_VectorFunctionNULL(uint32_t u32_vectorNum)
// {
//    return;
// }


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  ar_gpioinit
 *
 * Description:
 *   Based on configuration within the .config file, it does:
 *    - Remaps positions of alternative functions.
 *
 *   Typically called from stm32_start().
 *
 * Assumptions:
 *   This function is called early in the initialization sequence so that
 *   no mutual exclusion is necessary.
 *
 ****************************************************************************/

_EXT_ITCM void ar_gpioinit(void)
{
    /* PWM pin: output mode and set low
       UART pin: UART1 UART2
       Other pin: input mode , except pin117-ITAG-TDI
    */

    int i;

    for (i = HAL_GPIO_NUM0; i <= HAL_GPIO_NUM79; i++)
    {
        HAL_GPIO_InPut(i);
    }

    /* PWM pin */
    for (i = HAL_GPIO_NUM80; i <= HAL_GPIO_NUM89; i++)
    {
        HAL_GPIO_OutPut(i);
        HAL_GPIO_SetPin(HAL_GPIO_NUM0, HAL_GPIO_PIN_RESET);
    }

    for (i = HAL_GPIO_NUM90; i <= HAL_GPIO_NUM116; i++)
    {
        /* UART1 UART2 */
        if (i == HAL_GPIO_NUM111 || i <= HAL_GPIO_NUM112)
            continue;

        HAL_GPIO_InPut(i);
    }

}

/****************************************************************************
 * Name: ar_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...)
 *   function, it must be unconfigured with stm32_unconfiggpio() with
 *   the same cfgset first before it can be set to non-alternative function.
 *
 * Returns:
 *   OK on success
 *   A negated errono value on invalid port, or when pin is locked as ALT
 *   function.
 *
 * To-Do: Auto Power Enable
 ****************************************************************************/
_EXT_ITCM int ar_configgpio(uint32_t cfgset)
{
    unsigned int pin;
    unsigned int pinmode;
    unsigned int output;

    pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

    pinmode = cfgset & GPIO_MODE_MASK;

    output = (cfgset & GPIO_OUTPUT_MASK) ? HAL_GPIO_PIN_SET : HAL_GPIO_PIN_RESET;

    if (GPIO_DEFAULT == pinmode)
    {
        HAL_GPIO_SetMode(pin, HAL_GPIO_PIN_MODE0);
    }
    else if (GPIO_OUTPUT == pinmode)
    {
        HAL_GPIO_OutPut(pin);
        HAL_GPIO_SetPin(pin, output);
    }
    else if (GPIO_INPUT == pinmode)
    {
        HAL_GPIO_InPut(pin);
    }
    else
    {
        //AR_LOG("shouldn't  be here,ar_configgpio...\r\n");
    }

    return 0;
    //return SUCCESS;
}

/****************************************************************************
 * Name: ar_unconfiggpio
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set it
 *   into default HiZ state (and possibly mark it's unused) and unlock it whether
 *   it was previously selected as alternative function (GPIO_ALT|GPIO_CNF_AFPP|...).
 *
 *   This is a safety function and prevents hardware from schocks, as unexpected
 *   write to the Timer Channel Output GPIO to fixed '1' or '0' while it should
 *   operate in PWM mode could produce excessive on-board currents and trigger
 *   over-current/alarm function.
 *
 * Returns:
 *  OK on success
 *  A negated errno value on invalid port
 *
 * To-Do: Auto Power Disable
 ****************************************************************************/

_EXT_ITCM int ar_unconfiggpio(uint32_t cfgset)
{
    unsigned int pin;
    pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
    HAL_GPIO_InPut(pin);

    return 0;
    //return SUCCESS;
}

/****************************************************************************
 * Name: ar_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

_EXT_ITCM void ar_gpiowrite(uint32_t pinset, bool value)
{
    HAL_GPIO_SetPin(pinset, value);
}

/****************************************************************************
 * Name: ar_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

_EXT_ITCM bool ar_gpioread(uint32_t pinset)
{
    uint32_t retGpioState = 0;
    HAL_GPIO_GetPin(pinset, &retGpioState);

    return retGpioState;
}


