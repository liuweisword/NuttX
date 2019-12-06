/****************************************************************************
 * arch/arm/src/artosyn/ar_can.c
 *
 *   Copyright (C) 2011, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright (C) 2016 Omni Hoverboards Inc. All rights reserved.
 *   Author: Paul Alexander Patience <paul-a.patience@polymtl.ca>
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

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/can/can.h>
#include <chip/ar8020_memorymap.h>
#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "ar_can.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Delays *******************************************************************/
/* Time out for INAK bit */

#define INAK_TIMEOUT 65535

/* Bit timing ***************************************************************/

#define CAN_BIT_QUANTA (CONFIG_CAN_TSEG1 + CONFIG_CAN_TSEG2 + 1)

#ifndef CONFIG_DEBUG_CAN_INFO
#  undef CONFIG_AR_CAN_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
struct ar_can_s
{
  uint8_t  port;     /* CAN port number (1 or 2) */
  uint32_t base;     /* Base address of the CAN control registers */
  uint32_t ev_irq; /* Event IRQ */
  uint8_t  filter;   /* Filter number */
  uint32_t fbase;    /* Base address of the CAN filter registers */
  ENUM_CAN_BAUDR baud;     /* Configured baud */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CAN Register access */

// static uint32_t arcan_getreg(FAR struct ar_can_s *priv,
//                                 int offset);
// static uint32_t arcan_getfreg(FAR struct ar_can_s *priv,
//                                  int offset);
// static void arcan_putreg(FAR struct ar_can_s *priv, int offset,
//                             uint32_t value);
// static void arcan_putfreg(FAR struct ar_can_s *priv, int offset,
//                              uint32_t value);


#  define arcan_dumpctrlregs(priv,msg)
#  define arcan_dumpmbregs(priv,msg)
#  define arcan_dumpfiltregs(priv,msg)

/* Filtering (todo) */

// static int  arcan_addstdfilter(FAR struct ar_can_s *priv,
//                                   FAR struct canioc_stdfilter_s *arg);
// static int  arcan_delstdfilter(FAR struct ar_can_s *priv,
//                                   int arg);

/* CAN driver methods */

static void arcan_reset(FAR struct can_dev_s *dev);
static int  arcan_setup(FAR struct can_dev_s *dev);
static void arcan_shutdown(FAR struct can_dev_s *dev);
static void arcan_rxint(FAR struct can_dev_s *dev, bool enable);
static void arcan_txint(FAR struct can_dev_s *dev, bool enable);
static int  arcan_ioctl(FAR struct can_dev_s *dev, int cmd,
                           unsigned long arg);
static int  arcan_remoterequest(FAR struct can_dev_s *dev,
                                   uint16_t id);
static int  arcan_send(FAR struct can_dev_s *dev,
                          FAR struct can_msg_s *msg);
static bool arcan_txready(FAR struct can_dev_s *dev);
static bool arcan_txempty(FAR struct can_dev_s *dev);


/* CAN interrupt handling */
static int  arcan_interrupt(int irq, FAR void *context, FAR void *arg);

// static int  arcan_enterinitmode(FAR struct ar_can_s *priv);
// static int  arcan_exitinitmode(FAR struct ar_can_s *priv);
// static int  arcan_bittiming(FAR struct ar_can_s *priv);
static int  arcan_cellinit(FAR struct ar_can_s *priv);
static int  arcan_filterinit(FAR struct ar_can_s *priv);

/* TX mailbox status */

// static bool arcan_txmb0empty(uint32_t tsr_regval);
// static bool arcan_txmb1empty(uint32_t tsr_regval);
// static bool arcan_txmb2empty(uint32_t tsr_regval);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_canops =
{
  .co_reset         = arcan_reset,
  .co_setup         = arcan_setup,
  .co_shutdown      = arcan_shutdown,
  .co_rxint         = arcan_rxint,
  .co_txint         = arcan_txint,
  .co_ioctl         = arcan_ioctl,
  .co_remoterequest = arcan_remoterequest,
  .co_send          = arcan_send,
  .co_txready       = arcan_txready,
  .co_txempty       = arcan_txempty,
};

#ifdef CONFIG_AR_CAN0
static struct ar_can_s g_can0priv =
{
  .port             = 0,
  .ev_irq           = AR_IRQ_CAN0,
  .filter           = 0,
  .base             = AR_CAN0_BASE,
  .fbase            = AR_CAN0_BASE,
  .baud             = CAN_BAUDR_500K,
};

static struct can_dev_s g_can0dev =
{
  .cd_ops           = &g_canops,
  .cd_priv          = &g_can0priv,
};
#endif

#ifdef CONFIG_AR_CAN1
static struct ar_can_s g_can1priv =
{
  .port             = 0,
  .ev_irq           = AR_IRQ_CAN1,
  .filter           = 0,
  .base             = AR_CAN1_BASE,
  .fbase            = AR_CAN1_BASE,
  .baud             = CAN_BAUDR_500K,
};

static struct can_dev_s g_can1dev =
{
  .cd_ops           = &g_canops,
  .cd_priv          = &g_can1priv,
};
#endif


/****************************************************************************
 * Private Functions
 ****************************************************************************/

// static uint32_t arcan_getreg(FAR struct ar_can_s *priv, int offset)
// {
//   return getreg32(priv->base + offset);
// }

// static uint32_t arcan_getfreg(FAR struct ar_can_s *priv, int offset)
// {
//   return getreg32(priv->fbase + offset);
// }


/****************************************************************************
 * Name: arcan_putreg
 * Name: arcan_putfreg
 *
 * Description:
 *   Set the value of a CAN register or filter block register.
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *   offset - The offset to the register to write
 *   value - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

// static void arcan_putreg(FAR struct ar_can_s *priv, int offset,
//                             uint32_t value)
// {
//   putreg32(value, priv->base + offset);
// }

// static void arcan_putfreg(FAR struct ar_can_s *priv, int offset,
//                              uint32_t value)
// {
//   putreg32(value, priv->fbase + offset);
// }



/****************************************************************************
 * Name: arcan_reset
 *
 * Description:
 *   Reset the CAN device.  Called early to initialize the hardware. This
 *   function is called, before stm32can_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void arcan_reset(FAR struct can_dev_s *dev)
{
  FAR struct ar_can_s *priv = dev->cd_priv;

  caninfo("arcan_reset CAN%d\n", priv->port);

  irqstate_t flags = enter_critical_section();  

  up_disable_irq(priv->ev_irq);
  irq_detach(priv->ev_irq);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: arcan_setup
 *
 * Description:
 *   Configure the CAN. This method is called the first time that the CAN
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching CAN interrupts.
 *   All CAN interrupts are disabled upon return.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int arcan_setup(FAR struct can_dev_s *dev)
{
  FAR struct ar_can_s *priv = dev->cd_priv;
  int ret;

  caninfo("CAN%d  irq: %d \n", priv->port, priv->ev_irq);

  if ((ret = arcan_cellinit(priv)) != OK)
  {
    canerr("ERROR: CAN%d cell initialization failed: %d\n",
            priv->port, ret);
    return ret;
  }

  arcan_dumpctrlregs(priv, "After cell initialization");
  arcan_dumpmbregs(priv, NULL);

  /* CAN filter initialization */ 
  if((ret = arcan_filterinit(priv)) !=- OK)
  {
    canerr("ERROR: CAN%d filter initialization failed: %d\n",
             priv->port, ret);
    return ret;
  }

  arcan_dumpfiltregs(priv, "After filter initialization");

  if((ret = irq_attach(priv->ev_irq, arcan_interrupt, dev)) != OK)
  {
    canerr("ERROR: Failed to attach CAN%d IRQ (%d)",
             priv->port, priv->ev_irq);
    return ret;
  }

  up_enable_irq(priv->ev_irq);

  return ret;
}

/****************************************************************************
 * Name: arcan_shutdown
 *
 * Description:
 *   Disable the CAN.  This method is called when the CAN device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void arcan_shutdown(FAR struct can_dev_s *dev)
{
  FAR struct ar_can_s *priv = dev->cd_priv;

  caninfo("arcan_shutdown CAN%d\n", priv->port);

  // Disable interrupts
  up_disable_irq(priv->ev_irq);
  irq_detach(priv->ev_irq);

  arcan_reset(dev);
}

/****************************************************************************
 * Name: arcan_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void arcan_rxint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct ar_can_s *priv = (struct ar_can_s *)dev->cd_priv;

  volatile STRU_CAN_TYPE *pst_canReg = (STRU_CAN_TYPE *)priv->base;

  if (enable)
  {
    pst_canReg->u32_reg4 |= (0xF0); // set RIE / ROIE / RFIE / RAFIE
  }
  else
  {
    pst_canReg->u32_reg4 &= ~(0xF0); // clear RIE / ROIE / RFIE / RAFIE    
  }

  caninfo("arcan_rxint CAN%d enable: 0x%02x\n", priv->port, pst_canReg->u32_reg4);
}

/****************************************************************************
 * Name: arcan_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void arcan_txint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct ar_can_s *priv = dev->cd_priv;
  // uint32_t regval;

  caninfo("arcan_txint CAN%d enable: %d\n", priv->port, enable);

  /* Support only disabling the transmit mailbox interrupt */

  if (!enable)
    {
// TODO        
//       regval  = stm32can_getreg(priv, STM32_CAN_IER_OFFSET);
//       regval &= ~CAN_IER_TMEIE;
//       stm32can_putreg(priv, STM32_CAN_IER_OFFSET, regval);
    }
}

/****************************************************************************
 * Name: arcan_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/
// /*
// static int arcan_ioctl(FAR struct can_dev_s *dev, int cmd,
//                           unsigned long arg)
// {
//   FAR struct ar_can_s *priv;
//   int ret = -ENOTTY;
// 
//   caninfo("cmd=%04x arg=%lu\n", cmd, arg);
// 
//   DEBUGASSERT(dev && dev->cd_priv);
//   priv = dev->cd_priv;
// 
//   /* Handle the command */
// 
//   switch (cmd)
//     {
//       /* CANIOC_GET_BITTIMING:
//        *   Description:    Return the current bit timing settings
//        *   Argument:       A pointer to a write-able instance of struct
//        *                   canioc_bittiming_s in which current bit timing
//        *                   values will be returned.
//        *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
//        *                   (ERROR) is returned with the errno variable set
//        *                   to indicate the nature of the error.
//        *   Dependencies:   None
//        */
// 
//       case CANIOC_GET_BITTIMING:
//         {
//           FAR struct canioc_bittiming_s *bt =
//             (FAR struct canioc_bittiming_s *)arg;
//           uint32_t regval;
//           uint32_t brp;
// 
//           DEBUGASSERT(bt != NULL);
//           regval       = stm32can_getreg(priv, STM32_CAN_BTR_OFFSET);
//           bt->bt_sjw   = ((regval & CAN_BTR_SJW_MASK) >> CAN_BTR_SJW_SHIFT) + 1;
//           bt->bt_tseg1 = ((regval & CAN_BTR_TS1_MASK) >> CAN_BTR_TS1_SHIFT) + 1;
//           bt->bt_tseg2 = ((regval & CAN_BTR_TS2_MASK) >> CAN_BTR_TS2_SHIFT) + 1;
// 
//           brp          = ((regval & CAN_BTR_BRP_MASK) >> CAN_BTR_BRP_SHIFT) + 1;
//           bt->bt_baud  = STM32_PCLK1_FREQUENCY /
//                          (brp * (bt->bt_tseg1 + bt->bt_tseg2 + 1));
//           ret = OK;
//         }
//         break;
// 
//       /* CANIOC_SET_BITTIMING:
//        *   Description:    Set new current bit timing values
//        *   Argument:       A pointer to a read-able instance of struct
//        *                   canioc_bittiming_s in which the new bit timing
//        *                   values are provided.
//        *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
//        *                   (ERROR)is returned with the errno variable set
//        *                    to indicate thenature of the error.
//        *   Dependencies:   None
//        *
//        * REVISIT: There is probably a limitation here:  If there are multiple
//        * threads trying to send CAN packets, when one of these threads
//        * reconfigures the bitrate, the MCAN hardware will be reset and the
//        * context of operation will be lost.  Hence, this IOCTL can only safely
//        * be executed in quiescent time periods.
//        */
// 
//       case CANIOC_SET_BITTIMING:
//         {
//           FAR const struct canioc_bittiming_s *bt =
//             (FAR const struct canioc_bittiming_s *)arg;
//           uint32_t brp;
//           uint32_t can_bit_quanta;
//           uint32_t tmp;
//           uint32_t regval;
// 
//           DEBUGASSERT(bt != NULL);
//           DEBUGASSERT(bt->bt_baud < STM32_PCLK1_FREQUENCY);
//           DEBUGASSERT(bt->bt_sjw > 0 && bt->bt_sjw <= 4);
//           DEBUGASSERT(bt->bt_tseg1 > 0 && bt->bt_tseg1 <= 16);
//           DEBUGASSERT(bt->bt_tseg2 > 0 && bt->bt_tseg2 <=  8);
// 
//           regval = stm32can_getreg(priv, STM32_CAN_BTR_OFFSET);
// 
//           /* Extract bit timing data */
//           /* tmp is in clocks per bit time */
// 
//           tmp = STM32_PCLK1_FREQUENCY / bt->bt_baud;
// 
//           /* This value is dynamic as requested by user */
// 
//           can_bit_quanta = bt->bt_tseg1 + bt->bt_tseg2 + 1;
// 
//           if (tmp < can_bit_quanta)
//             {
//               /* This timing is not possible */
// 
//               ret = -EINVAL;
//               break;
//             }
// 
//           /* Otherwise, nquanta is can_bit_quanta, ts1 and ts2 are
//            * provided by the user and we calculate brp to achieve
//            * can_bit_quanta quanta in the bit times
//            */
// 
//           else
//             {
//               brp = (tmp + (can_bit_quanta/2)) / can_bit_quanta;
//               DEBUGASSERT(brp >= 1 && brp <= CAN_BTR_BRP_MAX);
//             }
// 
//           caninfo("TS1: %d TS2: %d BRP: %d\n",
//                   bt->bt_tseg1, bt->bt_tseg2, brp);
// 
//           /* Configure bit timing. */
// 
//           regval &= ~(CAN_BTR_BRP_MASK | CAN_BTR_TS1_MASK |
//                       CAN_BTR_TS2_MASK | CAN_BTR_SJW_MASK);
//           regval |= ((brp          - 1) << CAN_BTR_BRP_SHIFT) |
//                     ((bt->bt_tseg1 - 1) << CAN_BTR_TS1_SHIFT) |
//                     ((bt->bt_tseg2 - 1) << CAN_BTR_TS2_SHIFT) |
//                     ((bt->bt_sjw   - 1) << CAN_BTR_SJW_SHIFT);
// 
//           /* Bit timing can only be configured in init mode. */
// 
//           ret = stm32can_enterinitmode(priv);
//           if (ret < 0)
//             {
//               break;
//             }
// 
//           stm32can_putreg(priv, STM32_CAN_BTR_OFFSET, regval);
// 
//           ret = stm32can_exitinitmode(priv);
//           if (ret >= 0)
//             {
//               priv->baud  = STM32_PCLK1_FREQUENCY /
//                 (brp * (bt->bt_tseg1 + bt->bt_tseg2 + 1));
//             }
//         }
//         break;
// 
//       /* CANIOC_GET_CONNMODES:
//        *   Description:    Get the current bus connection modes
//        *   Argument:       A pointer to a write-able instance of struct
//        *                   canioc_connmodes_s in which the new bus modes will
//        *                   be returned.
//        *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
//        *                   (ERROR)is returned with the errno variable set
//        *                   to indicate the nature of the error.
//        *   Dependencies:   None
//        */
// 
//       case CANIOC_GET_CONNMODES:
//         {
//           FAR struct canioc_connmodes_s *bm =
//             (FAR struct canioc_connmodes_s *)arg;
//           uint32_t regval;
// 
//           DEBUGASSERT(bm != NULL);
// 
//           regval          = stm32can_getreg(priv, STM32_CAN_BTR_OFFSET);
// 
//           bm->bm_loopback = ((regval & CAN_BTR_LBKM) == CAN_BTR_LBKM);
//           bm->bm_silent   = ((regval & CAN_BTR_SILM) == CAN_BTR_SILM);
//           ret = OK;
//           break;
//         }
// 
//       /* CANIOC_SET_CONNMODES:
//        *   Description:    Set new bus connection modes values
//        *   Argument:       A pointer to a read-able instance of struct
//        *                   canioc_connmodes_s in which the new bus modes
//        *                   are provided.
//        *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
//        *                   (ERROR) is returned with the errno variable set
//        *                   to indicate the nature of the error.
//        *   Dependencies:   None
//        */
// 
//       case CANIOC_SET_CONNMODES:
//         {
//           FAR struct canioc_connmodes_s *bm =
//             (FAR struct canioc_connmodes_s *)arg;
//           uint32_t regval;
// 
//           DEBUGASSERT(bm != NULL);
// 
//           regval = stm32can_getreg(priv, STM32_CAN_BTR_OFFSET);
// 
//           if (bm->bm_loopback)
//             {
//               regval |= CAN_BTR_LBKM;
//             }
//           else
//             {
//               regval &= ~CAN_BTR_LBKM;
//             }
// 
//           if (bm->bm_silent)
//             {
//               regval |= CAN_BTR_SILM;
//             }
//           else
//             {
//               regval &= ~CAN_BTR_SILM;
//             }
// 
//           /* This register can only be configured in init mode. */
// 
//           ret = stm32can_enterinitmode(priv);
//           if (ret < 0)
//             {
//               break;
//             }
// 
//           stm32can_putreg(priv, STM32_CAN_BTR_OFFSET, regval);
// 
//           ret = stm32can_exitinitmode(priv);
//         }
//         break;
// 
// #ifdef CONFIG_CAN_EXTID
//       /* CANIOC_ADD_EXTFILTER:
//        *   Description:    Add an address filter for a extended 29 bit
//        *                   address.
//        *   Argument:       A reference to struct canioc_extfilter_s
//        *   Returned Value: A non-negative filter ID is returned on success.
//        *                   Otherwise -1 (ERROR) is returned with the errno
//        *                   variable set to indicate the nature of the error.
//        */
// 
//       case CANIOC_ADD_EXTFILTER:
//         {
//           DEBUGASSERT(arg != 0);
//           ret = stm32can_addextfilter(priv,
//                                       (FAR struct canioc_extfilter_s *)arg);
//         }
//         break;
// 
//       /* CANIOC_DEL_EXTFILTER:
//        *   Description:    Remove an address filter for a standard 29 bit
//        *                   address.
//        *   Argument:       The filter index previously returned by the
//        *                   CANIOC_ADD_EXTFILTER command
//        *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
//        *                   (ERROR)is returned with the errno variable set
//        *                   to indicate the nature of the error.
//        */
// 
//       case CANIOC_DEL_EXTFILTER:
//         {
// #if 0 /* Unimplemented */
//           DEBUGASSERT(arg <= priv->config->nextfilters);
// #endif
//           ret = stm32can_delextfilter(priv, (int)arg);
//         }
//         break;
// #endif
// 
//       /* CANIOC_ADD_STDFILTER:
//        *   Description:    Add an address filter for a standard 11 bit
//        *                   address.
//        *   Argument:       A reference to struct canioc_stdfilter_s
//        *   Returned Value: A non-negative filter ID is returned on success.
//        *                   Otherwise -1 (ERROR) is returned with the errno
//        *                   variable set to indicate the nature of the error.
//        */
// 
//       case CANIOC_ADD_STDFILTER:
//         {
//           DEBUGASSERT(arg != 0);
//           ret = stm32can_addstdfilter(priv,
//                                       (FAR struct canioc_stdfilter_s *)arg);
//         }
//         break;
// 
//       /* CANIOC_DEL_STDFILTER:
//        *   Description:    Remove an address filter for a standard 11 bit
//        *                   address.
//        *   Argument:       The filter index previously returned by the
//        *                   CANIOC_ADD_STDFILTER command
//        *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
//        *                   (ERROR) is returned with the errno variable set
//        *                   to indicate the nature of the error.
//        */
// 
//       case CANIOC_DEL_STDFILTER:
//         {
// #if 0 /* Unimplemented */
//           DEBUGASSERT(arg <= priv->config->nstdfilters);
// #endif
//           ret = stm32can_delstdfilter(priv, (int)arg);
//         }
//         break;
// 
//       /* Unsupported/unrecognized command */
// 
//       default:
//         canerr("ERROR: Unrecognized command: %04x\n", cmd);
//         break;
//     }
// 
//   return ret;
// }*/
// /*
static int arcan_ioctl(FAR struct can_dev_s *dev, int cmd,
                          unsigned long arg)
{
    caninfo("arcan_ioctl can data erase \n");

  
  // FAR struct ar_can_s *priv;
  int ret = -ENOTTY;

  // caninfo("cmd=%04x arg=%lu\n", cmd, arg);

  // DEBUGASSERT(dev && dev->cd_priv);
  // priv = dev->cd_priv;

  // /* Handle the command */

  // // TODO

  return ret;
}

/****************************************************************************
 * Name: arcan_remoterequest
 *
 * Description:
 *   Send a remote request
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int arcan_remoterequest(FAR struct can_dev_s *dev, uint16_t id)
{
#warning "Remote request not implemented"
  return -ENOSYS;
}

/****************************************************************************
 * Name: arcan_send
 *
 * Description:
 *    Send one can message.
 *
 *    One CAN-message consists of a maximum of 10 bytes.  A message is
 *    composed of at least the first 2 bytes (when there are no data bytes).
 *
 *    Byte 0:      Bits 0-7: Bits 3-10 of the 11-bit CAN identifier
 *    Byte 1:      Bits 5-7: Bits 0-2 of the 11-bit CAN identifier
 *                 Bit 4:    Remote Tranmission Request (RTR)
 *                 Bits 0-3: Data Length Code (DLC)
 *    Bytes 2-10: CAN data
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int arcan_send(FAR struct can_dev_s *dev,
                         FAR struct can_msg_s *msg)
{
    return OK;
}

/****************************************************************************
 * Name: arcan_txready
 *
 * Description:
 *   Return true if the CAN hardware can accept another TX message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if the CAN hardware is ready to accept another TX message.
 *
 ****************************************************************************/

static bool arcan_txready(FAR struct can_dev_s *dev)
{
    return OK;
}

/****************************************************************************
 * Name: arcan_txempty
 *
 * Description:
 *   Return true if all message have been sent.  If for example, the CAN
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.  This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware before calling
 *   co_shutdown().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if there are no pending TX transfers in the CAN hardware.
 *
 ****************************************************************************/

static bool arcan_txempty(FAR struct can_dev_s *dev)
{
    return OK;
}

/****************************************************************************
 * Name: arcan_interrupt
 *
 * Description:
 *   CAN interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *   rxmb - The RX mailbox number.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/
static int arcan_interrupt(int irq, FAR void *context, FAR void *arg)
{
  struct can_dev_s *dev = (struct can_dev_s *)arg;
  
  struct can_hdr_s hdr = {0};

  uint32_t buffer[4] = {0};

  FAR struct ar_can_s *priv = (struct ar_can_s *)(dev->cd_priv);

  volatile STRU_CAN_TYPE *pst_canReg = (STRU_CAN_TYPE *)priv->base;

  if ((pst_canReg->u32_reg4 & 0x8000) != 0) 
  {
      // clear Receive Irq
      pst_canReg->u32_reg4 |= 0x8000;

      uint32_t rstat = ((pst_canReg->u32_reg3) & (3 << 24)) >> 24;

      if (rstat == 0) 
      {
        // empty
        return ERROR;
      }

      uint32_t rxcontrolData = pst_canReg->u32_rxBuf[1];

      if ((rxcontrolData & 1<<7) == 0) // std frame
      {
        hdr.ch_id = pst_canReg->u32_rxBuf[0] & CAN_AMASK_ID10_0;
      }
      else //ext frame
      {
#ifdef CONFIG_CAN_EXTID
        hdr.ch_id = pst_canReg->u32_rxBuf[0] & CAN_AMASK_ID28_0;
        hdr.ch_extid = 1;
        canerr("ERROR: not support ext frame\n");
#endif
      }

      hdr.ch_rtr = (rxcontrolData & 1<<6);
      hdr.ch_dlc = (rxcontrolData & 0xf);

      for (size_t i = 0; i < hdr.ch_dlc / 4; i)
      {
          buffer[i] = pst_canReg->u32_rxBuf[2+i];
      }

      

      can_receive(dev, &hdr, (uint8_t *)buffer[0]);
      // release 
      pst_canReg->u32_reg3 |= (1<<28);


  }
  else if ((pst_canReg->u32_reg4 & 0x0c00) != 0) 
  {
    pst_canReg->u32_reg4 |=0x0c000;
  }
  else 
  {
    canerr("some error: 0x%x \n", pst_canReg->u32_reg4);
  }

  return OK;
}




/****************************************************************************
 * Name: arcan_bittiming
 *
 * Description:
 *   Set the CAN bit timing register (BTR) based on the configured BAUD.
 *
 * "The bit timing logic monitors the serial bus-line and performs sampling
 *  and adjustment of the sample point by synchronizing on the start-bit edge
 *  and resynchronizing on the following edges.
 *
 * "Its operation may be explained simply by splitting nominal bit time into
 *  three segments as follows:
 *
 * 1. "Synchronization segment (SYNC_SEG): a bit change is expected to occur
 *     within this time segment. It has a fixed length of one time quantum
 *     (1 x tCAN).
 * 2. "Bit segment 1 (BS1): defines the location of the sample point. It
 *     includes the PROP_SEG and PHASE_SEG1 of the CAN standard. Its duration
 *     is programmable between 1 and 16 time quanta but may be automatically
 *     lengthened to compensate for positive phase drifts due to differences
 *     in the frequency of the various nodes of the network.
 * 3. "Bit segment 2 (BS2): defines the location of the transmit point. It
 *     represents the PHASE_SEG2 of the CAN standard. Its duration is
 *     programmable between 1 and 8 time quanta but may also be automatically
 *     shortened to compensate for negative phase drifts."
 *
 * Pictorially:
 *
 *  |<----------------- NOMINAL BIT TIME ----------------->|
 *  |<- SYNC_SEG ->|<------ BS1 ------>|<------ BS2 ------>|
 *  |<---- Tq ---->|<----- Tbs1 ------>|<----- Tbs2 ------>|
 *
 * Where
 *   Tbs1 is the duration of the BS1 segment
 *   Tbs2 is the duration of the BS2 segment
 *   Tq is the "Time Quantum"
 *
 * Relationships:
 *
 *   baud = 1 / bit_time
 *   bit_time = Tq + Tbs1 + Tbs2
 *   Tbs1 = Tq * ts1
 *   Tbs2 = Tq * ts2
 *   Tq = brp * Tpclk1
 *   baud = Fpclk1 / (brp  * (1 + ts1 + ts2))
 *
 * Where:
 *   Tpclk1 is the period of the APB1 clock (PCLK1).
 *
 * Input Parameter:
 *   priv - A reference to the CAN block status
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

// static int arcan_bittiming(FAR struct ar_can_s *priv)
// {
//     return OK;
// }

/****************************************************************************
 * Name: arcan_enterinitmode
 *
 * Description:
 *   Put the CAN cell in Initialization mode. This only disconnects the CAN
 *   peripheral, no registers are changed. The initialization mode is
 *   required to change the baud rate.
 *
 * Input Parameter:
 *   priv - A pointer to the private data structure for this CAN block
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

// static int arcan_enterinitmode(FAR struct ar_can_s *priv)
// {
//     return OK;
// }

/****************************************************************************
 * Name: arcan_exitinitmode
 *
 * Description:
 *   Put the CAN cell out of the Initialization mode (to Normal mode)
 *
 * Input Parameter:
 *   priv - A pointer to the private data structure for this CAN block
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

// static int arcan_exitinitmode(FAR struct ar_can_s *priv)
// {
//     return OK;
// }

/****************************************************************************
 * Name: arcan_cellinit
 *
 * Description:
 *   CAN cell initialization
 *
 * Input Parameter:
 *   priv - A pointer to the private data structure for this CAN block
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int arcan_cellinit(FAR struct ar_can_s *priv)
{   
    volatile STRU_CAN_TYPE *pst_canReg = (STRU_CAN_TYPE *)priv->base;

    // CFG_STAT-->RESET=1
    pst_canReg->u32_reg3 |= (1<<7);

    // clear S_Seg_1,S_Seg_2,S_SJW
    unsigned int u32_tmpData =  (pst_canReg->u32_reg5) & (~(0x3F | (0x1F<<8) | (0xF<<16)));

    // set S_Seg_1=16,S_Seg_2=13,S_SJW=2, BT=((S_Seg_1+2)+(S_Seg_2+1))*TQ = 32TQ
    u32_tmpData |= ((0x10) | (0xD<<8) | (0x2<<16));
    pst_canReg->u32_reg5 = u32_tmpData;

    //clear S_PRESC
    pst_canReg->u32_reg6 &= (~0xFF);    

    // need check, now is not correct
    switch(priv->baud)
    {
        case CAN_BAUDR_125K:
        { 
            pst_canReg->u32_reg6 |= 0x1F;     
            break;
        }
        case CAN_BAUDR_250K:
        {
            pst_canReg->u32_reg6 |= 0xF;
            break;
        }
        case CAN_BAUDR_500K:
        {
            pst_canReg->u32_reg6 |= 0x7; 
            break;
        }
        case CAN_BAUDR_1M:
        {
            pst_canReg->u32_reg6 |= 0x3;
            break;
        }
        default:
        {
            pst_canReg->u32_reg6 |= 0x7;
            canerr("baud rate error,set default baud rate 500K.\n");
            break;
        }
    }

    pst_canReg->u32_reg8 &= ~(0x0F); 
    pst_canReg->u32_reg8 &= ~(1<<5); 
    pst_canReg->u32_reg9 &= ~(CAN_AMASK_ID10_0); 
    pst_canReg->u32_reg9 |= (0x01 & CAN_AMASK_ID10_0);
    pst_canReg->u32_reg8 |= (1<<5);

    pst_canReg->u32_reg8 |= (1<<16);
    pst_canReg->u32_reg9 &= ~(CAN_AMASK_ID10_0);     
    pst_canReg->u32_reg9 |= (0x01 & CAN_AMASK_ID10_0);
    pst_canReg->u32_reg9 &= ~(1 << 30); 

    pst_canReg->u32_reg3 &= ~(1<<7);

    return OK;
}

/****************************************************************************
 * Name: arcan_filterinit
 *
 * Description:
 *   CAN filter initialization.  CAN filters are not currently used by this
 *   driver.  The CAN filters can be configured in a different way:
 *
 *   1. As a match of specific IDs in a list (IdList mode), or as
 *   2. And ID and a mask (IdMask mode).
 *
 *   Filters can also be configured as:
 *
 *   3. 16- or 32-bit.  The advantage of 16-bit filters is that you get
 *      more filters;  The advantage of 32-bit filters is that you get
 *      finer control of the filtering.
 *
 *   One filter is set up for each CAN.  The filter resources are shared
 *   between the two CAN modules:  CAN1 uses only filter 0 (but reserves
 *   0 through CAN_NFILTERS/2-1); CAN2 uses only filter CAN_NFILTERS/2
 *   (but reserves CAN_NFILTERS/2 through CAN_NFILTERS-1).
 *
 *   32-bit IdMask mode is configured.  However, both the ID and the MASK
 *   are set to zero thus supressing all filtering because anything masked
 *   with zero matches zero.
 *
 * Input Parameter:
 *   priv - A pointer to the private data structure for this CAN block
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int arcan_filterinit(FAR struct ar_can_s *priv)
{
    return OK;
}

/****************************************************************************
 * Name: arcan_addextfilter
 *
 * Description:
 *   Add a filter for extended CAN IDs
 *
 * Input Parameter:
 *   priv - A pointer to the private data structure for this CAN block
 *   arg  - A pointer to a structure describing the filter
 *
 * Returned Value:
 *   A non-negative filter ID is returned on success.
 *   Otherwise -1 (ERROR) is returned with the errno
 *   set to indicate the nature of the error.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_EXTID
static int arcan_addextfilter(FAR struct ar_can_s *priv,
                                 FAR struct canioc_extfilter_s *arg)
{
    return -ENOTTY;
}
#endif

/****************************************************************************
 * Name: arcan_delextfilter
 *
 * Description:
 *   Remove a filter for extended CAN IDs
 *
 * Input Parameter:
 *   priv - A pointer to the private data structure for this CAN block
 *   arg  - The filter index previously returned by the
 *            CANIOC_ADD_EXTFILTER command
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise -1 (ERROR)
 *   returned with the errno variable set to indicate the
 *   of the error.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_EXTID
static int arcan_delextfilter(FAR struct ar_can_s *priv, int arg)
{
    return -ENOTTY;
}
#endif

/****************************************************************************
 * Name: arcan_addstdfilter
 *
 * Description:
 *   Add a filter for standard CAN IDs
 *
 * Input Parameter:
 *   priv - A pointer to the private data structure for this CAN block
 *   arg  - A pointer to a structure describing the filter
 *
 * Returned Value:
 *   A non-negative filter ID is returned on success.
 *   Otherwise -1 (ERROR) is returned with the errno
 *   set to indicate the nature of the error.
 *
 ****************************************************************************/

// static int arcan_addstdfilter(FAR struct ar_can_s *priv,
//                                  FAR struct canioc_stdfilter_s *arg)
// {
//     return -ENOTTY;
// }

/****************************************************************************
 * Name: arcan_delstdfilter
 *
 * Description:
 *   Remove a filter for standard CAN IDs
 *
 * Input Parameter:
 *   priv - A pointer to the private data structure for this CAN block
 *   arg  - The filter index previously returned by the
 *            CANIOC_ADD_STDFILTER command
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise -1 (ERROR)
 *   returned with the errno variable set to indicate the
 *   of the error.
 *
 ****************************************************************************/

// static int arcan_delstdfilter(FAR struct ar_can_s *priv, int arg)
// {
//     return -ENOTTY;
// }

/****************************************************************************
 * Name: arcan_txmb0empty
 *
 * Input Parameter:
 *   tsr_regval - value of CAN transmit status register
 *
 * Returned Value:
 *   Returns true if mailbox 0 is empty and can be used for sending.
 *
 ****************************************************************************/

// static bool arcan_txmb0empty(uint32_t tsr_regval)
// {
//     return OK;
// }

/****************************************************************************
 * Name: arcan_txmb1empty
 *
 * Input Parameter:
 *   tsr_regval - value of CAN transmit status register
 *
 * Returned Value:
 *   Returns true if mailbox 1 is empty and can be used for sending.
 *
 ****************************************************************************/

// static bool arcan_txmb1empty(uint32_t tsr_regval)
// {
//     return OK;
// }

/****************************************************************************
 * Name: ar2can_txmb2empty
 *
 * Input Parameter:
 *   tsr_regval - value of CAN transmit status register
 *
 * Returned Value:
 *   Returns true if mailbox 2 is empty and can be used for sending.
 *
 ****************************************************************************/

// static bool arcan_txmb2empty(uint32_t tsr_regval)
// {
//     return OK;
// }

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ar_caninitialize
 *
 * Description:
 *   Initialize the selected CAN port
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple CAN interfaces)
 *
 * Returned Value:
 *   Valid CAN device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct can_dev_s *ar_caninitialize(int port)
{
  FAR struct can_dev_s *dev = NULL;

  caninfo("CAN%d\n", port);

  /* NOTE:  Peripherical clocking for CAN1 and/or CAN2 was already provided
   * by stm32_clockconfig() early in the reset sequence.
   */

#ifdef CONFIG_AR_CAN0
  if (port == 0)
  {
    /* Select the CAN1 device structure */

    dev = &g_can0dev;

    /* Configure CAN1 pins.  The ambiguous settings in the stm32*_pinmap.h
      * file must have been disambiguated in the board.h file.
      */

    // ar_configgpio(GPIO_CAN1_RX);
    // ar_configgpio(GPIO_CAN1_TX);
  }
  else
#endif
#ifdef CONFIG_AR_CAN1
  if (port == 1)
    {
      /* Select the CAN2 device structure */

      dev = &g_can1dev;

      /* Configure CAN2 pins.  The ambiguous settings in the stm32*_pinmap.h
       * file must have been disambiguated in the board.h file.
       */

      // ar_configgpio(GPIO_CAN2_RX);
      // ar_configgpio(GPIO_CAN2_TX);
    }
#endif
    {
      canerr("ERROR: Unsupported port %d\n", port);
      return NULL;
    }

  return dev;
}


