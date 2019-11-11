/****************************************************************************
 * arch/arm/src/artosyn/ar_serial.c
 *
 *   Copyright (C) 2015-2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *              liuwei <wei.liu@cecooleye.cn>
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
#include <stddef.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/config.h>

#include <arch/irq.h>
#include <arch/serial.h>

#include "chip/ar_define.h"
#include "chip/ar_config.h"
#include "chip/ar_memorymap.h"
#include "up_arch.h"
#include "ar_uart.h"
#include "ar_rcc.h"


//#include <nuttx/serial/serial.h>

#define ZEROPAD	1		/* pad with zero */
#define SIGN	2		/* unsigned/signed long */
#define PLUS	4		/* show plus */
#define SPACE	8		/* space if plus */
#define LEFT	16		/* left justified */
#define SMALL	32		/* Must be 32 == 0x20 */
#define SPECIAL	64		/* 0x */

#define AR_UART_PRINTF_BUF_LEN  1024

#define __do_div(n, base) ({ \
int __res; \
__res = ((unsigned long) n) % (unsigned) base; \
n = ((unsigned long) n) / (unsigned) base; \
__res; })

#define CP_SIZE                  (1024*64)

#define BUFFER_LENGTH 1024
uint32_t tmpBuffer[BUFFER_LENGTH] = {0};
uint32_t currentIndex = 0;



struct up_dev_s
{
  uint32_t uartbase;  /* Base address of UART registers */
  uint32_t baud;      /* Configured baud */
  uint32_t ier;       /* Saved IER value */
  uint8_t  irq;       /* IRQ associated with this UART */
  uint8_t  parity;    /* 0=none, 1=odd, 2=even */
  uint8_t  bits;      /* Number of bits (7 or 8) */
  bool     stopbits2; /* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

static char g_uart0rxbuffer[ARCFG_UART0_RXBUFF];
static char g_uart0txbuffer[ARCFG_UART0_TXBUFF];

static char g_uart1rxbuffer[ARCFG_UART1_RXBUFF];
static char g_uart1txbuffer[ARCFG_UART1_TXBUFF];

static char g_uart2rxbuffer[ARCFG_UART2_RXBUFF];
static char g_uart2txbuffer[ARCFG_UART2_TXBUFF];

static char g_uart3rxbuffer[ARCFG_UART3_RXBUFF];
static char g_uart3txbuffer[ARCFG_UART3_TXBUFF];

static char g_uart4rxbuffer[ARCFG_UART4_RXBUFF];
static char g_uart4txbuffer[ARCFG_UART4_TXBUFF];

static char g_uart5rxbuffer[ARCFG_UART5_RXBUFF];
static char g_uart5txbuffer[ARCFG_UART5_TXBUFF];

static char g_uart6rxbuffer[ARCFG_UART6_RXBUFF];
static char g_uart6txbuffer[ARCFG_UART6_TXBUFF];


static struct up_dev_s g_uart0priv =
{
  .uartbase       = AR_UART0_BASE,
  .baud           = ARCFG_UART0_BAUD,
  .irq            = AR_IRQ_UART0,
  .parity         = ARCFG_UART0_PARITY,
  .bits           = ARCFG_UART0_BITS,
  .stopbits2      = ARCFG_UART0_STOPBITS,
};


static struct up_dev_s g_uart1priv =
{
  .uartbase       = AR_UART1_BASE,
  .baud           = ARCFG_UART1_BAUD,
  .irq            = AR_IRQ_UART1,
  .parity         = ARCFG_UART1_PARITY,
  .bits           = ARCFG_UART1_BITS,
  .stopbits2      = ARCFG_UART1_STOPBITS,
};

static struct up_dev_s g_uart2priv =
{
  .uartbase       = AR_UART2_BASE,
  .baud           = ARCFG_UART2_BAUD,
  .irq            = AR_IRQ_UART2,
  .parity         = ARCFG_UART2_PARITY,
  .bits           = ARCFG_UART2_BITS,
  .stopbits2      = ARCFG_UART2_STOPBITS,
};

static struct up_dev_s g_uart3priv =
{
  .uartbase       = AR_UART3_BASE,
  .baud           = ARCFG_UART3_BAUD,
  .irq            = AR_IRQ_UART3,
  .parity         = ARCFG_UART3_PARITY,
  .bits           = ARCFG_UART3_BITS,
  .stopbits2      = ARCFG_UART3_STOPBITS,
};

static struct up_dev_s g_uart4priv =
{
  .uartbase       = AR_UART4_BASE,
  .baud           = ARCFG_UART4_BAUD,
  .irq            = AR_IRQ_UART4,
  .parity         = ARCFG_UART4_PARITY,
  .bits           = ARCFG_UART4_BITS,
  .stopbits2      = ARCFG_UART4_STOPBITS,
};

static struct up_dev_s g_uart5priv =
{
  .uartbase       = AR_UART5_BASE,
  .baud           = ARCFG_UART5_BAUD,
  .irq            = AR_IRQ_UART5,
  .parity         = ARCFG_UART5_PARITY,
  .bits           = ARCFG_UART5_BITS,
  .stopbits2      = ARCFG_UART5_STOPBITS,
};

static struct up_dev_s g_uart6priv =
{
  .uartbase       = AR_UART6_BASE,
  .baud           = ARCFG_UART6_BAUD,
  .irq            = AR_IRQ_UART6,
  .parity         = ARCFG_UART6_PARITY,
  .bits           = ARCFG_UART6_BITS,
  .stopbits2      = ARCFG_UART6_STOPBITS,
};


static uart_dev_t g_uart0port =
{
  .recv     =
  {
    .size   = ARCFG_UART0_RXBUFF,
    .buffer = g_uart0rxbuffer,
  },
  .xmit     =
  {
    .size   = ARCFG_UART0_TXBUFF,
    .buffer = g_uart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart0priv,
};

static uart_dev_t g_uart1port =
{
  .recv     =
  {
    .size   = ARCFG_UART1_RXBUFF,
    .buffer = g_uart1rxbuffer,
  },
  .xmit     =
  {
    .size   = ARCFG_UART1_TXBUFF,
    .buffer = g_uart1txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart1priv,
};

static uart_dev_t g_uart2port =
{
  .recv     =
  {
    .size   = ARCFG_UART2_RXBUFF,
    .buffer = g_uart2rxbuffer,
  },
  .xmit     =
  {
    .size   = ARCFG_UART2_TXBUFF,
    .buffer = g_uart2txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart2priv,
};

static uart_dev_t g_uart3port =
{
  .recv     =
  {
    .size   = ARCFG_UART3_RXBUFF,
    .buffer = g_uart3rxbuffer,
  },
  .xmit     =
  {
    .size   = ARCFG_UART3_TXBUFF,
    .buffer = g_uart3txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart3priv,
};

static uart_dev_t g_uart4port =
{
  .recv     =
  {
    .size   = ARCFG_UART4_RXBUFF,
    .buffer = g_uart4rxbuffer,
  },
  .xmit     =
  {
    .size   = ARCFG_UART4_TXBUFF,
    .buffer = g_uart4txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart4priv,
};

static uart_dev_t g_uart5port =
{
  .recv     =
  {
    .size   = ARCFG_UART5_RXBUFF,
    .buffer = g_uart5rxbuffer,
  },
  .xmit     =
  {
    .size   = ARCFG_UART5_TXBUFF,
    .buffer = g_uart5txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart5priv,
};

static uart_dev_t g_uart6port =
{
  .recv     =
  {
    .size   = ARCFG_UART6_RXBUFF,
    .buffer = g_uart6rxbuffer,
  },
  .xmit     =
  {
    .size   = ARCFG_UART6_TXBUFF,
    .buffer = g_uart6txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart6priv,
};

// static uart_dev_t g_uart7port =
// {
//   .recv     =
//   {
//     .size   = ARCFG_UART7_RXBUFF,
//     .buffer = g_uart7rxbuffer,
//   },
//   .xmit     =
//   {
//     .size   = ARCFG_UART7_TXBUFF,
//     .buffer = g_uart7txbuffer,
//   },
//   .ops      = &g_uart_ops,
//   .priv     = &g_uart7priv,
// };

// static uart_dev_t g_uart8port =
// {
//   .recv     =
//   {
//     .size   = ARCFG_UART8_RXBUFF,
//     .buffer = g_uart8rxbuffer,
//   },
//   .xmit     =
//   {
//     .size   = ARCFG_UART8_TXBUFF,
//     .buffer = g_uart8txbuffer,
//   },
//   .ops      = &g_uart_ops,
//   .priv     = &g_uart8priv,
// };



// read
static inline uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
    return getreg32(priv->uartbase + offset);
}

// write
static inline void up_serialout(struct up_dev_s *priv, int offset, uint32_t value)
{
    putreg32(value, priv->uartbase + offset);
}

// disable the all int
static inline void up_disableuartint(struct up_dev_s *priv, uint32_t *ier)
{
    if (ier)
    {
        *ier = priv->ier & UART_IER_ALLIE;
    }

    priv->ier &= ~UART_IER_ALLIE;
    up_serialout(priv, AR_UART_IER_OFFSET, priv->ier);
}


// restore the uart int
static inline void up_restoreuartint(struct up_dev_s *priv, uint32_t ier)
{
  priv->ier |= ier & UART_IER_ALLIE;
  up_serialout(priv, AR_UART_IER_OFFSET, priv->ier);
}


// enable  LCR  breaks  BIT 6
static inline void up_enablebreaks(struct up_dev_s *priv, bool enable)
{
  uint32_t lcr = up_serialin(priv, AR_UART_LCR_OFFSET);

  if (enable)
    {
      lcr |= UART_LCR_BRK;
    }
  else
    {
      lcr &= ~UART_LCR_BRK;
    }

  up_serialout(priv, AR_UART_LCR_OFFSET, lcr);
}

static inline void ar_uartconfig(void)
{
    //uint32_t   regval;
    irqstate_t flags;

    flags   = enter_critical_section();

    /* Step 1: Enable power on UART0 */
    /* Step 2: Enable clocking on UART */
    /* Step 3: Configure I/O pins */
    leave_critical_section(flags);
};



/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is
 *   called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
    uint16_t dl;
    uint32_t lcr;
    up_serialout(priv, AR_UART_FCR_OFFSET, (UART_FCR_RXRST | UART_FCR_TXRST));

    if(priv->uartbase == AR_UART0_BASE)
    {
        up_serialout(priv, AR_UART_FCR_OFFSET, (UART_FCR_FIFOEN | UART_FCR_RXTRIGGER_0));
    }

    priv->ier = up_serialin(priv, AR_UART_IER_OFFSET);

    lcr = 0;

    if (priv->bits == 7)
    {
        lcr |= UART_LCR_DLS_7BIT;
    }
    else
    {
        lcr |= UART_LCR_DLS_8BIT;
    }

    if (priv->stopbits2)
    {
        lcr |= UART_LCR_STOP;
    }

    if (priv->parity == 1)
    {
        lcr |= (UART_LCR_PE | UART_LCR_PS_ODD);
    }
    else if (priv->parity == 2)
    {
        lcr |= (UART_LCR_PE | UART_LCR_PS_EVEN);
    }

    /* Enter DLAB=1 */
    up_serialout(priv, AR_UART_LCR_OFFSET, (lcr | UART_LCR_DLAB));

    /* Set the BAUD divisor */
    dl = (CPU0_CPU1_CORE_PLL_CLK/2) / (16 * priv->baud);

    up_serialout(priv, AR_UART_DLH_OFFSET, dl >> 8);
    up_serialout(priv, AR_UART_DLL_OFFSET, dl & 0xff);

    up_serialout(priv, AR_UART_LCR_OFFSET, lcr);

    up_serialout(priv, AR_UART_FCR_OFFSET,
            (UART_FCR_RXTRIGGER_8 | UART_FCR_TXRST | UART_FCR_RXRST |
                UART_FCR_FIFOEN));
    return OK;
}



/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	up_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
    int ret;

    /* Attach and enable the IRQ */

    ret = irq_attach(priv->irq, up_interrupt, dev);
    if (ret == OK)
    {
        /* Enable the interrupt (RX and TX interrupts are still disabled
        * in the UART
        */
        up_enable_irq(priv->irq);
    }

    return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
    up_disable_irq(priv->irq);
    irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/
static int up_interrupt(int irq, void *context, void *arg)
{
    struct uart_dev_s *dev = (struct uart_dev_s *)arg;
    struct up_dev_s   *priv;
    uint32_t           status;
    int                passes;

    DEBUGASSERT(dev != NULL && dev->priv != NULL);
    priv = (struct up_dev_s *)dev->priv;

    /* Loop until there are no characters to be transferred or,
    * until we have been looping for a long time.
    */

    for (passes = 0; passes < 256; passes++)
    {
        /* Get the current UART status and check for loop
        * termination conditions
        */

        status = up_serialin(priv, AR_UART_IIR_OFFSET);

        /* The UART_IIR_INTSTATUS bit should be zero if there are pending
        * interrupts
        */

        if ((status & UART_IIR_INTID_MASK) == UART_IIR_INTID_NIP)  //
        {
            /* Break out of the loop when there is no longer a
            * pending interrupt
            */

            break;
        }

        /* Handle the interrupt by its interrupt ID field */

        switch (status & UART_IIR_INTID_MASK)
        {
            /* Handle incoming, receive bytes (with or without timeout) */
            case UART_IIR_INTID_CTI:
            case UART_IIR_INTID_RDA:
            {
                uart_recvchars(dev);
                break;
            }

            /* Handle outgoing, transmit bytes */

            case UART_IIR_INTID_THRE:
            {
                uart_xmitchars(dev);
                break;
            }

            /* Just clear modem status interrupts */

            case UART_IIR_INTID_MSI:
            {
                /* Read the modem status register (MSR) to clear */

                status = up_serialin(priv, AR_UART_MSR_OFFSET);
                // _info("MSR: %02x\n", status);
                break;
            }

            /* Just clear any line status interrupts */

            case UART_IIR_INTID_RLS:
            {
                /* Read the line status register (LSR) to clear */

                status = up_serialin(priv, AR_UART_LSR_OFFSET);
                // _info("LSR: %02x\n", status);
                break;
            }
            
            case UART_IIR_INTID_BUSY:
            {
                status = up_serialin(priv, AR_UART_USR_OFFSET);
                // _err("ERROR: UART_IIR_INTID_BUSY: %02x\n", status);
                break;
            }

            /* There should be no other values */

            default:
            {
                // _err("ERROR: Unexpected IIR: %02x\n", status);
                break;
            }
        }
    }
    return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	struct inode      *inode = filep->f_inode;
	struct uart_dev_s *dev   = inode->i_private;
	struct up_dev_s   *priv  = (struct up_dev_s *)dev->priv;
	int                ret    = OK;

	switch (cmd)
	{

		case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
		{
			irqstate_t flags = enter_critical_section();
			up_enablebreaks(priv, true);
			leave_critical_section(flags);
		}
		break;

		case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
		{
			irqstate_t flags;
			flags = enter_critical_section();
			up_enablebreaks(priv, false);
			leave_critical_section(flags);
		}
		break;

		case TCGETS:
		{
			struct termios *termiosp = (struct termios *)arg;

			if (!termiosp)
			{
				ret = -EINVAL;
				break;
			}

			/* TODO:  Other termios fields are not yet returned.
				* Note that cfsetospeed is not necessary because we have
				* knowledge that only one speed is supported.
				* Both cfset(i|o)speed() translate to cfsetspeed.
				*/

			cfsetispeed(termiosp, priv->baud);
		}
		break;

		case TCSETS:
		{
			struct termios *termiosp = (struct termios *)arg;
			uint32_t           lcr;  /* Holds current values of line control register */
			uint16_t           dl;   /* Divisor latch */

			if (!termiosp)
			{
				ret = -EINVAL;
				break;
			}

			/* TODO:  Handle other termios settings.
				* Note that only cfgetispeed is used because we have knowledge
				* that only one speed is supported.
				*/

			/* Get the c_speed field in the termios struct */

			priv->baud = cfgetispeed(termiosp);


			lcr = getreg32(priv->uartbase + AR_UART_LCR_OFFSET);
			up_serialout(priv, AR_UART_LCR_OFFSET, (lcr | UART_LCR_DLAB));

			/* Set the BAUD divisor */

			dl = (CPU0_CPU1_CORE_PLL_CLK/2) / (16 * priv->baud);

      // _err("dl = %08x", dl);

			up_serialout(priv, AR_UART_DLH_OFFSET, dl >> 8);
			up_serialout(priv, AR_UART_DLL_OFFSET, dl & 0xff);

      
      // dl = up_serialin(priv, AR_UART_DLH_OFFSET) << 8 | up_serialin(priv, AR_UART_DLL_OFFSET);

      // _err("dl1 = %08x", dl);


			up_serialout(priv, AR_UART_LCR_OFFSET, lcr);

      up_serialout(priv, AR_UART_FCR_OFFSET,
                  (UART_FCR_RXTRIGGER_8 | UART_FCR_TXRST | UART_FCR_RXRST |
                  UART_FCR_FIFOEN));
		}
		break;

		default:
			ret = -ENOTTY;
			break;
	}

	return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	uint32_t rbr;

	*status = up_serialin(priv, AR_UART_LSR_OFFSET);
	rbr     = up_serialin(priv, AR_UART_RBR_OFFSET);

	return rbr;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	if (enable)
	{
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
		priv->ier |= UART_IER_ERBFI;
#endif
	}
	else
	{
		priv->ier &= ~UART_IER_ERBFI;
	}

	up_serialout(priv, AR_UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  	return ((up_serialin(priv, AR_UART_LSR_OFFSET) & UART_LSR_RDR) != 0);
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  	up_serialout(priv, AR_UART_THR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
		priv->ier |= UART_IER_ETBEI;
		up_serialout(priv, AR_UART_IER_OFFSET, priv->ier);

		/* Fake a TX interrupt here by just calling uart_xmitchars() with
		* interrupts disabled (note this may recurse).
		*/

		uart_xmitchars(dev);
#endif
    }
  else
    {
		priv->ier &= ~UART_IER_ETBEI;
		up_serialout(priv, AR_UART_IER_OFFSET, priv->ier);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, AR_UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, AR_UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 *   NOTE: Configuration of the CONSOLE UART was performed by up_lowsetup()
 *   very early in the boot sequence.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* Configure all UARTs (except the CONSOLE UART) and disable interrupts */

  ar_uartconfig();

  /* Configuration whichever one is the console */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif

}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{

#ifdef CONSOLE_DEV
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif

#ifdef TTYS0_DEV
    (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif

#ifdef TTYS1_DEV
    (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif

#ifdef TTYS2_DEV
    (void)uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif

#ifdef TTYS3_DEV
    (void)uart_register("/dev/ttyS3", &TTYS3_DEV);
#endif

#ifdef TTYS4_DEV
    (void)uart_register("/dev/ttyS4", &TTYS4_DEV);
#endif

#ifdef TTYS5_DEV
    (void)uart_register("/dev/ttyS5", &TTYS5_DEV);
#endif

#ifdef TTYS6_DEV
    (void)uart_register("/dev/ttyS6", &TTYS6_DEV);
#endif
}

void up_lowputc(char ch)
{
    volatile uart_type *uart_regs = (uart_type *)AR_UART0_BASE;
    while ((uart_regs->LSR & UART_LSR_THRE) != UART_LSR_THRE);
    uart_regs->RBR_THR_DLL = ch;
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_CONSOLE
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  uint32_t ier;
  up_disableuartint(priv, &ier);
#endif

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
#ifdef HAVE_CONSOLE
  up_restoreuartint(priv, ier);
#endif

  return ch;
}

