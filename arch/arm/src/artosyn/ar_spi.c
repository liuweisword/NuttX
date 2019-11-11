
/************************************************************************************
 * arm/arm/src/artosyn/ar_spi.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "cache.h"
#include "chip.h"
#include "chip/ar_define.h"
#include "chip/ar_config.h"
#include "ar_gpio.h"
#include "ar_spi.h"
#include "ar_uart.h"

#if defined(CONFIG_AR_SPI0) || defined(CONFIG_AR_SPI1) || \
    defined(CONFIG_AR_SPI2) || defined(CONFIG_AR_SPI3) || \
    defined(CONFIG_AR_SPI4) || defined(CONFIG_AR_SPI5) || \
    defined(CONFIG_AR_SPI6)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/
/* SPI interrupts */

#ifdef CONFIG_AR_SPI_INTERRUPTS
#   error "Interrupt driven SPI not yet supported"
#endif

/* Can't have both interrupt driven SPI and SPI DMA */

#if defined(CONFIG_AR_SPI_INTERRUPTS) && defined(CONFIG_AR_SPI_DMA)
#   error "Cannot enable both interrupt mode and DMA mode for SPI"
#endif

/* SPI DMA priority */

#ifdef CONFIG_AR_SPI_DMA

// #  if defined(CONFIG_SPI_DMAPRIO)
// #    define SPI_DMA_PRIO  CONFIG_SPI_DMAPRIO
// #  elif defined(DMA_SCR_PRIMED)
// #    define SPI_DMA_PRIO  DMA_SCR_PRILO
// #  else
// #    error "Unknown AR DMA"
// #  endif

// #if (SPI_DMA_PRIO & ~DMA_SCR_PL_MASK) != 0
// #    error "Illegal value for CONFIG_SPI_DMAPRIO"
// #endif


/* DMA channel configuration */

#  define SPI_RXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_SCR_MSIZE_16BITS|DMA_SCR_PSIZE_16BITS|DMA_SCR_MINC|DMA_SCR_DIR_P2M)
#  define SPI_RXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS |DMA_SCR_MINC|DMA_SCR_DIR_P2M)
#  define SPI_RXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_16BITS             |DMA_SCR_DIR_P2M)
#  define SPI_RXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS              |DMA_SCR_DIR_P2M)
#  define SPI_TXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_SCR_MSIZE_16BITS|DMA_SCR_PSIZE_16BITS|DMA_SCR_MINC|DMA_SCR_DIR_M2P)
#  define SPI_TXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS |DMA_SCR_MINC|DMA_SCR_DIR_M2P)
#  define SPI_TXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_16BITS             |DMA_SCR_DIR_M2P)
#  define SPI_TXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS              |DMA_SCR_DIR_M2P)
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct ar_spidev_s
{
    struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
    uint32_t         spibase;    /* SPIn base address */
    uint32_t         spiclock;   /* Clocking for the SPI module */
#ifdef CONFIG_AR_SPI_INTERRUPTS
    uint8_t          spiirq;     /* SPI IRQ number */
#endif
#ifdef CONFIG_AR_SPI_DMA
    volatile uint8_t rxresult;   /* Result of the RX DMA */
    volatile uint8_t txresult;   /* Result of the RX DMA */
    uint8_t          rxch;       /* The RX DMA channel number */
    uint8_t          txch;       /* The TX DMA channel number */
    DMA_HANDLE       rxdma;      /* DMA channel handle for RX transfers */
    DMA_HANDLE       txdma;      /* DMA channel handle for TX transfers */
    sem_t            rxsem;      /* Wait for RX DMA to complete */
    sem_t            txsem;      /* Wait for TX DMA to complete */
    uint32_t         txccr;      /* DMA control register for TX transfers */
    uint32_t         rxccr;      /* DMA control register for RX transfers */
#endif
    sem_t            exclsem;    /* Held while chip is selected for mutual exclusion */
    uint32_t         frequency;  /* Requested clock frequency */
    uint32_t         actual;     /* Actual clock frequency */
    int8_t           nbits;      /* Width of word in bits */
    uint8_t          mode;       /* Mode 0,1,2,3 */

    const uint8_t   *txbuffer;
    uint8_t         *rxbuffer;
    size_t          txnwords;
    size_t          rxnwords;
  
};


/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Helpers */

static inline uint32_t spi_getreg32(FAR struct ar_spidev_s *priv, uint32_t offset);
static inline void spi_putreg32(FAR struct ar_spidev_s *priv, uint32_t offset,
                                 uint32_t value);
static inline uint16_t spi_readword(FAR struct ar_spidev_s *priv);
static inline void spi_writeword(FAR struct ar_spidev_s *priv, uint16_t byte);
static inline bool spi_9to16bitmode(FAR struct ar_spidev_s *priv);

/* DMA support */

#ifdef CONFIG_AR_SPI_DMA
static void        spi_dmarxwait(FAR struct ar_spidev_s *priv);
static void        spi_dmatxwait(FAR struct ar_spidev_s *priv);
static inline void spi_dmarxwakeup(FAR struct ar_spidev_s *priv);
static inline void spi_dmatxwakeup(FAR struct ar_spidev_s *priv);
static void        spi_dmarxcallback(DMA_HANDLE handle, uint8_t isr, void *arg);
static void        spi_dmatxcallback(DMA_HANDLE handle, uint8_t isr, void *arg);
static void        spi_dmarxsetup(FAR struct ar_spidev_s *priv,
                                  FAR void *rxbuffer, FAR void *rxdummy, size_t nwords);
static void        spi_dmatxsetup(FAR struct ar_spidev_s *priv,
                                  FAR const void *txbuffer, FAR const void *txdummy, size_t nwords);
static inline void spi_dmarxstart(FAR struct ar_spidev_s *priv);
static inline void spi_dmatxstart(FAR struct ar_spidev_s *priv);
#endif

/* SPI methods */

static int         spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t    spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void        spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void        spi_setbits(FAR struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int         spi_hwfeatures(FAR struct spi_dev_s *dev,
                                  spi_hwfeatures_t features);
#endif
static uint16_t    spi_send(FAR struct spi_dev_s *dev, uint16_t wd);
static void        spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                                FAR void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void        spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                                size_t nwords);
static void        spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
                                 size_t nwords);
#endif

/* Initialization */

static void        spi_bus_initialize(FAR struct ar_spidev_s *priv);

/************************************************************************************
 * Private Data
 ************************************************************************************/
#ifdef CONFIG_AR_SPI0
static const struct spi_ops_s g_sp0iops =
{
    .lock              = spi_lock,
    .select            = ar_spi0select,
    .setfrequency      = spi_setfrequency,
    .setmode           = spi_setmode,
    .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
    .hwfeatures        = spi_hwfeatures,
#endif
    .status            = ar_spi0status,
#ifdef CONFIG_SPI_CMDDATA
    .cmddata           = AR_spi0cmddata,
#endif
    .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
    .exchange          = spi_exchange,
#else
    .sndblock          = spi_sndblock,
    .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
    .registercallback  = ar_spi0register,   /* Provided externally */
#else
    .registercallback  = 0,                 /* Not implemented */
#endif
};

static struct ar_spidev_s g_spi0dev =
{
    .spidev   = { &g_sp0iops },
    .spibase  = AR_SPI0_BASE,
    .spiclock = AR_BUS_CLK,
#ifdef CONFIG_AR_SPI_INTERRUPTS
    .spiirq   = AR_IRQ_SPI0,
#endif
#ifdef CONFIG_AR_SPI_DMA
    .rxch     = DMAMAP_SPI0_RX,
    .txch     = DMAMAP_SPI0_TX,
#endif
};
#endif


#ifdef CONFIG_AR_SPI1
static const struct spi_ops_s g_sp1iops =
{
    .lock              = spi_lock,
    .select            = ar_spi1select,
    .setfrequency      = spi_setfrequency,
    .setmode           = spi_setmode,
    .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
    .hwfeatures        = spi_hwfeatures,
#endif
    .status            = ar_spi1status,
#ifdef CONFIG_SPI_CMDDATA
    .cmddata           = AR_spi1cmddata,
#endif
    .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
    .exchange          = spi_exchange,
#else
    .sndblock          = spi_sndblock,
    .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
    .registercallback  = ar_spi1register,   /* Provided externally */
#else
    .registercallback  = 0,                 /* Not implemented */
#endif
};

static struct ar_spidev_s g_spi1dev =
{
    .spidev   = { &g_sp1iops },
    .spibase  = AR_SPI1_BASE,
    .spiclock = AR_BUS_CLK,
#ifdef CONFIG_AR_SPI_INTERRUPTS
    .spiirq   = AR_IRQ_SPI1,
#endif
#ifdef CONFIG_AR_SPI_DMA
    .rxch     = DMAMAP_SPI1_RX,
    .txch     = DMAMAP_SPI1_TX,
#endif
};
#endif

#ifdef CONFIG_AR_SPI2
static const struct spi_ops_s g_sp2iops =
{
    .lock              = spi_lock,
    .select            = ar_spi2select,
    .setfrequency      = spi_setfrequency,
    .setmode           = spi_setmode,
    .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
    .hwfeatures        = spi_hwfeatures,
#endif
    .status            = ar_spi2status,
#ifdef CONFIG_SPI_CMDDATA
    .cmddata           = AR_spi2cmddata,
#endif
    .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
    .exchange          = spi_exchange,
#else
    .sndblock          = spi_sndblock,
    .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
    .registercallback  = ar_spi2register,   /* Provided externally */
#else
    .registercallback  = 0,                 /* Not implemented */
#endif
};

static struct ar_spidev_s g_spi2dev =
{
    .spidev   = { &g_sp2iops },
    .spibase  = AR_SPI1_BASE,
    .spiclock = AR_BUS_CLK,
#ifdef CONFIG_AR_SPI_INTERRUPTS
    .spiirq   = AR_IRQ_SPI2,
#endif
#ifdef CONFIG_AR_SPI_DMA
    .rxch     = DMAMAP_SPI2_RX,
    .txch     = DMAMAP_SPI2_TX,
#endif
};
#endif

#ifdef CONFIG_AR_SPI3
static const struct spi_ops_s g_sp3iops =
{
    .lock              = spi_lock,
    .select            = ar_spi3select,
    .setfrequency      = spi_setfrequency,
    .setmode           = spi_setmode,
    .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
    .hwfeatures        = spi_hwfeatures,
#endif
    .status            = ar_spi3status,
#ifdef CONFIG_SPI_CMDDATA
    .cmddata           = AR_spi3cmddata,
#endif
    .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
    .exchange          = spi_exchange,
#else
    .sndblock          = spi_sndblock,
    .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
    .registercallback  = ar_spi3register,   /* Provided externally */
#else
    .registercallback  = 0,                 /* Not implemented */
#endif
};

static struct ar_spidev_s g_spi3dev =
{
    .spidev   = { &g_sp3iops },
    .spibase  = AR_SPI3_BASE,
    .spiclock = AR_BUS_CLK,
#ifdef CONFIG_AR_SPI_INTERRUPTS
    .spiirq   = AR_IRQ_SPI3,
#endif
#ifdef CONFIG_AR_SPI_DMA
    .rxch     = DMAMAP_SPI3_RX,
    .txch     = DMAMAP_SPI3_TX,
#endif
};
#endif

#ifdef CONFIG_AR_SPI4
static const struct spi_ops_s g_sp4iops =
{
    .lock              = spi_lock,
    .select            = ar_spi4select,
    .setfrequency      = spi_setfrequency,
    .setmode           = spi_setmode,
    .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
    .hwfeatures        = spi_hwfeatures,
#endif
    .status            = ar_spi4status,
#ifdef CONFIG_SPI_CMDDATA
    .cmddata           = AR_spi4cmddata,
#endif
    .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
    .exchange          = spi_exchange,
#else
    .sndblock          = spi_sndblock,
    .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
    .registercallback  = ar_spi4register,   /* Provided externally */
#else
    .registercallback  = 0,                   /* Not implemented */
#endif
};

static struct ar_spidev_s g_spi4dev =
{
  .spidev   = { &g_sp4iops },
  .spibase  = AR_SPI4_BASE,
  .spiclock = AR_BUS_CLK,
  #ifdef CONFIG_AR_SPI_INTERRUPTS
  .spiirq   = AR_IRQ_SPI4,
#endif
#ifdef CONFIG_AR_SPI_DMA
  .rxch     = DMAMAP_SPI4_RX,
  .txch     = DMAMAP_SPI4_TX,
#endif
};
#endif

#ifdef CONFIG_AR_SPI5
static const struct spi_ops_s g_sp5iops =
{
    .lock              = spi_lock,
    .select            = ar_spi5select,
    .setfrequency      = spi_setfrequency,
    .setmode           = spi_setmode,
    .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
    .hwfeatures        = spi_hwfeatures,
#endif
    .status            = ar_spi5status,
#ifdef CONFIG_SPI_CMDDATA
    .cmddata           = AR_spi5cmddata,
#endif
    .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
    .exchange          = spi_exchange,
#else
    .sndblock          = spi_sndblock,
    .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
    .registercallback  = ar_spi5register,   /* Provided externally */
#else
    .registercallback  = 0,                 /* Not implemented */
#endif
};

static struct ar_spidev_s g_spi5dev =
{
    .spidev   = { &g_sp5iops },
    .spibase  = AR_SPI5_BASE,
    .spiclock = AR_BUS_CLK,
#ifdef CONFIG_AR_SPI_INTERRUPTS
    .spiirq   = AR_IRQ_SPI5,
#endif
#ifdef CONFIG_AR_SPI_DMA
    .rxch     = DMAMAP_SPI5_RX,
    .txch     = DMAMAP_SPI5_TX,
#endif
};
#endif

#ifdef CONFIG_AR_SPI6
static const struct spi_ops_s g_sp6iops =
{
    .lock              = spi_lock,
    .select            = ar_spi6select,
    .setfrequency      = spi_setfrequency,
    .setmode           = spi_setmode,
    .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
    .hwfeatures        = spi_hwfeatures,
#endif
    .status            = ar_spi6status,
#ifdef CONFIG_SPI_CMDDATA
    .cmddata           = AR_spi6cmddata,
#endif
    .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
    .exchange          = spi_exchange,
#else
    .sndblock          = spi_sndblock,
    .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
    .registercallback  = ar_spi6register,   /* Provided externally */
#else
    .registercallback  = 0,                 /* Not implemented */
#endif
};

static struct ar_spidev_s g_spi6dev =
{
    .spidev   = { &g_sp6iops },
    .spibase  = AR_SPI6_BASE,
    .spiclock = AR_BUS_CLK,
#ifdef CONFIG_AR_SPI_INTERRUPTS
    .spiirq   = AR_IRQ_SPI6,
#endif
#ifdef CONFIG_AR_SPI_DMA
    .rxch     = DMAMAP_SPI6_RX,
    .txch     = DMAMAP_SPI6_TX,
#endif
};
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: spi_getreg32
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 8-bit register
 *
 ************************************************************************************/

static inline uint32_t spi_getreg32(FAR struct ar_spidev_s *priv, uint32_t offset)
{
    return getreg32(priv->spibase + offset);
}

/************************************************************************************
 * Name: spi_putreg32
 *
 * Description:
 *   Write a 8-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 8-bit value to be written
 *
 ************************************************************************************/

static inline void spi_putreg32(FAR struct ar_spidev_s *priv, uint32_t offset, uint32_t value)
{
    putreg32(value, priv->spibase + offset);
}

/************************************************************************************
 * Name: spi_readword
 *
 * Description:
 *   Read one byte from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   Byte as read
 *
 ************************************************************************************/

static inline uint16_t spi_readword(FAR struct ar_spidev_s *priv)
{
    /* Wait until the receive buffer is not empty */

    while ((spi_getreg32(priv, SPI_SR) & SPI_SR_RFNE) == 0)
    {
        ;
    }

    /* Then return the received byte */

    return spi_getreg32(priv, SPI_DR);
}

/************************************************************************************
 * Name: spi_writeword
 *
 * Description:
 *   Write one byte to SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   byte - Byte to send
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void spi_writeword(FAR struct ar_spidev_s *priv, uint16_t word)
{
    /* Wait until the transmit buffer is empty */

    while ((spi_getreg32(priv, SPI_SR) & SPI_SR_TFE) == 0)
    {
    }

    /* Then send the byte */

    spi_putreg32(priv, SPI_DR, word);
}

/************************************************************************************
 * Name: spi_readbyte
 *
 * Description:
 *   Read one byte from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   Byte as read
 *
 ************************************************************************************/

static inline uint8_t spi_readbyte(FAR struct ar_spidev_s *priv)
{
    return spi_readword(priv);
}


static inline void ar_spi_modifyreg32(FAR struct ar_spidev_s *priv,
                                         uint8_t offset, uint32_t clearbits,
                                         uint32_t setbits)
{
    modifyreg32(priv->spibase+offset, clearbits, setbits);
}



static inline void ar_spi_disable_all_interrupts(FAR struct ar_spidev_s *priv)
{
    ar_spi_modifyreg32(priv, SPI_IMR, SPI_IMR_MASK, 0);
}


/************************************************************************************
 * Name: spi_writebyte
 *
 * Description:
 *   Write one 8-bit frame to the SPI FIFO
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   byte - Byte to send
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void spi_writebyte(FAR struct ar_spidev_s *priv, uint8_t byte)
{
    spi_writeword(priv, byte);
}

/************************************************************************************
 * Name: spi_9to16bitmode
 *
 * Description:
 *   Check if the SPI is operating in more then 8 bit mode
 *
 * Input Parameters:
 *   priv     - Device-specific state data
 *
 * Returned Value:
 *   true: >8 bit mode-bit mode, false: <= 8-bit mode
 *
 ************************************************************************************/

static inline bool spi_9to16bitmode(FAR struct ar_spidev_s *priv)
{
    return ((spi_getreg32(priv, SPI_CTRLR0) & SPI_CTRLR0_DFS_MASK) > 7);
}

/************************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ************************************************************************************/

#ifdef CONFIG_AR_SPI_DMA
static void spi_dmarxwait(FAR struct ar_spidev_s *priv)
{
    /* Take the semaphore (perhaps waiting).  If the result is zero, then the DMA
    * must not really have completed???
    */

    while (sem_wait(&priv->rxsem) != 0 || priv->rxresult == 0)
    {
        /* The only case that an error should occur here is if the wait was awakened
        * by a signal.
        */

        ASSERT(errno == EINTR);
     }
}
#endif

/************************************************************************************
 * Name: spi_dmatxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ************************************************************************************/

#ifdef CONFIG_AR_SPI_DMA
static void spi_dmatxwait(FAR struct ar_spidev_s *priv)
{
    /* Take the semaphore (perhaps waiting).  If the result is zero, then the DMA
    * must not really have completed???
    */

    while (sem_wait(&priv->txsem) != 0 || priv->txresult == 0)
    {
        /* The only case that an error should occur here is if the wait was awakened
        * by a signal.
        */

        ASSERT(errno == EINTR);
    }
}
#endif

/************************************************************************************
 * Name: spi_dmarxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ************************************************************************************/

#ifdef CONFIG_AR_SPI_DMA
static inline void spi_dmarxwakeup(FAR struct ar_spidev_s *priv)
{
    (void)sem_post(&priv->rxsem);
}
#endif

/************************************************************************************
 * Name: spi_dmatxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ************************************************************************************/

#ifdef CONFIG_AR_SPI_DMA
static inline void spi_dmatxwakeup(FAR struct ar_spidev_s *priv)
{
    (void)sem_post(&priv->txsem);
}
#endif

/************************************************************************************
 * Name: spi_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ************************************************************************************/

#ifdef CONFIG_AR_SPI_DMA
static void spi_dmarxcallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
    FAR struct ar_spidev_s *priv = (FAR struct ar_spidev_s *)arg;

    /* Wake-up the SPI driver */

    priv->rxresult = isr | 0x080;  /* OR'ed with 0x80 to assure non-zero */
    spi_dmarxwakeup(priv);
}
#endif

/************************************************************************************
 * Name: spi_dmatxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ************************************************************************************/

#ifdef CONFIG_AR_SPI_DMA
static void spi_dmatxcallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
    FAR struct ar_spidev_s *priv = (FAR struct ar_spidev_s *)arg;

    /* Wake-up the SPI driver */

    priv->txresult = isr | 0x080;  /* OR'ed with 0x80 to assure non-zero */
    spi_dmatxwakeup(priv);
}
#endif

/************************************************************************************
 * Name: spi_dmarxsetup
 *
 * Description:
 *   Setup to perform RX DMA
 *
 ************************************************************************************/

#ifdef CONFIG_AR_SPI_DMA
static void spi_dmarxsetup(FAR struct ar_spidev_s *priv, FAR void *rxbuffer,
                           FAR void *rxdummy, size_t nwords)
{
    /* 8- or 16-bit mode? */

    if (spi_9to16bitmode(priv))
    {
        /* 16-bit mode -- is there a buffer to receive data in? */

        if (rxbuffer)
        {
            priv->rxccr = SPI_RXDMA16_CONFIG;
        }
        else
        {
            rxbuffer    = rxdummy;
            priv->rxccr = SPI_RXDMA16NULL_CONFIG;
        }
    }
    else
    {
        /* 8-bit mode -- is there a buffer to receive data in? */

        if (rxbuffer)
        {
            priv->rxccr = SPI_RXDMA8_CONFIG;
        }
        else
        {
            rxbuffer    = rxdummy;
            priv->rxccr = SPI_RXDMA8NULL_CONFIG;
        }
    }

    /* Configure the RX DMA */

    ar_dmasetup(priv->rxdma, priv->spibase + AR_SPI_DR_OFFSET,
                    (uint32_t)rxbuffer, nwords, priv->rxccr);
}
#endif

/************************************************************************************
 * Name: spi_dmatxsetup
 *
 * Description:
 *   Setup to perform TX DMA
 *
 ************************************************************************************/

#ifdef CONFIG_AR_SPI_DMA
static void spi_dmatxsetup(FAR struct ar_spidev_s *priv, FAR const void *txbuffer,
                           FAR const void *txdummy, size_t nwords)
{
    /* 8- or 16-bit mode? */

    if (spi_9to16bitmode(priv))
    {
        /* 16-bit mode -- is there a buffer to transfer data from? */

        if (txbuffer)
        {
            priv->txccr = SPI_TXDMA16_CONFIG;
        }
        else
        {
            txbuffer    = txdummy;
            priv->txccr = SPI_TXDMA16NULL_CONFIG;
        }
    }
    else
    {
        /* 8-bit mode -- is there a buffer to transfer data from? */

        if (txbuffer)
        {
            priv->txccr = SPI_TXDMA8_CONFIG;
        }
        else
        {
            txbuffer    = txdummy;
            priv->txccr = SPI_TXDMA8NULL_CONFIG;
        }
    }

    /* Setup the TX DMA */

    ar_dmasetup(priv->txdma, priv->spibase + AR_SPI_DR_OFFSET,
                    (uint32_t)txbuffer, nwords, priv->txccr);
}
#endif

/************************************************************************************
 * Name: spi_dmarxstart
 *
 * Description:
 *   Start RX DMA
 *
 ************************************************************************************/

#ifdef CONFIG_AR_SPI_DMA
static void spi_dmarxstart(FAR struct ar_spidev_s *priv)
{
    priv->rxresult = 0;
    ar_dmastart(priv->rxdma, spi_dmarxcallback, priv, false);
}
#endif

/************************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start TX DMA
 *
 ************************************************************************************/

#ifdef CONFIG_AR_SPI_DMA
static void spi_dmatxstart(FAR struct ar_spidev_s *priv)
{
    priv->txresult = 0;
    ar_dmastart(priv->txdma, spi_dmatxcallback, priv, false);
}
#endif

/************************************************************************************
 * Name: spi_modifyctrlr0
 *
 * Description:
 *   Clear and set bits in the CR1 register
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_modifyctrlr0(FAR struct ar_spidev_s *priv, uint32_t setbits,
                          uint32_t clrbits)
{
    uint16_t cr1;
    cr1 = spi_getreg32(priv, SPI_CTRLR0);
    cr1 &= ~clrbits;
    cr1 |= setbits;
    spi_putreg32(priv, SPI_CTRLR0, cr1);
}

/************************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
    FAR struct ar_spidev_s *priv = (FAR struct ar_spidev_s *)dev;

    if (lock)
    {
        /* Take the semaphore (perhaps waiting) */

        while (sem_wait(&priv->exclsem) != 0)
        {
            /* The only case that an error should occur here is if the wait was awakened
            * by a signal.
            */

            ASSERT(errno == EINTR);
        }
    }
    else
    {
        (void)sem_post(&priv->exclsem);
    }

    return OK;
}

/************************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ************************************************************************************/

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
    FAR struct ar_spidev_s *priv = (FAR struct ar_spidev_s *)dev;
    uint32_t actual;
    uint32_t divider;
    uint32_t add;
    uint32_t spiclkmax = priv->spiclock;

    /* Limit to max possible (if STM32_SPI_CLK_MAX is defined in board.h) */

        
    if (frequency > spiclkmax)
    {
        frequency = spiclkmax;
    }

    /* Has the frequency changed? */

    if(frequency == 0)
    {
        
        ;
    }
    else if (frequency != priv->frequency)
    {
        /* Choices are limited by PCLK frequency with a set of divisors */


        add = 0;
        if (0 != spiclkmax % frequency)
        {
            add = 1;
        }

        divider = spiclkmax/frequency + add;
        actual = spiclkmax/divider;


        spi_putreg32(priv, SPI_SSIENR, 0x00);            //disable ssi
        spi_putreg32(priv, SPI_BAUDR, divider);
        spi_putreg32(priv, SPI_SSIENR, 0x01);            //enable ssi

        /* Save the frequency selection so that subsequent reconfigurations will be
        * faster.
        */

        priv->frequency = frequency;
        priv->actual    = actual;
    }

    return priv->actual;
}

/************************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ************************************************************************************/

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
    FAR struct ar_spidev_s *priv = (FAR struct ar_spidev_s *)dev;
    uint32_t setbits;
    uint32_t clrbits;

    /* Has the mode changed? */

    if (mode != priv->mode)
    {
        /* Yes... Set CR1 appropriately */

        switch (mode)
        {
            case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
                setbits = 0;
                clrbits = SPI_CTRLR0_SCPOL | SPI_CTRLR0_SCPH;
                break;

            case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
                setbits = SPI_CTRLR0_SCPH;
                clrbits = SPI_CTRLR0_SCPOL;
                break;

            case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
                setbits = SPI_CTRLR0_SCPOL;
                clrbits = SPI_CTRLR0_SCPH;
                break;

            case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
                setbits = SPI_CTRLR0_SCPOL | SPI_CTRLR0_SCPH;
                clrbits = 0;
                break;

            default:
                return;
        }

        setbits |= SPI_CTRLR0_FRF_MOTO;

        clrbits |= SPI_CTRLR0_TMOD_MASK; // only tx & rx mode
        

        spi_putreg32(priv, SPI_SSIENR, 0x00);            //disable ssi
        spi_modifyctrlr0(priv, setbits, clrbits);
        spi_putreg32(priv, SPI_SSIENR, 0x01);            //enable ssi

        /* Save the mode so that subsequent re-configurations will be faster */

        priv->mode = mode;
    }
}

/************************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
    FAR struct ar_spidev_s *priv = (FAR struct ar_spidev_s *)dev;
    uint32_t data;
    int savbits = nbits;
   
    /* Has the number of bits changed? */

    if (nbits != priv->nbits)
    {
        /* Set the number of bits (valid range 4-16) */

        if (nbits < 4 || nbits > 16)
        {
            return;
        }

        spi_putreg32(priv, SPI_SSIENR, 0x00);            //disable ssi

        data = spi_getreg32(priv, SPI_CTRLR0) & (~(SPI_CTRLR0_DFS_MASK));
        data |= (nbits - 1);
        spi_putreg32(priv, SPI_CTRLR0, data);

        spi_putreg32(priv, SPI_SSIENR, 0x01);            //enable ssi

        /* Save the selection so the subsequence re-configurations will be faster */

        priv->nbits = savbits; // nbits has been clobbered... save the signed value.
    }
}

/****************************************************************************
 * Name: spi_hwfeatures
 *
 * Description:
 *   Set hardware-specific feature flags.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   features - H/W feature flags
 *
 * Returned Value:
 *   Zero (OK) if the selected H/W features are enabled; A negated errno
 *   value if any H/W feature is not supportable.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int spi_hwfeatures(FAR struct spi_dev_s *dev, spi_hwfeatures_t features)
{
#ifdef CONFIG_SPI_BITORDER
    FAR struct ar_spidev_s *priv = (FAR struct ar_spidev_s *)dev;
    uint16_t setbitscr1;
    uint16_t clrbitscr1;
    uint16_t setbitscr2;
    uint16_t clrbitscr2;
    int savbits = nbits;

    /* Transfer data LSB first? */

    if ((features & HWFEAT_LSBFIRST) != 0)
    {
        setbits = SPI_CR1_LSBFIRST;
        clrbits = 0;
    }
    else
    {
        setbits = 0;
        clrbits = SPI_CR1_LSBFIRST;
    }

    spi_modifycr1(priv, 0, SPI_CR1_SPE);
    spi_modifycr1(priv, setbits, clrbits);
    spi_modifycr1(priv, SPI_CR1_SPE, 0);

    /* Other H/W features are not supported */

    return ((features & ~HWFEAT_LSBFIRST) == 0) ? OK : -ENOSYS;
#else
    return -ENOSYS;
#endif
}
#endif

/************************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ************************************************************************************/

static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
    FAR struct ar_spidev_s *priv = (FAR struct ar_spidev_s *)dev;
    uint32_t regval;
    uint16_t ret;

    DEBUGASSERT(priv && priv->spibase);

    /* According to the number of bits, access data register as word or byte
    * This is absolutely required because of packing. With nbits <=8 bit frames,
    * two bytes are received by a 16-bit read of the data register!
    */
    
    if (spi_9to16bitmode(priv))
    {       
        spi_putreg32(priv, SPI_SLAVE_EN, 0);
        spi_writeword(priv, wd);
        spi_putreg32(priv, SPI_SLAVE_EN, 1 );        
        ret = spi_readword(priv);
    }
    else
    {
        uint8_t data = (wd & 0xFF);
        spi_putreg32(priv, SPI_SLAVE_EN, 0);
        spi_writebyte(priv, data);
        spi_putreg32(priv, SPI_SLAVE_EN, 1 );
        ret = (uint16_t)spi_readbyte(priv);
    }

    /* Check and clear any error flags (Reading from the SR clears the error
    * flags).
    */

    regval = spi_getreg32(priv, SPI_SR);


    
    UNUSED(regval);
    return ret;
}

/************************************************************************************
 * Name: spi_exchange (no DMA).  aka spi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchaned in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void ar_send_data(FAR struct ar_spidev_s *priv) 
{
    int wordLens = priv->txnwords > 8 ? 8:priv->txnwords;

    if(priv->txbuffer == NULL)
    {
        for(int i=0;i<wordLens;i++) 
        {
            spi_putreg32(priv,SPI_DR,0xff);
        }
    }
    else
    {
        for(int i=0;i<wordLens;i++) 
        {
            spi_putreg32(priv,SPI_DR,*(priv->txbuffer++));
        }
    }
    

    priv->txnwords -= wordLens;
}















#if !defined(CONFIG_AR_SPI_DMA) || defined(CONFIG_AR_DMACAPABLE)
#if !defined(CONFIG_AR_SPI_DMA)
static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
#else
static void spi_exchange_nodma(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                               FAR void *rxbuffer, size_t nwords)
#endif
{
    FAR struct ar_spidev_s *priv = (FAR struct ar_spidev_s *)dev;
    DEBUGASSERT(priv && priv->spibase);

    irqstate_t state = enter_critical_section();
    /* 8- or 16-bit mode? */

    if (spi_9to16bitmode(priv))
    {
        /* 16-bit mode */
        const uint16_t *src  = (const uint16_t *)txbuffer;
        uint16_t *dest = (uint16_t *)rxbuffer;
        uint16_t  word;

        while (nwords-- > 0)
        {
            /* Get the next word to write.  Is there a source buffer? */

            if (src)
            {
                word = *src++;
            }
            else
            {
                word = 0xffff;
            }

            /* Exchange one word */

            word = spi_send(dev, word);

            /* Is there a buffer to receive the return value? */

            if (dest)
            {
                *dest++ = word;
            }
        }
    }
    else
    {
        /* 8-bit mode */

        priv->txbuffer = (const uint8_t *)txbuffer;
        priv->rxbuffer = (uint8_t *)rxbuffer;
        priv->txnwords = nwords;
        priv->rxnwords = nwords;

        spi_getreg32(priv,SPI_ICR);
        ar_spi_disable_all_interrupts(priv);//add

        spi_putreg32(priv, SPI_SLAVE_EN, 1 );
   
        ar_send_data(priv);

        spi_putreg32(priv,SPI_IMR,SPI_IMR_TXEIM | SPI_IMR_RXFIM);

        do
        {
            uint32_t status;
            status = spi_getreg32(priv, SPI_ISR);
            if(status == 0)
            {
                continue;
            }

            if (status & SPI_ISR_RXFIS)  //rx
            {
                uint32_t level = spi_getreg32(priv,SPI_RXFLR);      //to do 获得fifo的深度
                
                for(int i=0;i<level;i++)
                {
                    uint32_t data = spi_getreg32(priv,SPI_DR);
                    if(priv->rxbuffer != NULL)
                    {
                        *(priv->rxbuffer) = data;
                        priv->rxbuffer ++;
                    }

                                    
                    priv->rxnwords--;
                }
                
                

                if ((priv->txnwords == priv->rxnwords) && (priv->txnwords != 0))
                {
                    uint8_t imrtem = spi_getreg32(priv,SPI_IMR);
                    imrtem = imrtem | SPI_IMR_TXEIM;
                    spi_putreg32(priv,SPI_IMR,imrtem);
                }

                if(priv->rxnwords <=0) 
                {
                    spi_getreg32(priv, SPI_ICR);
                    ar_spi_disable_all_interrupts(priv);
                }
                
            }

            if (status & SPI_ISR_TXEIS) //tx 
            {  

                if(priv->txnwords > 0) 
                {
                    if(priv->txnwords == priv->rxnwords)
                    {
                        ar_send_data(priv);
                    } 
                    else 
                    {
                        uint8_t imrtem = spi_getreg32(priv,SPI_IMR);
                        imrtem = imrtem & (~SPI_IMR_TXEIM);
                        spi_putreg32(priv,SPI_IMR,imrtem);
                    }
                    
                } 
                else
                {   //清tx中断
                    uint8_t imrtem = spi_getreg32(priv,SPI_IMR);
                    imrtem = imrtem & (~SPI_IMR_TXEIM);
                    spi_putreg32(priv,SPI_IMR,imrtem);

                }
                
            }

            if (status & (~(SPI_ISR_TXEIS | SPI_ISR_RXFIS)))    // some error happpened.
            {
                spi_putreg32(priv, SPI_IMR,0);
            }

        }
        while(priv->txnwords || priv->rxnwords);

        leave_critical_section(state);
        // while (nwords-- > 0)
        // {
        //     /* Get the next word to write.  Is there a source buffer? */

        //     if (src)
        //     {
        //         word = *src++;
        //     }
        //     else
        //     {
        //         word = 0xff;
        //     }
            
        //     /* Exchange one byte */
        //     word = (uint8_t)spi_send(dev, (uint16_t)word);
        //     /* Is there a buffer to receive the return value? */

        //     if (dest)
        //     {
        //         *dest++ = word;
        //     }
        // }
    }


}
#endif /* !CONFIG_AR_SPI_DMA || CONFIG_AR_DMACAPABLE */

/****************************************************************************
 * Name: spi_exchange (with DMA capability)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_AR_SPI_DMA
static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
    FAR struct ar_spidev_s *priv = (FAR struct ar_spidev_s *)dev;

    DEBUGASSERT(priv != NULL);

#ifdef CONFIG_AR_DMACAPABLE
    if ((txbuffer && !ar_dmacapable((uint32_t)txbuffer, nwords, priv->txccr)) ||
        (rxbuffer && !ar_dmacapable((uint32_t)rxbuffer, nwords, priv->rxccr)))
    {
        /* Unsupported memory region, fall back to non-DMA method. */

        spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
    }
    else
#endif
    {
        static uint8_t rxdummy[ARMV7M_DCACHE_LINESIZE]
        __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
        static const uint16_t txdummy = 0xffff;
        size_t buflen = nwords;

        if (spi_9to16bitmode(priv))
        {
            buflen = nwords * sizeof(uint16_t);
        }

        DEBUGASSERT(priv->spibase != 0);

        /* Setup DMAs */

        spi_dmarxsetup(priv, rxbuffer, (uint16_t *)rxdummy, nwords);
        spi_dmatxsetup(priv, txbuffer, &txdummy, nwords);

        /* Flush cache to physical memory */

        if (txbuffer)
        {
            arch_flush_dcache((uintptr_t)txbuffer, (uintptr_t)txbuffer + buflen);
        }

        /* Start the DMAs */

        spi_dmarxstart(priv);
        spi_dmatxstart(priv);

        /* Then wait for each to complete */

        spi_dmarxwait(priv);
        spi_dmatxwait(priv);

        /* Force RAM re-read */

        if (rxbuffer)
        {
            arch_invalidate_dcache((uintptr_t)rxbuffer,
                                    (uintptr_t)rxbuffer + buflen);
        }
        else
        {
            arch_invalidate_dcache((uintptr_t)rxdummy,
                                    (uintptr_t)rxdummy + sizeof(rxdummy));
        }
    }
}
#endif /* CONFIG_AR_SPI_DMA */

/****************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer, size_t nwords)
{
  return spi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/************************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to recieve data
 *   nwords   - the length of data that can be received in the buffer in number
 *              of words.  The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer, size_t nwords)
{
  return spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/************************************************************************************
 * Name: spi_bus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus in its default state (Master, 8-bit, mode 0, etc.)
 *
 * Input Parameter:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_bus_initialize(FAR struct ar_spidev_s *priv)
{

    priv->frequency = 0;
    priv->nbits     = 8;
    priv->mode      = SPIDEV_MODE0;

    spi_putreg32(priv, SPI_SLAVE_EN, 0);
    spi_putreg32(priv, SPI_SSIENR, 0x00);          //disable ssi
    spi_putreg32(priv, SPI_CTRLR0, SPI_CTRL0_DEF_VALUE);
    /* Select a default frequency of approx. 400KHz */
    spi_setfrequency((FAR struct spi_dev_s *)priv, 400000); /* Select a default frequency of approx. 400KHz */
    spi_putreg32(priv,SPI_TXFTLR,0);
    spi_putreg32(priv,SPI_RXFTLR,0);//Rx
    spi_getreg32(priv, SPI_ICR);

    sem_init(&priv->exclsem, 0, 1);

#ifdef CONFIG_AR_SPI_DMA
  /* Initialize the SPI semaphores that is used to wait for DMA completion.
   * This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  sem_init(&priv->rxsem, 0, 0);
  sem_init(&priv->txsem, 0, 0);

  sem_setprotocol(&priv->rxsem, SEM_PRIO_NONE);
  sem_setprotocol(&priv->txsem, SEM_PRIO_NONE);

  /* Get DMA channels.  NOTE: stm32_dmachannel() will always assign the DMA channel.
   * if the channel is not available, then stm32_dmachannel() will block and wait
   * until the channel becomes available.  WARNING: If you have another device sharing
   * a DMA channel with SPI and the code never releases that channel, then the call
   * to stm32_dmachannel()  will hang forever in this function!  Don't let your
   * design do that!
   */

  priv->rxdma = ar_dmachannel(priv->rxch);
  priv->txdma = ar_dmachannel(priv->txch);
  DEBUGASSERT(priv->rxdma && priv->txdma);

//   spi_modifycr2(priv, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN, 0);
#endif
    spi_putreg32(priv, SPI_SSIENR, 0x01);          //enable ssi
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: ar_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ************************************************************************************/

FAR struct spi_dev_s *ar_spibus_initialize(int bus)
{
    FAR struct ar_spidev_s *priv = NULL;

    irqstate_t flags = enter_critical_section();
#ifdef CONFIG_AR_SPI0
    if (bus == 0)
    {
        /* Select SPI1 */

        priv = &g_spi0dev;

        /* Only configure if the bus is not already configured */
        
        if (spi_getreg32(priv, SPI_SSIENR) == 0)
        {
            /* Configure SPI1 pins: SCK, MISO, and MOSI */

            ar_configgpio(GPIO_SPI0_SCK);
            ar_configgpio(GPIO_SPI0_MISO);
            ar_configgpio(GPIO_SPI0_MOSI);

            /* Set up default configuration: Master, 8-bit, etc. */

            spi_bus_initialize(priv);
        }
    }
    else
#endif

#ifdef CONFIG_AR_SPI1
    if (bus == 1)
    {
        /* Select SPI1 */

        priv = &g_spi1dev;

        /* Only configure if the bus is not already configured */
        
        if (spi_getreg32(priv, SPI_SSIENR) == 0)
        {
            /* Configure SPI1 pins: SCK, MISO, and MOSI */

            ar_configgpio(GPIO_SPI1_SCK);
            ar_configgpio(GPIO_SPI1_MISO);
            ar_configgpio(GPIO_SPI1_MOSI);

            /* Set up default configuration: Master, 8-bit, etc. */

            spi_bus_initialize(priv);
        }
    }
    else
#endif
#ifdef CONFIG_AR_SPI2
    if (bus == 2)
    {
        /* Select SPI2 */

        priv = &g_spi2dev;

        /* Only configure if the bus is not already configured */
        
        if (spi_getreg32(priv, SPI_SSIENR) == 0)
        {
            /* Configure SPI2 pins: SCK, MISO, and MOSI */

            ar_configgpio(GPIO_SPI2_SCK);
            ar_configgpio(GPIO_SPI2_MISO);
            ar_configgpio(GPIO_SPI2_MOSI);

            /* Set up default configuration: Master, 8-bit, etc. */

            spi_bus_initialize(priv);
        }
    }
    else
#endif
#ifdef CONFIG_AR_SPI3
    if (bus == 3)
    {
        /* Select SPI3 */

        priv = &g_spi3dev;

        /* Only configure if the bus is not already configured */
        
        if (spi_getreg32(priv, SPI_SSIENR) == 0)
        {
            /* Configure SPI3 pins: SCK, MISO, and MOSI */

            ar_configgpio(GPIO_SPI3_SCK);
            ar_configgpio(GPIO_SPI3_MISO);
            ar_configgpio(GPIO_SPI3_MOSI);

            /* Set up default configuration: Master, 8-bit, etc. */

            spi_bus_initialize(priv);
        }
    }
    else
#endif
#ifdef CONFIG_AR_SPI4
    if (bus == 4)
    {
        /* Select SPI4 */

        priv = &g_spi4dev;

        /* Only configure if the bus is not already configured */
        
        if (spi_getreg32(priv, SPI_SSIENR) == 0)
        {
            /* Configure SPI4 pins: SCK, MISO, and MOSI */

            ar_configgpio(GPIO_SPI4_SCK);
            ar_configgpio(GPIO_SPI4_MISO);
            ar_configgpio(GPIO_SPI4_MOSI);

            /* Set up default configuration: Master, 8-bit, etc. */

            spi_bus_initialize(priv);
        }
    }
    else
#endif
#ifdef CONFIG_AR_SPI5
    if (bus == 5)
    {
        /* Select SPI5 */

        priv = &g_spi5dev;

        /* Only configure if the bus is not already configured */
        
        if (spi_getreg32(priv, SPI_SSIENR) == 0)
        {
            /* Configure SPI5 pins: SCK, MISO, and MOSI */

            ar_configgpio(GPIO_SPI5_SCK);
            ar_configgpio(GPIO_SPI5_MISO);
            ar_configgpio(GPIO_SPI5_MOSI);

            /* Set up default configuration: Master, 8-bit, etc. */

            spi_bus_initialize(priv);
        }
    }
    else
#endif
#ifdef CONFIG_AR_SPI6
    if (bus == 6)
    {
        /* Select SPI6 */

        priv = &g_spi6dev;

        /* Only configure if the bus is not already configured */
        
        if (spi_getreg32(priv, SPI_SSIENR) == 0)
        {
            /* Configure SPI6 pins: SCK, MISO, and MOSI */

            ar_configgpio(GPIO_SPI6_SCK);
            ar_configgpio(GPIO_SPI6_MISO);
            ar_configgpio(GPIO_SPI6_MOSI);

            /* Set up default configuration: Master, 8-bit, etc. */

            spi_bus_initialize(priv);
        }
    }
    else
#endif
    {
        spierr("ERROR: Unsupported SPI bus: %d\n", bus);
        return NULL;
    }

    leave_critical_section(flags);
    return (FAR struct spi_dev_s *)priv;
}

#endif  /* CONFIG_AR_SPI0 || CONFIG_AR_SPI1 || CONFIG_AR_SPI2 || \
        * CONFIG_AR_SPI3 || CONFIG_AR_SPI4 || CONFIG_AR_SPI5    \
        * CONFIG_AR_SPI6 */



