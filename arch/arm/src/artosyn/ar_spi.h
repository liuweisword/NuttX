/************************************************************************************
 * arch/arm/src/artosyn/ar_spi.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_ARTOSYN_AR_SPI_H
#define __ARCH_ARM_SRC_ARTOSYN_AR_SPI_H

#include <nuttx/config.h>
#include <stdbool.h>
#include <stdint.h>
#include "chip.h"
#include "chip/ar8020_spi.h"
 
#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct spi_dev_s;

/************************************************************************************
 * Name: ar_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameter:
 *   bus number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ************************************************************************************/

FAR struct spi_dev_s *ar_spibus_initialize(int bus);

/************************************************************************************
 * Name:  ar_spi1/2/...select and ar_spi1/2/...status
 *
 * Description:
 *   The external functions, ar_spi1/2/...select, ar_spi1/2/...status, and
 *   ar_spi1/2/...cmddata must be provided by board-specific logic.  These are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *   (including ar_spibus_initialize()) are provided by common ar logic.  To use this
 *   common SPI logic on your board:
 *
 *   1. Provide logic in ar_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide ar_spi1/2/...select() and ar_spi1/2/...status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration file, then
 *      provide ar_spi1/2/...cmddata() functions in your board-specific logic.
 *      These functions will perform cmd/data selection operations using GPIOs in the
 *      way your board is configured.
 *   4. Add a calls to ar_spibus_initialize() in your low level application
 *      initialization logic
 *   5. The handle returned by ar_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/
#ifdef CONFIG_AR_SPI0
void ar_spi0select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t ar_spi0status(FAR struct spi_dev_s *dev, uint32_t devid);
int ar_spi0cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_AR_SPI1
void ar_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t ar_spi1status(FAR struct spi_dev_s *dev, uint32_t devid);
int ar_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_AR_SPI2
void ar_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t ar_spi2status(FAR struct spi_dev_s *dev, uint32_t devid);
int ar_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_AR_SPI3
void ar_spi3select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t ar_spi3status(FAR struct spi_dev_s *dev, uint32_t devid);
int ar_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_AR_SPI4
void ar_spi4select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t ar_spi4status(FAR struct spi_dev_s *dev, uint32_t devid);
int ar_spi4cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_AR_SPI5
void ar_spi5select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t ar_spi5status(FAR struct spi_dev_s *dev, uint32_t devid);
int ar_spi5cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_AR_SPI6
void ar_spi6select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t ar_spi6status(FAR struct spi_dev_s *dev, uint32_t devid);
int ar_spi6cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

/************************************************************************************
 * Name: ar_spi1/2/...register
 *
 * Description:
 *   If the board supports a card detect callback to inform the SPI-based MMC/SD
 *   driver when an SD card is inserted or removed, then CONFIG_SPI_CALLBACK should
 *   be defined and the following function(s) must be implemented.  These functions
 *   implements the registercallback method of the SPI interface (see
 *   include/nuttx/spi/spi.h for details)
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The function to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CALLBACK
#ifdef CONFIG_AR_SPI0
int ar_spi0register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
                       FAR void *arg);
#endif

#ifdef CONFIG_AR_SPI1
int ar_spi1register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
#endif

#ifdef CONFIG_AR_SPI2
int ar_spi2register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
#endif

#ifdef CONFIG_AR_SPI3
int ar_spi3register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
#endif

#ifdef CONFIG_AR_SPI4
int ar_spi4register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
#endif

#ifdef CONFIG_AR_SPI5
int ar_spi5register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
#endif

#ifdef CONFIG_AR_SPI6
int ar_spi6register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
#endif

#endif

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
#define SPI_BASE_0                  (0x40100000)    //ssi_0 
#define SPI_BASE_1                  (0x40120000)    //ssi_1 
#define SPI_BASE_2                  (0x40140000)    //ssi_2 
#define SPI_BASE_3                  (0x40160000)    //ssi_3 
#define SPI_BASE_4                  (0x40180000)    //ssi_4 
#define SPI_BASE_5                  (0x401a0000)    //ssi_5 
#define SPI_BASE_6                  (0x401c0000)    //ssi_6

#define SPI_CTRLR0                  (0x00)    //Control Register0
#define SPI_CTRLR1                  (0x04)    //Control Register1
#define SPI_SSIENR                  (0x08)    //SSI Enable Reg
#define SPI_MWCR                    (0x0C)    //Microwire Control Reg
#define SPI_SLAVE_EN                (0x10)    //Slave Enable Reg
#define SPI_BAUDR                   (0x14)    //Buad Rate Select
#define SPI_TXFTLR                  (0x18)    //Transmit FIFO Threshold Level
#define SPI_RXFTLR                  (0x1c)    //Receive FIFO Threshold Level
#define SPI_TXFLR                   (0x20)    //Transmit FIFO Level Register
#define SPI_RXFLR                   (0x24)    //Receive FIFO Level Register
#define SPI_SR                      (0x28)    //Status Register
#define SPI_IMR                     (0x2C)    //Interrupt Mask Register
#define SPI_ISR                     (0x30)    //Interrupt Status Register
#define SPI_RISR                    (0x34)    //Raw Interrupt Status Register
#define SPI_TXOICR                  (0x38)    //Transmit FIFO Overflow Interrupt Clear Register
#define SPI_RXOICR                  (0x3c)    //Receive FIFO OVerflow Interrupt Clear Register
#define SPI_RXUICR                  (0x40)    //Receive FIFO Underflow interrupt Clear register
#define SPI_MSTICR                  (0x44)    //Multi-master Interrupt Clear Register
#define SPI_ICR                     (0x48)    //Interrupt Clear Register
#define SPI_DMACR                   (0x4C)    //DMA Control Register
#define SPI_DMATDLR                 (0x50)    //DMA Transmit Data Level 
#define SPI_DMARDLR                 (0x54)    //DMA Receive Data Level 
#define SPI_IDR                     (0x58)    //Identification Register
#define SPI_COMP_VERSION            (0x5C)    //
#define SPI_DR                      (0x60)    //Data Register


#define SPI_BASE_CLK_MHZ            (166)
#define SPI_TXFLR_MAX               (8)
#define SPI_RXFLR_MAX               (8)
#define SPI_MAX_CHANNEL             (8) 

#define SPI_CTRL0_DEF_VALUE         (0x347)
#define SPI_TXFTLR_DEF_VALUE        (0x04)
#define SPI_RXFTLR_DEF_VALUE        (0x03)
#define SPI_SSIENR_DEF_VALUE        (0x01)

#define SPI_ISR_MASK                (0x3F) // 
#define SPI_ISR_MSTIS               (0x20) // Multi-Master Contention Interrupt Status
#define SPI_ISR_RXFIS               (0x10) // Receive FIFO Full Interrupt Status
#define SPI_ISR_RXOIS               (0x08) // Receive FIFO Overflow Interrupt Status
#define SPI_ISR_RXUIS               (0x04) // Receive FIFO Underflow Interrupt Status
#define SPI_ISR_TXOIS               (0x02) // Transmit FIFO Overflow Interrupt Status
#define SPI_ISR_TXEIS               (0x01) // Transmit FIFO Empty Interrupt Status

#define SPI_CTRLR0_TMOD_MASK        (0x03 << 8) //
#define SPI_CTRLR0_TMOD_TR          (0x00 << 8) //Transmit & Receive
#define SPI_CTRLR0_TMOD_TO          (0x01 << 8) //Transmit Only
#define SPI_CTRLR0_TMOD_RO          (0x02 << 8) //Receive Only
#define SPI_CTRLR0_TMOD_EE          (0x03 << 8) //EEPROM Read
   
#define SPI_CTRLR0_DFS_MASK         (0x0F) // 
#define SPI_CTRLR0_DFS_8BIT         (0x07) // 8 bit
#define SPI_CTRLR0_DFS_16BIT        (0x0F) // 16 bit

#define SPI_CTRLR0_SCPOL            (1 << 7)
#define SPI_CTRLR0_SCPH             (1 << 6)

#define SPI_CTRLR0_FRF_MASK         (0x03 << 4)        
#define SPI_CTRLR0_FRF_MOTO         (0 << 4)
#define SPI_CTRLR0_FRF_TI           (1 << 4)
#define SPI_CTRLR0_FRF_NSM          (2 << 4)

#define SPI_IMR_MASK                (0x3F) //
#define SPI_IMR_MSTIM               (0x20) // Multi-Master Contention Interrupt Mask
#define SPI_IMR_RXFIM               (0x10) // Receive FIFO Full Interrupt Mask
#define SPI_IMR_RXOIM               (0x08) // Receive FIFO Overflow Interrupt Mask
#define SPI_IMR_RXUIM               (0x04) // Receive FIFO Underflow Interrupt Mask
#define SPI_IMR_TXOIM               (0x02) // Transmit FIFO Overflow Interrupt Mask
#define SPI_IMR_TXEIM               (0x01) // Transmit FIFO Empty Interrupt Mask

#define SPI_SR_DCOL                 (0x40) // Data Collision Error.
#define SPI_SR_TXE                  (0x20) // Transmission Error
#define SPI_SR_RFF                  (0x10) // Receive FIFO Full.
#define SPI_SR_RFNE                 (0x08) // Receive FIFO Not Empty
#define SPI_SR_TFE                  (0x04) // Transmit FIFO Empty
#define SPI_SR_TFNF                 (0x02) // Transmit FIFO Not Full
#define SPI_SR_BUSY                 (0x01) // SSI Busy Flag

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_ARTOSYN_AR_SPI_H */

