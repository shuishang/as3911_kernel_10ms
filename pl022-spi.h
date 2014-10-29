/*****************************************************************************
* Copyright 2009 - 2011 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/


#ifndef BCM5892_PL022_SPI_H
#define BCM5892_PL022_SPI_H

/* the spi->mode bits understood by this driver: */
#define MODEBITS        (SPI_CS_HIGH|SPI_CPOL|SPI_CPHA)

#define SPI_REG_SIZE    (0x28)

#define SPI_CMD_BUF_SIZE        256             // in bytes
#define SPI_DATA_BUF_SIZE       4096    // in bytes

#define SPI_WAIT_DMA_CHAN       100

#define PL022_CR0   (0x00)
#define PL022_CR1   (0x04)
#define PL022_DR    (0x08)
#define PL022_SR    (0x0c)
#define PL022_CPSR  (0x10)
#define PL022_IMSC  (0x14)
#define PL022_RIS   (0x18)
#define PL022_MIS   (0x1c)
#define PL022_CIS   (0x20)
#define PL022_DMACR (0x24)

#define PL022_SCR_MAP(scr) ((uint16_t)scr << 8)
#define PL022_SPH_MAP(sph) ((sph) ? 0x80 : 0)
#define PL022_SPO_MAP(spo) ((spo) ? 0x40 : 0)
#define PL022_DSS_MAP(data_size) ((data_size - 1) & 0xf)

#define PL022_FRF_SPI (0x00)
#define PL022_FRF_TI  (0x10)
#define PL022_FRF_MW  (0x20)

#define PL022_SR_BSY (0x10)
#define PL022_SR_RFF (0x08)
#define PL022_SR_RNE (0x04)
#define PL022_SR_TNF (0x02)
#define PL022_SR_TFE (0x01)

#define PL022_IRQ_TXW (0x08)   /* Transmit watermark */
#define PL022_IRQ_RXW (0x04)   /* Receive watermark */
#define PL022_IRQ_RXT (0x02)   /* Receive timeout */
#define PL022_IRQ_RXO (0x01)   /* Receive overflow */

#define PL022_DMACR_ENABLE  (0x03)  /* we always use DMA on rx and tx together */
#define PL022_DMACR_DISABLE (0x00)

#define PL022_REG(base, reg) ioread32(IO_ADDRESS(base) + reg)
#define PL022_WRITE_REG(val, base, reg) iowrite32(val, IO_ADDRESS(base) + reg)

#endif /* #ifndef BCM5892_PL022_SPI_H */
