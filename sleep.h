/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */
/*
 *      PROJECT:   AS3911 firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file sleep.h
 *
 *  \author Oliver Regenfelder
 *
 *  \brief Sleep function.
 */

#ifndef SLEEP_H
#define SLEEP_H

/*! \defgroup sleep Sleep
 *****************************************************************************
 * \brief Provides a sleep function.
 *****************************************************************************
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "ams_types.h"

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


 
#define CPSR_MAX 254  /* prescale divisor */
#define CPSR_MIN 2    /* prescale divisor min (must be divisible by 2) */
#define SCR_MAX 255   /* other divisor */
#define SCR_MIN 0
 
 //#define SPI_INPUT_CLOCK (66000000)  /* better if we can get this from the board */
 #define SPI_INPUT_CLOCK (50000000)
 
#define MAX_CALC_SPEED_HZ (SPI_INPUT_CLOCK / (CPSR_MIN * (1 + SCR_MIN)))
#define MIN_CALC_SPEED_HZ (SPI_INPUT_CLOCK / (254 * 256))
#define MAX_SUPPORTED_SPEED_HZ (12000000)
 
#define MAX_SPEED_HZ ( (MAX_SUPPORTED_SPEED_HZ < MAX_CALC_SPEED_HZ) ? MAX_SUPPORTED_SPEED_HZ : MAX_CALC_SPEED_HZ)
#define MIN_SPEED_HZ (MIN_CALC_SPEED_HZ)
 
#define SPI_TXFIFO_SIZE (8)
#define SPI_RXFIFO_SIZE (16)
#define TEST_BUF_SIZE 16



void sleepMilliseconds(unsigned int milliseconds);
void measure_counter_setup(void);
void measure_counter_start(void);
void measure_counter_stop(void);
unsigned int get_timer_count(void);
void quck_timer_count(u8 flag);


  void quck_ssp_setup(void);
 void quck_ssp_start(void);
 void quck_ssp_stop(void);
unsigned char  quck_ssp_read_printk(u8 *buf,u8 count);
u8 quck_ssp_write_printk( u8 * buf ,u8 count );




  
#endif /* SLEEP_H */
