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


#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <mach/shm.h>
#include <mach/reg_gpio.h>
#include <linux/amba/bus.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sizes.h>

#include <linux/broadcom/bcm_sysctl.h>
#include <linux/broadcom/timer.h>
//#include "../dma/pl080-dma.h"
#include "mach/pl080-dma.h"
#include "pl022-spi.h"

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

#define PRINTK (!debug_on) ?: printk
#define PROVIDE_LOOPBACK_TEST 1

/* ================================================================================================ */
/*  Forward Definitions (required for SPI and AMBA structs & vars) */
/* ================================================================================================ */

static ssize_t show_enabled_sysfs(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t set_enabled_sysfs(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_dma_sysfs(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t set_dma_sysfs(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_debug_sysfs(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t set_debug_sysfs(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

#if PROVIDE_LOOPBACK_TEST
static void pl022_spi_loopback_test (struct device *pDev, int startpos);
#endif

static int pl022_spi_setup (struct spi_device *pDev);
static int pl022_spi_transfer (struct spi_device *pDev, struct spi_message *pMsg);
static void pl022_spi_cleanup (struct spi_device *pDev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
static int pl022_amba_probe(struct amba_device *pDev, void *id);
#else
static int pl022_amba_probe(struct amba_device *pDev, struct amba_id *id);
#endif
static int pl022_amba_remove(struct amba_device *pDev);

#define FIRST_NONZERO(a,b,c) ({ typeof(a) _a = a; typeof(b) _b = b; typeof(c) _c = c; _a ? _a : (_b ? _b : _c); })

static int debug_on = 0;


/* ================================================================================================
 *  SPI Master Device Configuration/State
 * ================================================================================================
 */
struct pl022_master_device_data {
	struct work_struct work;

	struct workqueue_struct *wq;
	struct list_head msg_queue;
	spinlock_t lock;      /* lock for queue access */
	spinlock_t fifolock;  /* lock on fifo access */
	struct completion isr_completion;

	int polled;

	struct spi_transfer *cur_txfr;
	unsigned cur_txfr_numwords_left_tx;
	unsigned cur_txfr_numwords_left_rx;
	unsigned cur_txfr_tx_idx;
	unsigned cur_txfr_rx_idx;

	struct spi_device *last_spi_device;  /* for debug and test */
	unsigned errors;
};


struct pl022_platform_data {
	void   __iomem	  *base;	       /* PL022 SPI peripheral register base */
	int		  cs_gpio_num;
	int		  bus_num;
	unsigned	  is_enabled;
	unsigned          enable_dma;
	int               dma_tx_id;
	int               dma_rx_id;
};


/* ================================================================================================
 *  AMBA Device Structures & Variables
 * ================================================================================================
 */
static struct amba_id pl022_spi_amba_id_table[] = {
	{
		.id   = 0x000041022,
		.mask = 0x0000fffff,
	},
	{}
};

static struct amba_driver pl022_amba_driver = {
	.drv = {
		.name = "bcm589x-spi",
	},
	.id_table	= pl022_spi_amba_id_table,
	.probe		= pl022_amba_probe,
	.remove		= pl022_amba_remove,
};

/* ================================================================================================
 *  Hardware Helpers
 * ================================================================================================
 */

static int enable_periph(int gpio_aux, uint32_t bitmask, int setAux01)
{
	REG_1ST_GPIO_FROM_AUX(gpio_aux, REGOFFSET_GPIO_AUX_SEL) |= bitmask;

	if (setAux01) {
		REG_1ST_GPIO_FROM_AUX(gpio_aux, REGOFFSET_GPIO_AUX01_SEL) |= bitmask;
	} else {
		REG_1ST_GPIO_FROM_AUX(gpio_aux, REGOFFSET_GPIO_AUX01_SEL) &= ~bitmask;
	}
	return 0;
}


static int disable_periph(int gpio_aux, uint32_t bitmask)
{
	REG_1ST_GPIO_FROM_AUX(gpio_aux, REGOFFSET_GPIO_AUX_SEL) &= ~bitmask;
	REG_1ST_GPIO_FROM_AUX(gpio_aux, REGOFFSET_GPIO_AUX01_SEL) &= ~bitmask;

	return 0;
}


static void enable_pins(struct pl022_platform_data * pPlatform)
{
	switch (pPlatform->bus_num) {
	case 0:
		enable_periph(GPIO_AUX_SPI0, 0xf, 0);
		break;
	case 1:
		enable_periph(GPIO_AUX_SPI1, 0xf, 0);
		break;
	case 3:
		enable_periph(GPIO_AUX_SPI3, 0xf, 1);
		break;
	}

	reg_gpio_set_pin(pPlatform->cs_gpio_num, 1);  /* default CS resting state is high */
	reg_gpio_iotr_set_pin_type(pPlatform->cs_gpio_num, GPIO_PIN_TYPE_OUTPUT);
}


static void disable_pins(struct pl022_platform_data * pPlatform)
{
	switch (pPlatform->bus_num) {
	case 0:
		disable_periph(GPIO_AUX_SPI0, 0xf);
		break;
	case 1:
		disable_periph(GPIO_AUX_SPI1, 0xf);
		break;
	case 3:
		disable_periph(GPIO_AUX_SPI3, 0xf);
		break;
	case 4:
		disable_periph(GPIO_AUX_SPI4, 0x1e0);
		break;
	}
	reg_gpio_iotr_set_pin_type(pPlatform->cs_gpio_num, GPIO_PIN_TYPE_INPUT);
}

//base  就是指哪一个寄存器， 为spi0
//mode指相位，和极性   为0 
//data_size 指数据宽度  为8
static void config_hardware(void __iomem *base, unsigned speed, uint8_t mode, int data_size)
{
	 /* half_divisor = clock / (2*speed), rounded up: */
	unsigned half_divisor = (SPI_INPUT_CLOCK + (speed * 2 - 1)) / (speed*2);
	unsigned best_err = half_divisor;
	unsigned best_scr = SCR_MAX;
	unsigned best_half_cpsr = CPSR_MAX/2;
	unsigned scr, half_cpsr, err;

	unsigned polarity = (mode & SPI_CPOL);
	unsigned phase = (mode & SPI_CPHA);


	/* Loop over possible SCR values, calculating the appropriate CPSR and finding the best match
	 * For any SPI speeds above 260KHz, the first iteration will be it, and it will stop.
	 * The loop is left in for completeness */
	PRINTK(KERN_INFO "Setting up PL022 for: %dHz, mode %d, %d bits (target %d)\n",
	       speed, mode, data_size, half_divisor);

	for (scr = SCR_MIN; scr <= SCR_MAX; ++scr) {
		/* find the right cpsr (rounding up) for the given scr */
		half_cpsr = ((half_divisor + scr) / (1+scr));

		if (half_cpsr < CPSR_MIN/2)
			half_cpsr = CPSR_MIN/2;
		if (half_cpsr > CPSR_MAX/2)
			continue;

		err = ((1+scr) * half_cpsr) - half_divisor;

		if (err < best_err) {
			best_err = err;
			best_scr = scr;
			best_half_cpsr = half_cpsr;
			if (err == 0)
				break;
		}
	}

	PRINTK(KERN_INFO "Actual clock rate: %dHz\n", SPI_INPUT_CLOCK / (2 * best_half_cpsr * (1+best_scr)));

	PRINTK(KERN_INFO "Setting PL022 config: %08x %08x %08x\n",
		PL022_SCR_MAP(best_scr) | PL022_SPH_MAP(phase) | PL022_SPO_MAP(polarity) |
		PL022_FRF_SPI | PL022_DSS_MAP(data_size), 2, best_half_cpsr * 2);

	/* Set CR0 params */
	PL022_WRITE_REG(PL022_SCR_MAP(best_scr) | PL022_SPH_MAP(phase) | PL022_SPO_MAP(polarity) |
			PL022_FRF_SPI | PL022_DSS_MAP(data_size), base, PL022_CR0);

	/* Set prescale divisor */
	PL022_WRITE_REG(best_half_cpsr * 2, base, PL022_CPSR);

}

//反相引脚且设置为输出 spi_dev->master->dev.parent->platform_data->cs_gpio_num
void toggle_cs(struct spi_device *spi_dev, int *current_cs) {
	struct pl022_platform_data *pPlatform = spi_dev->master->dev.parent->platform_data;

	reg_gpio_set_pin(pPlatform->cs_gpio_num, !*current_cs);
	reg_gpio_iotr_set_pin_type(pPlatform->cs_gpio_num , GPIO_PIN_TYPE_OUTPUT);

	*current_cs = !*current_cs;
}

/* ================================================================================================
 * DMA Client callback
 * ================================================================================================
 */
static void dma_completion_callback (int channel, int status_mask, void *cb_data) {
	struct completion *completion = cb_data;
	complete(completion);
}


/* ================================================================================================
 *  Workqueue message processing
 * ================================================================================================
 */
//num_to_tx 要发送的字节数，
//写数据寄存器采用的是32位的方式
//master_data->cur_txfr ->tx_buf存放要发送的数据。
//master_data->cur_txfr_numwords_left_tx 发送完后，还剩余的字节数.
//master_data->cur_txfr_tx_idx   要发送的数据的索引 
static void load_spi_txfifo(int num_to_tx, struct pl022_platform_data *pPlatform,
					   struct pl022_master_device_data *master_data)
{
	struct spi_transfer *txfr = master_data->cur_txfr;

	const uint8_t *tx_buf_8 = txfr->tx_buf;
	const uint16_t *tx_buf_16 = txfr->tx_buf;

	int i;

	if (num_to_tx > master_data->cur_txfr_numwords_left_tx)
		num_to_tx = master_data->cur_txfr_numwords_left_tx;

	if (master_data->cur_txfr->bits_per_word <= 8) {
		/* bytes or smaller */
		for (i = 0; i < num_to_tx; ++i) {
			/* wait until txfifo is not full */
			while ((PL022_REG(pPlatform->base, PL022_SR) & PL022_SR_TNF) == 0);

			PL022_WRITE_REG(tx_buf_8 ? tx_buf_8[master_data->cur_txfr_tx_idx++] :
					0, pPlatform->base, PL022_DR);
		}
	} else {
		/* bigger words */
		for (i = 0; i < num_to_tx; ++i) {
			/* wait until txfifo is not full */
			while ((PL022_REG(pPlatform->base, PL022_SR) & PL022_SR_TNF) == 0);

			PL022_WRITE_REG(tx_buf_16 ? tx_buf_16[master_data->cur_txfr_tx_idx++] :
					0, pPlatform->base, PL022_DR);
		}
	}

	master_data->cur_txfr_numwords_left_tx -= num_to_tx;

	if (tx_buf_8) {
		PRINTK(KERN_INFO "Tx:");
		for (i = num_to_tx; i > 0; --i)
			PRINTK(" %02x", tx_buf_8[master_data->cur_txfr_tx_idx - i]);
		PRINTK("\n");
	} else {
		PRINTK(KERN_INFO "Tx blank(%d)\n", num_to_tx);
	}
}

//min_to_rx
// master_data->cur_txfr->rx_buf
//master_data->cur_txfr_rx_idx
static int unload_spi_rxfifo(int min_to_rx, struct pl022_platform_data *pPlatform,
					    struct pl022_master_device_data *master_data)
{
	struct spi_transfer *txfr = master_data->cur_txfr;

	uint8_t *rx_buf_8 = txfr->rx_buf;
	uint16_t *rx_buf_16 = txfr->rx_buf;

	int i;
 	volatile uint16_t vol_tmp;
	int num_rxd = 0;

	if (min_to_rx > master_data->cur_txfr_numwords_left_rx)
		min_to_rx = master_data->cur_txfr_numwords_left_rx;

	if (master_data->cur_txfr->bits_per_word <= 8) {
		do {
			while (PL022_REG(pPlatform->base, PL022_SR) & PL022_SR_RNE) {
				if (rx_buf_8) {
					rx_buf_8[master_data->cur_txfr_rx_idx++] =
						PL022_REG(pPlatform->base, PL022_DR);
				} else
					vol_tmp =  PL022_REG(pPlatform->base, PL022_DR);
				++num_rxd;
			}
		} while (num_rxd < min_to_rx);
	} else {
		/* bigger words */
		do {
			while (PL022_REG(pPlatform->base, PL022_SR) & PL022_SR_RNE) {
				if (rx_buf_16) {
					rx_buf_16[master_data->cur_txfr_rx_idx++] =
						PL022_REG(pPlatform->base, PL022_DR);
				} else
					vol_tmp =  PL022_REG(pPlatform->base, PL022_DR);
				++num_rxd;
			}
		} while (num_rxd < min_to_rx);
	}

	if (rx_buf_8) {
		PRINTK(KERN_INFO "Rx:");
		for (i = num_rxd; i > 0; --i)
			PRINTK(" %02x", rx_buf_8[master_data->cur_txfr_rx_idx - i]);
		PRINTK(" (%d)\n", master_data->cur_txfr_numwords_left_rx);
	}

	master_data->cur_txfr_numwords_left_rx -= num_rxd;

	return num_rxd;
}


//好像是初始化,但是最后一段没看懂.

static int perform_spi_transfer_poll(struct spi_device *spi_dev, struct spi_transfer *txfr) {
	struct pl022_platform_data *pPlatform = spi_dev->master->dev.parent->platform_data;
	struct pl022_master_device_data *master_data = dev_get_drvdata(&spi_dev->master->dev);

	unsigned speed = txfr->speed_hz;
	unsigned bits_per_word = txfr->bits_per_word;

	int num_this_time = SPI_TXFIFO_SIZE;

	PRINTK(KERN_INFO "PL022-SPI: perform_spi_transfer_poll(0x%p, 0x%p)\n", spi_dev, txfr);

	if (!speed)
		speed = spi_dev->max_speed_hz;
	if (!bits_per_word)
		bits_per_word = spi_dev->bits_per_word;
	if (!bits_per_word)
		bits_per_word = 8;

	config_hardware(pPlatform->base, speed, spi_dev->mode, bits_per_word);

	master_data->cur_txfr = txfr;
	master_data->cur_txfr_numwords_left_tx = (bits_per_word <= 8) ? txfr->len : (txfr->len / 2);
	master_data->cur_txfr_numwords_left_rx = master_data->cur_txfr_numwords_left_tx;
	master_data->cur_txfr_tx_idx = 0;
	master_data->cur_txfr_rx_idx = 0;

	while (master_data->cur_txfr_numwords_left_rx) {
		load_spi_txfifo(num_this_time, pPlatform, master_data);
		num_this_time = unload_spi_rxfifo(1, pPlatform, master_data);
	}

	return 0;
}

static int perform_spi_transfer_irq(struct spi_device *spi_dev, struct spi_transfer *txfr) {
	int timeout;
	struct pl022_platform_data *pPlatform = spi_dev->master->dev.parent->platform_data;
	struct pl022_master_device_data *master_data = dev_get_drvdata(&spi_dev->master->dev);

	unsigned speed = txfr->speed_hz;
	unsigned bits_per_word = txfr->bits_per_word;
	unsigned long flags;

	PRINTK(KERN_INFO "PL022-SPI: perform_spi_transfer_irq(0x%p, 0x%p)\n", spi_dev, txfr);

	if (!speed)
		speed = spi_dev->max_speed_hz;
	if (!bits_per_word)
		bits_per_word = spi_dev->bits_per_word;
	if (!bits_per_word)
		bits_per_word = 8;

	config_hardware(pPlatform->base, speed, spi_dev->mode, bits_per_word);

	master_data->cur_txfr = txfr;
	master_data->cur_txfr_numwords_left_tx = (bits_per_word <= 8) ? txfr->len : (txfr->len / 2);
	master_data->cur_txfr_numwords_left_rx = master_data->cur_txfr_numwords_left_tx;
	master_data->cur_txfr_tx_idx = 0;
	master_data->cur_txfr_rx_idx = 0;

	spin_lock_irqsave(&master_data->fifolock, flags);
	load_spi_txfifo(SPI_TXFIFO_SIZE, pPlatform, master_data);
	PL022_WRITE_REG(PL022_IRQ_TXW | PL022_IRQ_RXT | PL022_IRQ_RXO, pPlatform->base, PL022_IMSC);
	spin_unlock_irqrestore(&master_data->fifolock, flags);

	timeout = wait_for_completion_timeout(&master_data->isr_completion, HZ);

	if (timeout == 0) {
		printk(KERN_WARNING "PL022-SPI: timeout waiting for completion\n");
		return -EIO;
	}

	return 0;
}


/* Interrupt handler for RXFIFO full interrupt */
static irqreturn_t bcm5892_spi_int(int irq, void *dev_id) {
	struct device *pDev = dev_id;
	struct pl022_platform_data *pPlatform = pDev->platform_data;
	struct spi_master *pMaster = dev_get_drvdata(pDev);
	struct pl022_master_device_data *master_data = dev_get_drvdata(&pMaster->dev);
	unsigned long flags;
	int handled = 0;
	int num_read;

	uint32_t irqstatus = PL022_REG(pPlatform->base, PL022_MIS) & (PL022_IRQ_TXW | PL022_IRQ_RXT | PL022_IRQ_RXO);

	if (irqstatus) {
		spin_lock_irqsave(&master_data->fifolock, flags);

		num_read = unload_spi_rxfifo(0, pPlatform, master_data);
		load_spi_txfifo(num_read, pPlatform, master_data);

		spin_unlock_irqrestore(&master_data->fifolock, flags);

		if (master_data->cur_txfr_numwords_left_rx == 0) {
			PL022_WRITE_REG(0, pPlatform->base, PL022_IMSC);

			complete(&master_data->isr_completion);
		}
		handled = 1;
	}

	return IRQ_RETVAL(handled);
}

static int perform_spi_transfer_dma(struct spi_device *spi_dev, struct spi_transfer *txfr) {
	int rx_dma_chan, tx_dma_chan, swap, timeout;
	struct pl022_platform_data *pPlatform = spi_dev->master->dev.parent->platform_data;
	struct pl022_master_device_data *master_data = dev_get_drvdata(&spi_dev->master->dev);

	unsigned speed = txfr->speed_hz;
	unsigned bits_per_word = txfr->bits_per_word;
	dma_addr_t rx = txfr->rx_dma, tx = txfr->tx_dma;
	int remaining_length = txfr->len;
	int this_rx_length;

	PRINTK(KERN_INFO "PL022-SPI: perform_spi_transfer_dma(0x%p, 0x%p)\n", spi_dev, txfr);

	if (!speed)
		speed = spi_dev->max_speed_hz;
	if (!bits_per_word)
		bits_per_word = spi_dev->bits_per_word;
	if (!bits_per_word)
		bits_per_word = 8;

	config_hardware(pPlatform->base, speed, spi_dev->mode, bits_per_word);

	rx_dma_chan = pl080_request_channel(1);
	tx_dma_chan = pl080_request_channel(1);

	if (rx_dma_chan < 0 || tx_dma_chan < 0) {
		if (rx_dma_chan >= 0) {
			pl080_release_channel(rx_dma_chan);
		}
		if (tx_dma_chan >= 0) {
			pl080_release_channel(tx_dma_chan);
		}
		return -EMFILE;
	}

	if (rx_dma_chan > tx_dma_chan) {
		swap = rx_dma_chan;
		rx_dma_chan = tx_dma_chan;
		tx_dma_chan = swap;
	}

	PL022_WRITE_REG(PL022_DMACR_ENABLE, pPlatform->base, PL022_DMACR);

	pl080_channel_set_src_dest(tx_dma_chan, PL080_DMA_MEM, pPlatform->dma_tx_id);
	pl080_channel_set_src_dest(rx_dma_chan, pPlatform->dma_rx_id, PL080_DMA_MEM);
	pl080_channel_set_irq_callback(rx_dma_chan, dma_completion_callback, &master_data->isr_completion);

	/* We cannot simply set up chained transfers for both rx and tx: when the rx transfer needs to follow
	 * an LLI link it will stall briefly, and if the tx does not stall at the same time, rx might overflow.
	 * So do the largest single transfer we can for the rx, and then do the same size transfer for tx.
	 */
	while (remaining_length > 0) {
		this_rx_length = pl080_channel_add_xfer_stage(
			rx_dma_chan, (dma_addr_t)pPlatform->base + PL022_DR,
			rx, remaining_length, PL080_CCTL_DI | PL080_CCTL_TCI | PL080_CCTL_DWIDTH32 |
			                      ((bits_per_word > 8) ? PL080_CCTL_SWIDTH16 : PL080_CCTL_SWIDTH8)
			);

		/* Keep TX width at 1 byte, to avoid TXFIFO overflows */
		pl080_channel_add_xfer(tx_dma_chan, tx,
				(dma_addr_t)pPlatform->base + PL022_DR, this_rx_length,
				PL080_CCTL_SI | ((bits_per_word > 8) ? PL080_CCTL_DWIDTH16 : PL080_CCTL_DWIDTH8) );

		pl080_channel_enable(rx_dma_chan);
		pl080_channel_enable(tx_dma_chan);

		timeout = wait_for_completion_timeout(&master_data->isr_completion, 5 * HZ);

		if (timeout == 0) {
			printk(KERN_WARNING "PL022-SPI: Timeout: rx %p tx %p, len %d (%d)\n",
			       txfr->rx_buf, txfr->tx_buf, remaining_length, this_rx_length);
			return -EIO;
		}

		pl080_channel_clear_xfers(rx_dma_chan);
		pl080_channel_clear_xfers(tx_dma_chan);

		remaining_length -= this_rx_length;
		rx += this_rx_length;
		tx += this_rx_length;
	}

	pl080_release_channel(tx_dma_chan);
	pl080_release_channel(rx_dma_chan);
	PL022_WRITE_REG(PL022_DMACR_DISABLE, pPlatform->base, PL022_DMACR);

	return 0;
}

static int setup_dma_mapping(struct spi_message *msg, struct spi_transfer *txfr) {
	void * tx_buf = (void*)(txfr->tx_buf);

	PRINTK(KERN_INFO "Checking DMA mapping\n");

	if (msg->is_dma_mapped)
		return 1;

	/* For very short messages, don't bother with all the overhead: send it without DMA */
	if (txfr->len <= SPI_TXFIFO_SIZE)
		return 0;

	PRINTK(KERN_INFO "Message needs to be mapped.  Doing so...\n");

	if (txfr->tx_buf) {
		txfr->tx_dma = dma_map_single(&msg->spi->dev, tx_buf, txfr->len, DMA_TO_DEVICE);
		if (txfr->tx_dma == 0)
			return 0;
	} else {
		txfr->tx_dma = 0;
	}

	if (txfr->rx_buf) {
		txfr->rx_dma = dma_map_single(&msg->spi->dev, txfr->rx_buf, txfr->len, DMA_FROM_DEVICE);
		if (txfr->rx_dma == 0) {
			if (txfr->tx_dma)
				dma_unmap_single(&msg->spi->dev, txfr->tx_dma, txfr->len, DMA_TO_DEVICE);
			return 0;
		}
	} else {
		txfr->rx_dma = 0;
	}

	PRINTK(KERN_INFO "Successful mapping: %08x %08x\n", txfr->tx_dma, txfr->rx_dma);

	return 1;
}

static void teardown_dma_mapping(struct spi_message *msg, struct spi_transfer *txfr) {
	if (msg->is_dma_mapped)
		return;  /* not our job: someone else did the mapping */

	if (txfr->tx_dma)
		dma_unmap_single(&msg->spi->dev, txfr->tx_dma, txfr->len, DMA_TO_DEVICE);
	if (txfr->rx_dma)
		dma_unmap_single(&msg->spi->dev, txfr->rx_dma, txfr->len, DMA_TO_DEVICE);
}

static void do_work(struct work_struct *work)
{
	struct pl022_master_device_data *master_data = container_of(work, struct pl022_master_device_data, work);
	struct pl022_platform_data *pPlatform;
	unsigned long flags;
	struct spi_transfer *txfr = NULL;
	int cs_pin_state;

	PRINTK(KERN_INFO "PL022-SPI: do_work(0x%p)\n", work);

	spin_lock_irqsave(&master_data->lock, flags);

	while (!list_empty(&master_data->msg_queue)) {
		struct spi_message *msg = container_of(master_data->msg_queue.next, struct spi_message, queue);
		pPlatform = msg->spi->master->dev.parent->platform_data;

		list_del_init(&master_data->msg_queue);
		spin_unlock_irqrestore(&master_data->lock, flags);

		cs_pin_state = !(msg->spi->mode & SPI_CS_HIGH);	 /* CS pin already in this state from setup */

		PRINTK(KERN_INFO "PL022-SPI:	do_work() working msg 0x%p\n", msg);

		toggle_cs(msg->spi, &cs_pin_state);  /* tell slave that we're talking to it */

		list_for_each_entry(txfr, &msg->transfers, transfer_list) {
			PRINTK(KERN_INFO "PL022-SPI:	    do_work() working transfer 0x%p\n", txfr);

			if (txfr->len > 0) {
				msg->status = -EMFILE;  /* signal that DMA is not available */
				PRINTK(KERN_INFO "pPlatform->enable_dma = %d\n", pPlatform->enable_dma);
				if (pPlatform->enable_dma && setup_dma_mapping(msg, txfr)) {
					msg->status = perform_spi_transfer_dma(msg->spi, txfr);
					teardown_dma_mapping(msg, txfr);
				}

				if (msg->status == -EMFILE) {
					if (master_data->polled)
						msg->status = perform_spi_transfer_poll(msg->spi, txfr);
					else
						msg->status = perform_spi_transfer_irq(msg->spi, txfr);
				}

				if (msg->status == 0)
					msg->actual_length += txfr->len;
				else
					break;

			}

			if (txfr->delay_usecs)
				udelay(txfr->delay_usecs);

			if (txfr->cs_change) {
				toggle_cs(msg->spi, &cs_pin_state);
			}
		}

		if (! (cs_pin_state == !(msg->spi->mode & SPI_CS_HIGH)) )  /* pin is not in rest state */
			toggle_cs(msg->spi, &cs_pin_state);		   /* so toggle it back to rest */

		msg->complete(msg->context);

		spin_lock_irqsave(&master_data->lock, flags);
	}
	spin_unlock_irqrestore(&master_data->lock, flags);

	PRINTK(KERN_INFO "PL022-SPI: do_work() done.\n");
}

/* ================================================================================================
 *  Helper functions for sysfs interface
 * ================================================================================================
 */


static int enable_spi_device(struct device* pDev)
{
	struct amba_device *amba_dev = container_of(pDev, struct amba_device, dev);

	struct pl022_platform_data *pPlatform;
	struct spi_master *pMaster;
	struct pl022_master_device_data *master_data;
	int rc;

	printk(KERN_INFO "PL022-SPI:	Turn on SPI Master Controller\n");

	pPlatform = pDev->platform_data;

	/* Allocate the SPI Master Controller */
	pMaster = bcm5892_spi_alloc_master(pDev, sizeof(*pMaster));
	if (!pMaster) {
		dev_err(pDev, "PL022-SPI: enable failed, could not alloc spi_master.\n");
		return -ENOMEM;
	}
	master_data = dev_get_drvdata(&pMaster->dev);

	PRINTK(KERN_INFO "PL022-SPI:	 Master allocated... pMaster:0x%p master_data:0x%p\n", pMaster, master_data);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	master_data->wq = bcm5892_create_singlethread_workqueue(pDev->bus_id);
#else
	master_data->wq = bcm5892_create_singlethread_workqueue("bcmspilock");
#endif
	if (!master_data->wq) {
		dev_err(pDev, "PL022-SPI: enable failed, could not alloc workqueue.\n");
		goto dealloc_master;
	}

	INIT_WORK(&master_data->work, do_work);
	spin_lock_init(&master_data->lock);
	spin_lock_init(&master_data->fifolock);
	INIT_LIST_HEAD(&master_data->msg_queue);

	dev_set_drvdata(pDev, pMaster);
	master_data = dev_get_drvdata(&pMaster->dev);

	/* Setup the module state */
	master_data->errors = 0;

	if (amba_dev->irq[0] == NO_IRQ) {
		PRINTK(KERN_INFO "PL022-SPI: No IRQ found: using polling mode\n");
		master_data->polled = 1;
	} else {
		PRINTK(KERN_INFO "PL022-SPI: IRQ found(%d), using irq mode\n", amba_dev->irq[0]);
		master_data->polled = 0;
		rc = request_irq(amba_dev->irq[0], bcm5892_spi_int, 0, "spi", pDev);
		if (rc) {
			dev_err(pDev, "PL022-SPI: enable failed, could not allocate IRQ\n");
			goto free_workqueue;
		}

	}

	init_completion(&master_data->isr_completion);

	pMaster->num_chipselect = 1;
	pMaster->bus_num = pPlatform->bus_num;
	pMaster->setup	  = pl022_spi_setup;
	pMaster->transfer = pl022_spi_transfer;
	pMaster->cleanup  = pl022_spi_cleanup;

	enable_pins(pPlatform);

	/* Power on module */
	PL022_WRITE_REG(2, pPlatform->base, PL022_CR1);

	/* Adaptor will be active starting now */
	PRINTK(KERN_INFO "PL022-SPI:	spi_register_master()\n");
	if (bcm5892_spi_register_master(pMaster)) {
		dev_err(pDev, "PL022-SPI: Failed spi_register_master()\n");
		goto disable_pins;
	}

	PRINTK(KERN_INFO "PL022-SPI:	spi_master_get()\n");
	if (pMaster != bcm5892_spi_master_get(pMaster)) {
		dev_err(pDev, "PL022-SPI: Failed spi_master_get()\n");
		goto disable_pins;
	}

	pPlatform->is_enabled = true;

	PRINTK(KERN_INFO "PL022-SPI:	PL022 SPI master controller - ON\n");

	return 0;

disable_pins:
	disable_pins(pPlatform);
	PL022_WRITE_REG(0, pPlatform->base, PL022_CR1);  /* power off module */
	if (!master_data->polled) {
		PL022_WRITE_REG(0, pPlatform->base, PL022_IMSC);
		free_irq(amba_dev->irq[0], pDev);
	}
free_workqueue:
	bcm5892_destroy_workqueue(master_data->wq);
dealloc_master:
	bcm5892_spi_master_put(pMaster);

	return -ENOMEM;
}


static int disable_spi_device(struct device* pDev)
{
	struct amba_device *amba_dev = container_of(pDev, struct amba_device, dev);
	struct pl022_platform_data *pPlatform = pDev->platform_data;
	struct spi_master *pMaster = dev_get_drvdata(pDev);
	struct pl022_master_device_data *master_data = dev_get_drvdata(&pMaster->dev);

	printk(KERN_INFO "PL022-SPI:	Turn off SPI Master Controller\n");

	disable_pins(pPlatform);

	if (!master_data->polled) {
		PL022_WRITE_REG(0, pPlatform->base, PL022_IMSC);
		free_irq(amba_dev->irq[0], pDev);
	}

	PL022_WRITE_REG(0, pPlatform->base, PL022_CR1);  /* power off module */

	bcm5892_flush_workqueue(master_data->wq);

	PRINTK(KERN_INFO "PL022-SPI:	spi_unregister_master\n");
	bcm5892_spi_unregister_master(pMaster);

	bcm5892_destroy_workqueue(master_data->wq);

	PRINTK(KERN_INFO "PL022-SPI:	spi_master_put()\n");
	bcm5892_spi_master_put(pMaster);

	pPlatform->is_enabled = false;

	PRINTK(KERN_INFO "PL022-SPI:	PL022 SPI master controller - OFF\n");

	return 0;
}

/* ================================================================================================
 *  Module accessors (sysfs)
 * ================================================================================================
 */

static DEVICE_ATTR(enabled, S_IWUSR | S_IRUGO, show_enabled_sysfs, set_enabled_sysfs);
static DEVICE_ATTR(dma, S_IWUSR | S_IRUGO, show_dma_sysfs, set_dma_sysfs);
static DEVICE_ATTR(debug, S_IWUSR | S_IRUGO, show_debug_sysfs, set_debug_sysfs);


static ssize_t show_enabled_sysfs(struct device *pDev, struct device_attribute *attr, char *buf)
{
	struct pl022_platform_data *pPlatform = pDev->platform_data;

	PRINTK(KERN_INFO "PL022-SPI: show_enabled_sysfs(0x%p, 0x%p, 0x%p)\n", pDev, attr, buf);

	/* Fill provided buffer with value indicating SPI state */
	return sprintf(buf, "%d\n", pPlatform->is_enabled);
}


static ssize_t set_enabled_sysfs(struct device *pDev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pl022_platform_data *pPlatform = pDev->platform_data;
	unsigned set_enabled = simple_strtoul(buf, NULL, 10);

	PRINTK(KERN_INFO "PL022-SPI: set_enabled_sysfs(0x%p, 0x%p, 0x%p, %d)\n", pDev, attr, buf, count);

	/* If the master is disabled and you set enabled, then enable it... */
	if ((!pPlatform->is_enabled) && (set_enabled)) {
		enable_spi_device(pDev);
		return count;
	}

	/* If the Master is enabled and you unset enabled then disable it... */
	if ((pPlatform->is_enabled) && (!set_enabled)) {
		disable_spi_device(pDev);
		return count;
	}

	/* Otherwise just return because you are merely confirming the current state... */
	return count;
}

static ssize_t show_dma_sysfs(struct device *pDev, struct device_attribute *attr, char *buf)
{
	struct pl022_platform_data *pPlatform = pDev->platform_data;

	return sprintf(buf, "%d\n", pPlatform->enable_dma);
}

static ssize_t set_dma_sysfs(struct device *pDev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pl022_platform_data *pPlatform = pDev->platform_data;

	pPlatform->enable_dma = simple_strtoul(buf, NULL, 10);

	return count;
}

static ssize_t show_debug_sysfs(struct device *pDev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", debug_on);
}

static ssize_t set_debug_sysfs(struct device *pDev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned set_debug = simple_strtoul(buf, NULL, 10);

#if PROVIDE_LOOPBACK_TEST
	switch (set_debug) {
	case 0:
	case 1:
		debug_on = set_debug;
		break;
	default:
		pl022_spi_loopback_test(pDev, set_debug);
		break;
	}
#else
	debug_on = set_debug;
#endif

	return count;
}


/* ================================================================================================
 *  SPI Master Functions
 * ================================================================================================
 */


/* setup mode and clock, etc (spi driver may call many times) */
static int pl022_spi_setup (struct spi_device *pDev)
{
	struct spi_master *pMaster = pDev->master;
	struct pl022_platform_data *pPlatform = pMaster->dev.parent->platform_data;
	struct pl022_master_device_data *master_data = dev_get_drvdata(&pMaster->dev);
	int initial_cs_pin_state = 1;

	PRINTK(KERN_INFO "pl022_spi_setup(0x%p) mode: %04x, bits:%d, hz:%d, cs:%d\n",
		pDev, pDev->mode, pDev->bits_per_word, pDev->max_speed_hz, pDev->chip_select);

	master_data->last_spi_device = pDev;

	if (pDev->mode & ~MODEBITS) {
		PRINTK(KERN_WARNING "pl022_spi_setup bad modebits\n");
		return -EINVAL;
	}

	pDev->bits_per_word = pDev->bits_per_word ? pDev->bits_per_word : 8;
	if (pDev->bits_per_word < 1 || pDev->bits_per_word > 16) {
		PRINTK(KERN_WARNING "pl022_spi_setup bad bits_per_word\n");
		return -EINVAL;
	}

	if (pDev->max_speed_hz < MIN_SPEED_HZ) {
		PRINTK(KERN_WARNING "pl022_spi_setup device is too slow (max speed %d, our min speed is %d)\n",
			pDev->max_speed_hz, MIN_SPEED_HZ);
		return -EINVAL;
	}

	if (pDev->max_speed_hz > MAX_SPEED_HZ)
		pDev->max_speed_hz = MAX_SPEED_HZ;

	if (pDev->chip_select != 0) {
		PRINTK(KERN_WARNING "pl022_spi_setup bad chip select\n");
		return -EINVAL;
	}

	PRINTK(KERN_INFO "PL022-SPI:	configuring hw\n");

	if (pDev->mode & SPI_CS_HIGH) {
		/* this device wants an inverted CS.  So toggle CS, and give it a chance to settle */
		toggle_cs(pDev, &initial_cs_pin_state);
		udelay(1000000 / pDev->max_speed_hz + 1);
	}

	config_hardware(pPlatform->base, pDev->max_speed_hz, pDev->mode, pDev->bits_per_word);

	PRINTK(KERN_INFO "PL022-SPI: pl022_spi_setup() done.\n");

	return 0;
}


/* bidirectional bulk transfers
 *
 * + The transfer() method may not sleep; its main role is
 *   just to add the message to the queue.
 * + For now there's no remove-from-queue operation, or
 *   any other request management
 * + To a given spi_device, message queueing is pure fifo
 *
 * + The master's main job is to process its message queue,
 *   selecting a chip then transferring data
 * + If there are multiple spi_device children, the i/o queue
 *   arbitration algorithm is unspecified (round robin, fifo,
 *   priority, reservations, preemption, etc)
 *
 * + Chipselect stays active during the entire message
 *   (unless modified by spi_transfer.cs_change != 0).
 * + The message transfers use clock and SPI mode parameters
 *   previously established by setup() for this device
 */
static int pl022_spi_transfer (struct spi_device *pDev, struct spi_message *pMsg)
{
	struct spi_master *pMaster = pDev->master;
	//struct pl022_platform_data *pPlatform = pMaster->dev.parent->platform_data;
	struct pl022_master_device_data *master_data = dev_get_drvdata(&pMaster->dev);
	struct spi_transfer *pTransfer = NULL;
	unsigned long flags;

	PRINTK( KERN_INFO "PL022-SPI: pl022_spi_transfer(0x%p, 0x%p)\n", pDev, pMsg);

	pMsg->actual_length = 0;

	/* check each transfer's parameters */
	list_for_each_entry (pTransfer, &pMsg->transfers, transfer_list) {
		u32 speed_hz = FIRST_NONZERO(pTransfer->speed_hz, pDev->max_speed_hz, MAX_SPEED_HZ);
		u8 bits_per_word = FIRST_NONZERO(pTransfer->bits_per_word, pDev->bits_per_word, 8);

		PRINTK(KERN_INFO "PL022-SPI:   Examining transfer item: %d %d\n", speed_hz, bits_per_word);

		if (!pTransfer->tx_buf && !pTransfer->rx_buf && pTransfer->len)
			return -EINVAL;
		if (bits_per_word < 1 || bits_per_word > 16)
			return -EINVAL;
		if (bits_per_word > 8 && (pTransfer->len & 1)) {
			PRINTK(KERN_WARNING "PL022-SPI: Odd number of bytes when bits_per_word > 8\n");
			return -EINVAL;
		}
		if (speed_hz < MIN_SPEED_HZ || speed_hz > MAX_SPEED_HZ) {
			PRINTK(KERN_WARNING "PL022-SPI: Transfer with bad speed\n");
			return -EINVAL;
		}
	}

	PRINTK(KERN_INFO "PL022-SPI:	Adding to queue\n");

	spin_lock_irqsave(&master_data->lock, flags);
	list_add_tail(&pMsg->queue, &master_data->msg_queue);
	bcm5892_queue_work(master_data->wq, &master_data->work);
	spin_unlock_irqrestore(&master_data->lock, flags);

	PRINTK( KERN_INFO "PL022-SPI: pl022_spi_transfer() done.\n");

	return 0;
}


/* called on release() to free memory provided by spi_master */
static void pl022_spi_cleanup (struct spi_device *pDev)
{
	PRINTK(KERN_INFO "PL022-SPI: pl022_spi_cleanup(0x%p) (empty)\n", pDev);
}

#if PROVIDE_LOOPBACK_TEST

void setup_test_buf(uint8_t *buf, int offset, int len, int size)
{
	int i;
	for (i = 0; i < size; ++i) {
		if (i < offset || i >= offset+len)
			buf[i] = 0xff;
		else
			buf[i] = ((i-offset) & 0x7f) + 1;
			//buf[i] = i;
			
	}
}

int check_test_buf(uint8_t *buf, int offset, int len, int size)
{
	unsigned i, out_min, out_max;
	uint8_t expected;
	for (i = 0; i < size; ++i) {
		if (i < offset || i >= offset+len)
			expected = 0xa5;
		else
			expected = ((i-offset) & 0x7f) + 1;

		if (buf[i] != expected) {
			printk(KERN_WARNING "bad buf: %02x at pos %d, should be %02x\n", buf[i], i, expected);
			out_min = (i > 2) ? i - 2 : 0;
			out_max = (i < size - 8 ? i + 8 : size);
			printk(KERN_WARNING "  ");
			for (i = out_min; i < out_max; ++i) {
				printk("%02x ", buf[i]);
			}
			printk("\n" KERN_WARNING "  ");
			for (i = out_min; i < out_max; ++i) {
				if (i < offset || i >= offset+len)
					expected = 0xa5;
				else
					expected = ((i-offset) & 0x7f) + 1;
				printk("%02x ", expected);
			}
			printk("\n");
			return 0;
		}
	}
	return 1;
}

/* Called when a loopback transfer is done */
static void pl022_spi_loopback_test_complete(void *context) {
	struct completion *c = context;
	complete(c);
}

/* Test function: sets loopback, and performs a variety of transfers */
static void pl022_spi_loopback_test (struct device *pDev, int startpos) {
	struct pl022_platform_data *pPlatform = pDev->platform_data;
	struct spi_master *pMaster = dev_get_drvdata(pDev);
	struct pl022_master_device_data *master_data = dev_get_drvdata(&pMaster->dev);
	struct spi_device *spi_dev = master_data->last_spi_device;
    unsigned int i=0;
	
	uint8_t *tx_buf = kzalloc(TEST_BUF_SIZE, GFP_KERNEL);
	uint8_t *rx_buf = kzalloc(TEST_BUF_SIZE, GFP_KERNEL);
	int tx_offset, rx_offset, len;

	struct completion test_completion;

	struct spi_transfer txfr = {
		.tx_buf = &tx_buf,
		.rx_buf = &rx_buf,
		.cs_change = 0,
		.bits_per_word = 8,
		.speed_hz = 11000000,
		.delay_usecs = 0,
	};

	struct spi_message msg;
	uint32_t status_reg, irq_reg;

	init_completion(&test_completion);

	PL022_WRITE_REG(3, pPlatform->base, PL022_CR1);  /* set loopback mode */

	status_reg = PL022_REG(pPlatform->base, PL022_SR);  irq_reg = PL022_REG(pPlatform->base, PL022_RIS);
	PRINTK(KERN_INFO "spi status %08x %08x\n", status_reg, irq_reg);

	spi_message_init(&msg);
	msg.spi = spi_dev;
	msg.complete = pl022_spi_loopback_test_complete;
	msg.context = &test_completion;
	spi_message_add_tail(&txfr, &msg);
	printk("spi_test_begin\n " );
	for (tx_offset = 0; tx_offset < 4; ++tx_offset) {
		for (rx_offset = 0; rx_offset < 4; ++rx_offset) {
			for (len = startpos; len < TEST_BUF_SIZE - ((tx_offset > rx_offset)? tx_offset : rx_offset); len++) {

				msg.actual_length = 0;
				msg.status = 0;
				txfr.tx_buf = tx_buf + tx_offset;
				txfr.rx_buf = rx_buf + rx_offset;
				txfr.len = len;

				setup_test_buf(tx_buf, tx_offset, len, TEST_BUF_SIZE);
				memset(rx_buf, 0xa5, TEST_BUF_SIZE);
				pl022_spi_transfer(spi_dev, &msg);
				wait_for_completion(&test_completion);
				if (!check_test_buf(rx_buf, rx_offset, len, TEST_BUF_SIZE)) {
					printk(KERN_WARNING "Failed at %d %d %d\n", tx_offset, rx_offset, len);

					status_reg = PL022_REG(pPlatform->base, PL022_SR);
					irq_reg = PL022_REG(pPlatform->base, PL022_RIS);

					PRINTK(KERN_INFO "spi status %08x %08x\n", status_reg, irq_reg);
					goto exit_loopback;
				}
				if (msg.status != 0)
					printk(KERN_WARNING "Bad status at %d %d %d (%d)\n",
					       tx_offset, rx_offset, len, msg.status);
				if (msg.actual_length != len)
					printk(KERN_WARNING "Bad length at %d %d %d (%d)\n",
					       tx_offset, rx_offset, len, msg.actual_length);
			    i++;
				printk("i:%x",i);
			}
		}
	}

exit_loopback:
	kfree(rx_buf);
	kfree(tx_buf);
	PL022_WRITE_REG(2, pPlatform->base, PL022_CR1);  /* unset loopback mode */
	printk(KERN_INFO "Completed loopback test\n");
}

#endif /* PROVIDE_LOOPBACK_TEST */

/* ================================================================================================ */
/*  AMBA Functions */
/* ================================================================================================ */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
static int pl022_amba_probe(struct amba_device *pDev, void *id)
#else
static int pl022_amba_probe(struct amba_device *pDev, struct amba_id *id)
#endif
{
	struct pl022_platform_data *pPlatform;

	int retval = 0;

	printk(KERN_INFO "PL022-SPI: pl022_amba_probe(pDev:%p, id:%p)\n", pDev, id);

	/* Request the AMBA memory region */
	retval = amba_request_regions(pDev, NULL);
	if (retval) {
		dev_err(&pDev->dev, "PL022-SPI: amba_request_regions(%p, NULL) failed\n", pDev);
		return retval;
	}

	/* We won't actually activate the device here, since user might not use all of them. */
	/* Instead, we set up sysfs hooks so user can enable the port */
	PRINTK(KERN_INFO "PL022-SPI:	device_create_file()\n");
	retval = bcm5892_device_create_file(&pDev->dev, &dev_attr_enabled);
	if (retval) {
		dev_err(&pDev->dev, "PL022-SPI: device_create_file(0x%p, 0x%p) failed\n",
			&pDev->dev, &dev_attr_enabled);
		goto err_release_regions;
	}

	retval = bcm5892_device_create_file(&pDev->dev, &dev_attr_dma);
	if (retval) {
		dev_err(&pDev->dev, "PL022-SPI: device_create_file(0x%p, 0x%p) failed\n",
			&pDev->dev, &dev_attr_dma);
		goto err_remove_enabled_file;
	}

	retval = bcm5892_device_create_file(&pDev->dev, &dev_attr_debug);
	if (retval) {
		dev_err(&pDev->dev, "PL022-SPI: device_create_file(0x%p, 0x%p) failed\n",
			&pDev->dev, &dev_attr_debug);
		goto err_remove_dma_file;
	}

	pPlatform = kzalloc(sizeof(struct pl022_master_device_data), GFP_KERNEL);
	if (!pPlatform) {
		dev_err(&pDev->dev, "PL022-SPI: set_speed failed, could not alloc spi_master.\n");
		retval = -ENOMEM;
		goto err_remove_debug_file;
	}

	PRINTK(KERN_INFO "PL022-SPI:	Platform data allocated... pPlatform:0x%p\n", pPlatform);
	pDev->dev.platform_data = pPlatform;

	pPlatform->base = (uint8_t* __iomem) (pDev->res.start);

	PRINTK(KERN_INFO "PL022-SPI:	iobase %p\n", pPlatform->base);

	switch (pDev->res.start) {
	case SPI0_REG_BASE_ADDR:
		pPlatform->bus_num = 0;
		pPlatform->cs_gpio_num = 7;
		pPlatform->dma_tx_id = PL080_DMA_SPI0_TX;
		pPlatform->dma_rx_id = PL080_DMA_SPI0_RX;
		break;
	case SPI1_REG_BASE_ADDR:
		pPlatform->bus_num = 1;
		pPlatform->cs_gpio_num = 11;
		pPlatform->dma_tx_id = PL080_DMA_SPI1_TX;
		pPlatform->dma_rx_id = PL080_DMA_SPI1_RX;
		break;
	case SPI3_REG_BASE_ADDR:
		pPlatform->bus_num = 3;
		pPlatform->cs_gpio_num = HW_GPIO1_PIN_MAX + 29;
		pPlatform->dma_tx_id = PL080_DMA_SPI3_TX;
		pPlatform->dma_rx_id = PL080_DMA_SPI3_RX;
		break;
	default:
		dev_err(&pDev->dev, "PL022-SPI: unknown bus_number.\n");
		retval = -ENOENT;
		goto err_release_platform;
	}

	PRINTK(KERN_INFO "PL022-SPI: pl022_amba_prob() done\n");

	return retval;

	/* Error Handling... */
err_release_platform:
	kfree(pPlatform);
err_remove_debug_file:
	bcm5892_device_remove_file(&pDev->dev, &dev_attr_debug);
err_remove_dma_file:
	bcm5892_device_remove_file(&pDev->dev, &dev_attr_dma);
err_remove_enabled_file:
	bcm5892_device_remove_file(&pDev->dev, &dev_attr_enabled);
err_release_regions:
	amba_release_regions(pDev);
	return retval;
}


static int pl022_amba_remove(struct amba_device *pDev)
{
	struct pl022_platform_data *pPlatform;

	printk(KERN_INFO "PL022-SPI: pl022_amba_remove(0x%p)\n", pDev);

	pPlatform = (pDev) ? (struct pl022_platform_data *) pDev->dev.platform_data : NULL;

	/* If the Master is enabled then down it... */
	if (pPlatform && pPlatform->is_enabled) {
		disable_spi_device(&pDev->dev);
	}

	bcm5892_device_remove_file(&pDev->dev, &dev_attr_debug);
	bcm5892_device_remove_file(&pDev->dev, &dev_attr_dma);
	bcm5892_device_remove_file(&pDev->dev, &dev_attr_enabled);

	pDev->dev.platform_data = NULL;
	kfree(pPlatform);

	PRINTK(KERN_INFO "PL022-SPI:	amba_release_regions(%p)\n", pDev);
	amba_release_regions(pDev);

	PRINTK(KERN_INFO "PL022-SPI: pl022_amba_remove() done.\n");

	return 0;
}


/* ================================================================================================
 *  Module Functions & Macros
 * ================================================================================================
 */


static int __init pl022_spi_init(void)
{
	int retval;

	printk(KERN_INFO "PL022-SPI: pl022_spi_init()\n");

	PRINTK(KERN_INFO "PL022-SPI:	amba_driver_register(%p)\n", &pl022_amba_driver);
	if ((retval = amba_driver_register(&pl022_amba_driver))) {
		PRINTK(KERN_INFO "PL022-SPI: SPI AMBA driver register failed\n");
		return retval;
	}

	PRINTK(KERN_INFO "PL022-SPI: Module loaded.\n");

	return 0;
}
module_init(pl022_spi_init);


static void __exit pl022_spi_exit(void)
{
	printk(KERN_INFO "PL022-SPI: pl022_spi_exit()\n");

	amba_driver_unregister(&pl022_amba_driver);

	PRINTK(KERN_INFO "PL022-SPI: pl022_spi_exit() done.\n");
}
module_exit(pl022_spi_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("PL022 SPI hardware driver for BCM5892");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bcm5892-spi");
