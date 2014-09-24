#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/sysctl.h>
#include <linux/ioport.h>
#include <mach/shm.h>
#include <linux/cdev.h>
#include <linux/kthread.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sizes.h>
#include <asm/uaccess.h>
#include <mach/reg_gpio.h>

#include <linux/broadcom/bcm_sysctl.h>
#include <linux/broadcom/timer.h>
#include <linux/spi/spi.h>

#include "sleep.h"
#include "main.h"

#define PRINTK   printk

//measure_counter
//timer 3.2  
//167ns /one tick
//715 s /one cycle
 void measure_counter_setup(void)
{
	unsigned long tcon;
	unsigned long reg_addr;

	reg_addr = IO_ADDRESS(TIM3_REG_BASE_ADDR+0x28) ;	
	tcon = __raw_readl(reg_addr);

	//���ڼ���
	tcon |=  (1<<6);	

	//��Ҫ�ж�
	tcon &=  ~(1<<5);

	//32bit������
	tcon |=  (1<<1);	

	//	printk("tcon :0x%x\n",tcon);
	__raw_writel(tcon, reg_addr);	

	reg_addr = IO_ADDRESS(TIM3_REG_BASE_ADDR+0x20) ;	

	//	tmp = OUPUT_CYCLE * TIMER_CLK /1000;
	//	printk("counter :%d\n",tmp);

	__raw_writel(0xffffffff, reg_addr);	
}

 void measure_counter_start(void)
{
	unsigned long tcon;
	unsigned long reg_addr;

	reg_addr = IO_ADDRESS(TIM3_REG_BASE_ADDR+0x28) ;	

	tcon = __raw_readl(reg_addr);

	//����
	tcon |=  (1<<7);	

	__raw_writel(tcon, reg_addr);	

	return;
}

 void measure_counter_stop(void)
{
	unsigned long tcon;
	unsigned long reg_addr;

	reg_addr = IO_ADDRESS(TIM3_REG_BASE_ADDR+0x28) ;	

	tcon = __raw_readl(reg_addr);

	//ֹͣ
	tcon &=  ~(1<<7);	

	__raw_writel(tcon, reg_addr);

	return;
}

unsigned int get_timer_count(void)
{

	return *((unsigned int *)(IO_ADDRESS(TIM3_REG_BASE_ADDR+0x24)));
}
u32 g_timer_count=0;

void quck_timer_count(u8 flag)
{
	u32 temp;
	temp=*((unsigned int *)(IO_ADDRESS(TIM3_REG_BASE_ADDR+0x24)));
//	temp=(temp*168)/1000000;
//	printk("%xms",(temp*168)/1000000);
	printk("%d:%dus ",flag,((g_timer_count-temp)*168)/1000);
	g_timer_count=temp;
	
}


//rst_end_count = *((unsigned int *)(IO_ADDRESS(TIM3_REG_BASE_ADDR+0x24)));
//ts_start_count = *((unsigned int *)(IO_ADDRESS(TIM3_REG_BASE_ADDR+0x24)));;
//------------------------------------------------------------------------
//---------------------------------------------------------------------


 //base  ����ָ��һ���Ĵ����� Ϊspi0
 //modeָ��λ���ͼ���	Ϊ0 
 //data_size ָ���ݿ��  Ϊ8
 //��Ҫ������CR0��CPSR, Ŀ����,֡��ʽ,λ��Ϊ8λ,��Ƶϵ��.
 #if 1
 static void config_hardware(void __iomem *base, unsigned speed, uint8_t mode, int data_size)
 {
	  /* half_divisor = clock / (2*speed), rounded up: */
	  printk("speed:%d, mode:%d,bits_per_word:%d  ",speed,mode,data_size);
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
		 PL022_FRF_TI | PL022_DSS_MAP(data_size), 2, best_half_cpsr * 2);
 
	 /* Set CR0 params */
	 PL022_WRITE_REG(PL022_SCR_MAP(best_scr) | PL022_SPH_MAP(phase) | PL022_SPO_MAP(polarity) |
			 PL022_FRF_TI | PL022_DSS_MAP(data_size), base, PL022_CR0);
 
	 /* Set prescale divisor */
	 PL022_WRITE_REG(best_half_cpsr * 2, base, PL022_CPSR);
	 PRINTK("best_half_cpsr* 2:%x \n",best_half_cpsr* 2);
 
 }
#endif
 void quck_ssp_setup(void)
{

	//__raw_writel(tcon, reg_addr);	
/*#define RF_CS    BCM5892_GPA7  
#define RF_MISO    BCM5892_GPA5    
#define RF_MOSI  BCM5892_GPA6
#define RF_SCL     BCM5892_GPA4
#define RF_POWER     BCM5892_GPB31*/
	//io
	//FSS �������Լ�ͨ��io�ڿ��ơ�
	//reg_gpio_iotr_set_pin_type(BCM5892_GPA7,GPIO_PIN_TYPE_ALTERNATIVE_FUNC0);
	reg_gpio_iotr_set_pin_type(BCM5892_GPA6,GPIO_PIN_TYPE_ALTERNATIVE_FUNC0);
	reg_gpio_iotr_set_pin_type(BCM5892_GPA5,GPIO_PIN_TYPE_ALTERNATIVE_FUNC0);	
	reg_gpio_iotr_set_pin_type(BCM5892_GPA4,GPIO_PIN_TYPE_ALTERNATIVE_FUNC0);
	config_hardware(SPI0_REG_BASE_ADDR,4000000,0,8);
	return;
}

void quck_ssp_start(void)
{
	//SSPCR1 
	PL022_WRITE_REG(3, SPI0_REG_BASE_ADDR, PL022_CR1);  /* start and set loopback mode */
	//PL022_WRITE_REG(2, SPI0_REG_BASE_ADDR, PL022_CR1);  /* start */
	return;
}

 void quck_ssp_stop(void)
{
	//PL022_WRITE_REG(3, SPI0_REG_BASE_ADDR, PL022_CR1);  /* start and set loopback mode /*/
	PL022_WRITE_REG(0, SPI0_REG_BASE_ADDR, PL022_CR1);  /* start */
	return;
}

//0�ɹ�  ,����ʧ��.
void  ssp_Write_Bytes(unsigned char *tx_buf_8,int num_to_tx)
{
	int i;

	for (i = 0; i < num_to_tx;i++) {
		/* wait until txfifo is not full */
		while ((PL022_REG(SPI0_REG_BASE_ADDR, PL022_SR) & PL022_SR_TNF) == 0);
	
		PL022_WRITE_REG(tx_buf_8[i], SPI0_REG_BASE_ADDR, PL022_DR);
	}


#if 1
	PRINTK(KERN_INFO "Tx:");
	for (i =0 ; i<num_to_tx ; ++i)
		PRINTK(" %02x", tx_buf_8[i]);
	PRINTK("\n");

#endif
	return ;
}

unsigned char  ssp_Read_Bytes(unsigned char *rx_buf_8,int num_rxd )
{
	int i=0;
#if 1
PRINTK(KERN_INFO "Rx:");	
#endif	
	do {
		while (PL022_REG(SPI0_REG_BASE_ADDR, PL022_SR) & PL022_SR_RNE) {
		
			rx_buf_8[i++] =PL022_REG(SPI0_REG_BASE_ADDR, PL022_DR);

		}
	} while ( i<1 );	
	//} while ( i<num_rxd );

#if 1
	
	for (i = 0 ; i <num_rxd;++i)
		PRINTK(" %02x", rx_buf_8[i]);
	PRINTK(" (%d)\n", num_rxd);

#endif
	return 1;
}
//0�ɹ�  ,����ʧ��.
unsigned char  quck_ssp_read_printk(u8 *buf,u8 count)
{
	//int ret,iNum                      = 0;
	if ( !count )
	{
		printk("error -quck_ssp_read_printk\n");
		return 0;
	}

	Spi_Select();
	ssp_Write_Bytes( &buf[ 0 ] ,1);
	ssp_Read_Bytes(&buf[1],count);
	Spi_Deselect();
	//local_irq_restore(flags);
	return 0;	
}
//0�ɹ�  ,����ʧ��.
//static ssize_t Spi_rfid_write(struct file *filp, const char *buf, u32 count, loff_t *f_pos)
u8 quck_ssp_write_printk( u8 * buf ,u8 count )
{

	if ( !count )
	{
		printk("error -quck_ssp_read_printk\n");
		return 0;
	}
	//local_irq_save(flags);
	Spi_Select();
	ssp_Write_Bytes( &buf[ 0 ],count);
	Spi_Deselect();
	//local_irq_restore(flags);
	return 0;

}

