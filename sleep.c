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

#include "sleep.h"
#include "main.h"
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

	//周期计数
	tcon |=  (1<<6);	

	//不要中断
	tcon &=  ~(1<<5);

	//32bit计数器
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

	//启动
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

	//停止
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


 void quck_ssp_setup(void)
{
	unsigned int  tcon;
	unsigned long reg_addr;
	//SSPCPSR  ,Must be an even number from 2 to 254,
	reg_addr = IO_ADDRESS(SPI0_REG_BASE_ADDR+0x0) ;	
	tcon =14; //__raw_readl(reg_addr);//分频系数
	__raw_writel(tcon, reg_addr);
	
         //SSPCR0
	reg_addr = IO_ADDRESS(SPI0_REG_BASE_ADDR+0x0) ;	
	tcon =0; //__raw_readl(reg_addr);
	// 2 = National Semiconductor MICROWIRE frame format 
	tcon |=  (1<<4);	// 01 = Texas Instruments synchronous serial frame
	tcon |=  (7<<0);//0111 = 8-bit data 
	__raw_writel(tcon, reg_addr);
	//SSPCR1 
	reg_addr = IO_ADDRESS(SPI0_REG_BASE_ADDR+0x4) ;	
	//Master mode  , 0 = SSP operation disabled  ,close loop func,
	tcon =0; //
	__raw_writel(tcon, reg_addr);

	//关闭中断
	//reg_addr = IO_ADDRESS(SPI0_REG_BASE_ADDR+0x14) ;	
	//Master mode  , 0 = SSP operation disabled  ,close loop func,
	//tcon =0; //
	//__raw_writel(tcon, reg_addr);	
/*#define RF_CS    BCM5892_GPA7  
#define RF_MISO    BCM5892_GPA5    
#define RF_MOSI  BCM5892_GPA6
#define RF_SCL     BCM5892_GPA4
#define RF_POWER     BCM5892_GPB31*/
	//io
	//FSS 引脚我自己通过io口控制。
	reg_gpio_iotr_set_pin_type(BCM5892_GPA7,GPIO_PIN_TYPE_ALTERNATIVE_FUNC0);
	reg_gpio_iotr_set_pin_type(BCM5892_GPA6,GPIO_PIN_TYPE_ALTERNATIVE_FUNC0);
	reg_gpio_iotr_set_pin_type(BCM5892_GPA5,GPIO_PIN_TYPE_ALTERNATIVE_FUNC0);	
	reg_gpio_iotr_set_pin_type(BCM5892_GPA4,GPIO_PIN_TYPE_ALTERNATIVE_FUNC0);

	return;
}

 void quck_ssp_start(void)
{
	unsigned long tcon;
	unsigned long reg_addr;

	//SSPCR1 
	reg_addr = IO_ADDRESS(SPI0_REG_BASE_ADDR+0x4) ;	
	tcon=__raw_readl(reg_addr);
	// 1 = SSP operation disabled 
	tcon |=  (1<<1);	//启动
	__raw_writel(tcon, reg_addr);

	return;
}

 void quck_ssp_stop(void)
{
	unsigned long tcon;
	unsigned long reg_addr;

	//SSPCR1 
	reg_addr = IO_ADDRESS(SPI0_REG_BASE_ADDR+0x4) ;	
	tcon=__raw_readl(reg_addr);
	// 1 = SSP operation disabled 
	tcon &= ~ (1<<1);	//不启动
	__raw_writel(tcon, reg_addr);

	return;
}

//0成功  ,其他失败.
unsigned char  ssp_Write_Byte(unsigned char d)
{
	unsigned long tcon;
	unsigned long reg_addr;

	//SSPSR
	reg_addr = IO_ADDRESS(SPI0_REG_BASE_ADDR+0xC) ;	
	//不是满的就写.
	while(!(__raw_readl(reg_addr) && 0x2 ));//wait until fifo is not full.
	//SSPDR
	reg_addr = IO_ADDRESS(SPI0_REG_BASE_ADDR+0x8) ;	
	__raw_writeb(d, reg_addr);	
	//等待发送完
	reg_addr = IO_ADDRESS(SPI0_REG_BASE_ADDR+0xC) ;		//SSPSR
	while(!(__raw_readl(reg_addr) && 0x1 ));
	return 0;

}

unsigned char  ssp_Read_Byte(void)
{
	unsigned long tcon;
	unsigned long reg_addr;

	//SSPSR
	reg_addr = IO_ADDRESS(SPI0_REG_BASE_ADDR+0xC) ;	
	//有数据就读,没有就等
	while(!(__raw_readl(reg_addr) && 0x4 ));
	//SSPDR
	reg_addr = IO_ADDRESS(SPI0_REG_BASE_ADDR+0x8) ;	
	return __raw_readb( reg_addr);	


}
//0成功  ,其他失败.
unsigned char  quck_ssp_read_printk(u8 *buf,u8 count)
{
	int ret,iNum                      = 0;

	
	if ( !count )
	{
		return 0;
	}

//	Spi_Select();
	ret = ssp_Write_Byte( buf[ 0 ] );
	if ( ret )
	{
		return 2;
	}
	
	for ( iNum = 0;iNum < count; iNum++ )
	{
		buf[ iNum+1 ] = ssp_Read_Byte( );
	}
	
//	Spi_Deselect();
	//local_irq_restore(flags);
	
	return 0;	
}
//0成功  ,其他失败.
//static ssize_t Spi_rfid_write(struct file *filp, const char *buf, u32 count, loff_t *f_pos)
u8 quck_ssp_write_printk( u8 * buf ,u8 count )
{

	int ret , iNum = 0;

	if ( !count )
	{
		return 0;	
	}
	
	/*if ( copy_from_user( Snd_data_buf, buf, count ) )
	{
		printk("get user data from user failed.\n");
		return  -EFAULT;
	}*/
	//memcpy(Snd_data_buf, buf, count );
	//local_irq_save(flags);
//	Spi_Select();
	for ( iNum = 0;iNum < count; iNum++ )
	{
		ret = ssp_Write_Byte( buf[ iNum ]);
		if ( ret )
		{
			return 2;
		}
	}
	
//	Spi_Deselect();
	//local_irq_restore(flags);
	
	return 0;

}

