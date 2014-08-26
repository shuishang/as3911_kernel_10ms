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


