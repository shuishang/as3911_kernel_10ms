#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h> 
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/fcntl.h> 
#include <linux/proc_fs.h>  
#include <linux/fs.h>   
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/sched.h>  /* current and everything */
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <asm/hardware.h>
#include <asm/uaccess.h> /**copy_to_user()**/
#include <asm/io.h>
#include <asm/atomic.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-gpioj.h>
#include <asm/arch/map.h>

#include "sleep.h"




void sleepMilliseconds(unsigned int milliseconds)
{
	//udelay(milliseconds*1000);
	mdelay(milliseconds);
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
