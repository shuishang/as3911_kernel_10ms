
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


#include "As3911_api.h"
#include "emv_main.h"
#include "emv_picc.h"
#include "As3911_def.h"
int md_fd                             = 0;

const char version[]={ "as3911_src_v2[ "__TIME__"]\r\n"};
int main(int argc, char **argv)
{

	AS3911_init();

	printk((char*)version);

	 emvGuiDigital();
	 while(1);
		
}


