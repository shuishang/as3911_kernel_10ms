

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/fcntl.h> 
#include <linux/fs.h>   
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>

//#include <asm/hardware.h>
#include <asm/uaccess.h> 
#include <asm/io.h>

#include <asm/io.h>
//#include <asm/arch/regs-gpio.h>
//#include <asm/arch/regs-gpioj.h>
#include <asm/atomic.h>
//#include <asm/arch/map.h>
//#include <asm/arch/regs-irq.h>
//#include <asm/arch/regs-adc.h>
#include <asm/irq.h>

#include <mach/reg_gpio.h>
#include <mach/hardware.h>


#include "as3911.h"
#include "as3911_interrupt.h"
#include "errno.h"

#include <linux/ioctl.h>
#include "sleep.h"
#include "Platform.h"
#define inter_debug printk
//#define inter_debug(...)

bool_t emvStopRequestReceived(void);

#define AS3911_IRQ_CLR() { reg_gpio_clear_interrupt( BCM5892_GPB12 );}


//unsigned int g_flags;
#define QUCK_AS3911_IRQ_OFF()  //  local_irq_save(g_flags)
#define  QUCK_AS3911_IRQ_ON()    // local_irq_restore(g_flags)

/*!
 *****************************************************************************
 * \brief Evaluates to true if there is a pending interrupt request from the
 * AS3911.
 *****************************************************************************
 */

/*
******************************************************************************
* LOCAL DATA TYPES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/

/*! AS3911 interrutp mask. */
static volatile u32 as3911InterruptMask = 0;
/*! Accumulated AS3911 interrupt status. */
static volatile u32 as3911InterruptStatus = 0;

unsigned  long  quck_InterruptStatus  ;

/*
typedef struct {

	unsigned long times;
	long long clk_start;
} AS3911_TIMER;

AS3911_TIMER As3911_Timer[ 2 ] = { 0 ,0 };*/
u32 g_jiffies;
u32 g_jiffies_count;
//(unsigned char TimerNo, int ms )
//TimerNo  定时器编号
// ms  延迟时间 ,单位 毫秒.
 u32 ggjiffies_count=1;

#if 0
static irqreturn_t as3911_interrupt(int irq,void * dev_id,struct pt_regs * regs)
{
	
	u32 irqStatus = 0;
	//u8 i;
	AS3911_IRQ_CLR();
	
	//for(i=0;i<5;i++)
	//{
	as3911ContinuousRead(AS3911_REG_IRQ_MAIN, (u8*) &irqStatus, 3);
	as3911InterruptStatus |= irqStatus & as3911InterruptMask;
	//}
	printk("interrupt\n");

	return IRQ_HANDLED;
}


void as3911InterruptInit(void)
{
	unsigned int IRQ;
	printk("as3911InterruptInit \n");
	reg_gpio_disable_interrupt( BCM5892_GPB12 );
	reg_gpio_iotr_set_pin_type( BCM5892_GPB12,GPIO_PIN_TYPE_INPUT_WITH_INTERRUPT );
	AS3911_IRQ_CLR();

	 //reg_gpio_itr_set_interrupt_type_level( BCM5892_GPB12, GPIO_LEWEL_TRIGGER );  
	// high level interrupt trigger   //GPIO_LOW_LEVEL_INTERRUPT_TRIGGER  //GPIO_HIGH_LEVEL_INTERRUPT_TRIGGER
	// reg_gpio_itr_set_interrupt_type( BCM5892_GPB12, GPIO_HIGH_LEVEL_INTERRUPT_TRIGGER );
	
	//raising edge interrupt
	reg_gpio_itr_set_interrupt_type_level( BCM5892_GPB12, GPIO_EDGE_TRIGGER );    
	reg_gpio_itr_set_interrupt_type( BCM5892_GPB12, GPIO_RISING_EDGE_INTERRUPT_TRIGGER ); 	

	reg_gpio_set_pull_up_down_enable(BCM5892_GPB12);
	reg_gpio_set_pull_up_down(BCM5892_GPB12, 1);
	reg_gpio_set_pull_up_down_enable(BCM5892_GPB12);
	
	IRQ=gpio_to_irq(BCM5892_GPB12);

	
	if ( request_irq(IRQ, as3911_interrupt, IRQF_DISABLED, "POWER_NAME", 0) < 0) 
	{  
		printk(KERN_ERR "Power : Can't allocate irq %d\n", IRQ);
	} 
	reg_gpio_enable_interrupt(BCM5892_GPB12);

}

#endif

void  TimerStart( unsigned char TimerNo, int ms )
{
//	measure_counter_start();
	TimerNo=TimerNo;
	g_jiffies= get_timer_count();
#if 1
	ms=100;
#endif

	g_jiffies_count=ms;

}
//返回0超时了, 返回其他整数没有超时.
int TimerCheck( unsigned char TimerNo )
{
	u32 temp;
	temp=g_jiffies-get_timer_count();
	temp=(temp*168)/1000000;
	return (temp>=g_jiffies_count? 0:1);
}


s8 as3911EnableInterrupts(u32 mask)
{
    s8 error = ERR_NONE;
    u32 irqMask = 0;
    QUCK_AS3911_IRQ_OFF();
	inter_debug("i_begin: ");
	//local_irq_restore(quck_InterruptStatus); 
    error |= as3911ContinuousRead(AS3911_REG_IRQ_MASK_MAIN, (u8*) &irqMask, 3);
    irqMask &= ~mask;
    as3911InterruptMask |= mask;
    error |= as3911ContinuousWrite(AS3911_REG_IRQ_MASK_MAIN, (u8*) &irqMask, 3);
    QUCK_AS3911_IRQ_ON();

    if (ERR_NONE == error)
        return ERR_NONE;
    else
        return ERR_IO;
}

s8 as3911DisableInterrupts(u32 mask)
{
    s8 error = ERR_NONE;
    u32 irqMask = 0;
    QUCK_AS3911_IRQ_OFF();
	//local_irq_save(quck_InterruptStatus);
    error |= as3911ContinuousRead(AS3911_REG_IRQ_MASK_MAIN, (u8*) &irqMask, 3);
    irqMask |= mask;
    as3911InterruptMask &=  ~mask;
    error |= as3911ContinuousWrite(AS3911_REG_IRQ_MASK_MAIN, (u8*) &irqMask, 3);
	inter_debug(" -iend\n");
    QUCK_AS3911_IRQ_ON();

    if (ERR_NONE == error)
        return ERR_NONE;
    else
        return ERR_IO;
}

s8 as3911ClearInterrupts(u32 mask)
{
    s8 error = ERR_NONE;
    u32 irqStatus = 0;

    QUCK_AS3911_IRQ_OFF();
	
    error |= as3911ContinuousRead(AS3911_REG_IRQ_MAIN, (u8*) &irqStatus, 3);
    as3911InterruptStatus |= irqStatus & as3911InterruptMask;
    as3911InterruptStatus &= ~mask;

    QUCK_AS3911_IRQ_ON();

    if (ERR_NONE == error)
        return ERR_NONE;
    else
        return ERR_IO;
}

s8 as3911WaitForInterruptTimed(u32 mask, u16 timeout, u32 *irqs)
{
	bool_t timerExpired  = FALSE;
	u32      irqStatus        = 0;
	if( timeout > 0 )
	{
		TimerStart( TIMER_INTERRUPT_STATUS, timeout );	
	}
	
   	do
   	{
   		//quck_printk(timeout);
   		as3911ContinuousRead(AS3911_REG_IRQ_MAIN, (u8*) &irqStatus, 3);
		as3911InterruptStatus |= irqStatus & as3911InterruptMask;
	//	quck_printk(get_timer_count());
		if(irqStatus)
	    inter_debug(" %x,%x,%x; ",irqStatus,mask,as3911InterruptStatus);

		
		irqStatus = as3911InterruptStatus & mask;

		if ( timeout > 0 )
		{
			if ( TimerCheck( TIMER_INTERRUPT_STATUS ) > 0 )
			{
				timerExpired  = FALSE;
			}
			else
			{
				timerExpired  = TRUE;
			}
		}
	        if (emvStopRequestReceived())
	        	{
	        	*irqs=0;
	        		  printk(" WaitForInterrupt\n");	 
			   return  -6;	
		}
         
		
	}while( !irqStatus && !timerExpired );

	if(timerExpired)
	{
		  printk(" timerExpired ");	 

	}
	QUCK_AS3911_IRQ_OFF();
	as3911InterruptStatus &= ~irqStatus;
	QUCK_AS3911_IRQ_ON();

	*irqs = irqStatus;

          return ERR_NONE;
}

s8 as3911GetInterrupts(u32 mask, u32 *irqs)
{
	u32 irqStatus;
    QUCK_AS3911_IRQ_OFF();
	
	as3911ContinuousRead(AS3911_REG_IRQ_MAIN, (u8*) &irqStatus, 3);
	as3911InterruptStatus |= irqStatus & as3911InterruptMask;
	inter_debug(" get:%x,%x ,%x",irqStatus,mask,as3911InterruptStatus);
    *irqs = as3911InterruptStatus & mask;
    as3911InterruptStatus &= ~mask;

    QUCK_AS3911_IRQ_ON();

    return ERR_NONE;
}



/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/

