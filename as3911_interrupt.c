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
 * PROJECT: AS3911 firmware
 * $Revision: $
 * LANGUAGE: ANSI C
 */

/*! \file as3911_interrupt.c
 *
 * \author Oliver Regenfelder
 *
 * \brief AS3911 interrupt handling and ISR
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "as3911.h"
#include "as3911_interrupt.h"
#include "errno.h"
#include <linux/jiffies.h>
#include <linux/ioctl.h>
#include "sleep.h"
/*
******************************************************************************
* DEFINES
******************************************************************************
*/

/*
******************************************************************************
* MACROS
******************************************************************************
*/

/*!
 *****************************************************************************
 * \brief Clear the interrupt flag associated with the as3911 interrupt.
 *****************************************************************************
 */
#define AS3911_IRQ_CLR() { ; }

/*!
 *****************************************************************************
 * \brief Evaluates to true if there is a pending interrupt request from the
 * AS3911.
 *****************************************************************************
 */
#define AS3911_IRQ_IS_SET() ( 1)

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
/*
typedef struct {

	unsigned long times;
	long long clk_start;
} AS3911_TIMER;

AS3911_TIMER As3911_Timer[ 2 ] = { 0 ,0 };*/
u32 g_jiffies;
u32 g_jiffies_count;
#define QUCK_HZ_DEF 5
//(unsigned char TimerNo, int ms )
//TimerNo  定时器编号
// ms  延迟时间 ,单位 毫秒.
 u32 ggjiffies_count=1;


void  TimerStart( unsigned char TimerNo, int ms )
{
	
	TimerNo=TimerNo;
	g_jiffies= get_timer_count();

	g_jiffies_count=ms;
	if(g_jiffies_count>15)
	{
		g_jiffies_count=15;

 	}
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

    //AS3911_IRQ_OFF();

    error |= as3911ContinuousRead(AS3911_REG_IRQ_MASK_MAIN, (u8*) &irqMask, 3);
    irqMask &= ~mask;
    as3911InterruptMask |= mask;
    error |= as3911ContinuousWrite(AS3911_REG_IRQ_MASK_MAIN, (u8*) &irqMask, 3);

    //AS3911_IRQ_ON();

    if (ERR_NONE == error)
        return ERR_NONE;
    else
        return ERR_IO;
}

s8 as3911DisableInterrupts(u32 mask)
{
    s8 error = ERR_NONE;
    u32 irqMask = 0;

    //AS3911_IRQ_OFF();

    error |= as3911ContinuousRead(AS3911_REG_IRQ_MASK_MAIN, (u8*) &irqMask, 3);
    irqMask |= mask;
    as3911InterruptMask &=  ~mask;
    error |= as3911ContinuousWrite(AS3911_REG_IRQ_MASK_MAIN, (u8*) &irqMask, 3);

    //AS3911_IRQ_ON();

    if (ERR_NONE == error)
        return ERR_NONE;
    else
        return ERR_IO;
}

s8 as3911ClearInterrupts(u32 mask)
{
    s8 error = ERR_NONE;
    u32 irqStatus = 0;

    //AS3911_IRQ_OFF();

    error |= as3911ContinuousRead(AS3911_REG_IRQ_MAIN, (u8*) &irqStatus, 3);
    as3911InterruptStatus |= irqStatus & as3911InterruptMask;
    as3911InterruptStatus &= ~mask;

    //AS3911_IRQ_ON();

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
   		as3911ContinuousRead(AS3911_REG_IRQ_MAIN, (u8*) &irqStatus, 3);
		as3911InterruptStatus |= irqStatus & as3911InterruptMask;
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
		
	}while( !irqStatus && !timerExpired );

	//AS3911_IRQ_OFF();
	as3911InterruptStatus &= ~irqStatus;
	//AS3911_IRQ_ON();

	*irqs = irqStatus;

          return ERR_NONE;
}

s8 as3911GetInterrupts(u32 mask, u32 *irqs)
{
    //AS3911_IRQ_OFF();

    *irqs = as3911InterruptStatus & mask;
    as3911InterruptStatus &= ~mask;

    //AS3911_IRQ_ON();

    return ERR_NONE;
}



/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
