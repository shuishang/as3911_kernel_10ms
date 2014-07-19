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

#include "as3911_interrupt.h"

#include "as3911.h"
#include "errno.h"

#include <linux/ioctl.h>
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

typedef struct {

	unsigned long times;
	long long clk_start;
} AS3911_TIMER;

AS3911_TIMER As3911_Timer[ 2 ] = { 0 ,0 };

//(unsigned char TimerNo, int ms )
//TimerNo  定时器编号
// ms  延迟时间 ,单位 毫秒.
void  TimerStart( unsigned char TimerNo, int ms )
{
	TimerNo=TimerNo;
	ms=TimerNo;

/*	struct timeval tv ;
	struct timezone tz;

	gettimeofday( &tv, &tz );
	As3911_Timer[ TimerNo ].clk_start = tv.tv_sec *1000 + tv.tv_usec/1000; //ms
	As3911_Timer[ TimerNo ].times      = ms;*/
}

int TimerCheck( unsigned char TimerNo )
{
	/*long RestTime                                   = 0;
	unsigned long CurrentTime            = 0;
	unsigned long ElapsedTime            =0;	
	struct timeval tv;
	struct timezone tz;

	gettimeofday( &tv, &tz );
	CurrentTime = tv.tv_sec * 1000 + tv.tv_usec /1000;
	ElapsedTime = CurrentTime - As3911_Timer[ TimerNo ].clk_start;
	RestTime = As3911_Timer[ TimerNo ].times - ElapsedTime;
	if ( RestTime > 0 )
	{
		return RestTime;
	}
	else
	{
		return 0;
	}*/
	return 0;
}


s8 as3911EnableInterrupts(u32 mask)
{
    s8 error = ERR_NONE;
    u32 irqMask = 0;

    AS3911_IRQ_OFF();

    error |= as3911ContinuousRead(AS3911_REG_IRQ_MASK_MAIN, (u8*) &irqMask, 3);
    irqMask &= ~mask;
    as3911InterruptMask |= mask;
    error |= as3911ContinuousWrite(AS3911_REG_IRQ_MASK_MAIN, (u8*) &irqMask, 3);

    AS3911_IRQ_ON();

    if (ERR_NONE == error)
        return ERR_NONE;
    else
        return ERR_IO;
}

s8 as3911DisableInterrupts(u32 mask)
{
    s8 error = ERR_NONE;
    u32 irqMask = 0;

    AS3911_IRQ_OFF();

    error |= as3911ContinuousRead(AS3911_REG_IRQ_MASK_MAIN, (u8*) &irqMask, 3);
    irqMask |= mask;
    as3911InterruptMask &=  ~mask;
    error |= as3911ContinuousWrite(AS3911_REG_IRQ_MASK_MAIN, (u8*) &irqMask, 3);

    AS3911_IRQ_ON();

    if (ERR_NONE == error)
        return ERR_NONE;
    else
        return ERR_IO;
}

s8 as3911ClearInterrupts(u32 mask)
{
    s8 error = ERR_NONE;
    u32 irqStatus = 0;

    AS3911_IRQ_OFF();

    error |= as3911ContinuousRead(AS3911_REG_IRQ_MAIN, (u8*) &irqStatus, 3);
    as3911InterruptStatus |= irqStatus & as3911InterruptMask;
    as3911InterruptStatus &= ~mask;

    AS3911_IRQ_ON();

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
    AS3911_IRQ_OFF();

    *irqs = as3911InterruptStatus & mask;
    as3911InterruptStatus &= ~mask;

    AS3911_IRQ_ON();

    return ERR_NONE;
}



/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
