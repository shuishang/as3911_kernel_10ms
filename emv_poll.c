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

/*! \file emv_poll.c
 *
 * \author Oliver Regenfelder
 *
 * \brief EMV compliant polling and collision detection.
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include <linux/stddef.h>
#include "emv_poll.h"
#include "emv_hal.h"
#include "emv_standard.h"
#include "emv_main.h"
#include "emv_typeA.h"
#include "emv_typeB.h"
#include "main.h"
#undef poll_debug 
#define poll_debug printk

//#define poll_debug(...)  
/*
******************************************************************************
* DEFINES
******************************************************************************
*/

/*! Frame delay time (timeout) used for ISO14443-A HLTA commands. */
#define EMV_HLTA_FDT   1250
#define  debug  printk
/*
******************************************************************************
* MACROS
******************************************************************************
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

/*! ISO14443-A cards polling flag as recommended by the EMV standard. */
static int emvTypeA;

/*! ISO14443-B cards polling flag as recommended by the EMV standard. */
static int emvTypeB;

/*
******************************************************************************
* LOCAL TABLES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL VARIABLE DEFINITIONS
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/

void emvPollSingleIteration(void)
{
    u8 hltaCommand[2];
    emvTypeA = 0;
    emvTypeB = 0;
    debug("emvPollSingleIteration()\r\n");
    emvHalSleepMilliseconds(EMV_T_P);
    if (emvTypeACardPresent())
    {
        /* ISO14443-A card(s) found. */
        emvTypeA = 1;

        /* Send HLTA command. */
        hltaCommand[0] = 0x50;
        hltaCommand[1] = 0x00 ; 	
        emvHalTransceive(hltaCommand, sizeof(hltaCommand), NULL, 0, NULL, EMV_HLTA_FDT, EMV_HAL_TRANSCEIVE_WITH_CRC);
    }

    emvHalSleepMilliseconds(EMV_T_P);
    if (emvTypeBCardPresent())
    {
        /* ISO14443-B card(s) found. */
        emvTypeB = 1;
    }
}


			/*while(1)
			{	
				local_irq_save(flags);
				quck_udelay(1000);
				SDeselect();
				quck_udelay(1000);
			 
			}
			 local_irq_restore(flags);
			*/
s16 emvPoll(void)
{
    u8 hltaCommand[2];
    unsigned long flags;		
    emvTypeA = 0;
    emvTypeB = 0;
    
   // debug("emvPoll() \r\n");	 	
    /* Poll as long as no cards are found. */
  //   local_irq_save(flags);
    while (1)
    {
        if (emvStopRequestReceived())
            return EMV_ERR_STOPPED;
        if (emvTypeA != 0)
            break;

        /* Wait for t_p. */
        emvHalSleepMilliseconds(4);


		
		printk("\n--------a_card_begin-----\n");
        if (emvTypeACardPresent())
        {
            /* ISO14443-A card(s) found. */
            emvTypeA = 1;
	   		poll_debug("emvTypeACardPresent() \r\n");	 
            /* Send HLTA command. */
             hltaCommand[0] = 0x50;
	    hltaCommand[1] = 0x00;	
	 //   debug("hltaCommand\r\n");	 	
            emvHalTransceive(hltaCommand, sizeof(hltaCommand), NULL, 0, NULL, EMV_HLTA_FDT, EMV_HAL_TRANSCEIVE_WITH_CRC);
        }

        if (emvTypeB != 0)
            break;

        /* Wait for t_p. */
        emvHalSleepMilliseconds (EMV_T_P);
    	printk("\n--------b_card_begin-----\n");
        if (emvTypeBCardPresent())
        {
            /* ISO14443-B card(s) found. */
	   		poll_debug("emvTypeBCardPresent() \r\n");	 		
            emvTypeB = 1;
        }
		deug_flag=1;
		
    }
  //  local_irq_restore(flags);
	
    return EMV_ERR_OK;
}

s16 emvCollisionDetection(EmvPicc_t *picc)
{
    if ((emvTypeA) != 0 && (emvTypeB != 0))
        return EMV_ERR_COLLISION;

    if (emvTypeA != 0)
    {
        return emvTypeAAnticollision(picc);
    }
    else if (emvTypeB != 0)
    {
        return emvTypeBAnticollision(picc);
    }
    else
        return EMV_ERR_COLLISION;
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
