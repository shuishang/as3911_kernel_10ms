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

void emvPollSingleIteration()
{
    emvTypeA = 0;
    emvTypeB = 0;
    debug("emvPollSingleIteration()\r\n");
    emvHalSleepMilliseconds(EMV_T_P);
    if (emvTypeACardPresent())
    {
        /* ISO14443-A card(s) found. */
        emvTypeA = 1;

        /* Send HLTA command. */
        u8 hltaCommand[2] = { 0x50, 0x00 };
        emvHalTransceive(hltaCommand, sizeof(hltaCommand), NULL, 0, NULL, EMV_HLTA_FDT, EMV_HAL_TRANSCEIVE_WITH_CRC);
    }

    emvHalSleepMilliseconds(EMV_T_P);
    if (emvTypeBCardPresent())
    {
        /* ISO14443-B card(s) found. */
        emvTypeB = 1;
    }
}

void  package_monitor_save_time()
{/*
	char buf[1024];
	struct timeb tp;
	struct tm	*tm;

	ftime(&tp);
	tm = localtime(&(tp.time));
	printk("[%03d] ", tp.millitm );

	return -1	 ;*/

}

s16 emvPoll()
{
    emvTypeA = 0;
    emvTypeB = 0;
    debug("emvPoll() \r\n");	 	
    /* Poll as long as no cards are found. */
    while (1)
    {
        if (emvStopRequestReceived())
            return EMV_ERR_STOPPED;
        if (emvTypeA != 0)
            break;

        /* Wait for t_p. */
        emvHalSleepMilliseconds(EMV_T_P);
	package_monitor_save_time();	  
	printk("    A\r\n");	
        if (emvTypeACardPresent())
        {
            /* ISO14443-A card(s) found. */
            emvTypeA = 1;
	    debug("emvTypeACardPresent() \r\n");	 
            /* Send HLTA command. */
            u8 hltaCommand[2] = { 0x50, 0x00 };
	    debug("hltaCommand\r\n");	 	
            emvHalTransceive(hltaCommand, sizeof(hltaCommand), NULL, 0, NULL, EMV_HLTA_FDT, EMV_HAL_TRANSCEIVE_WITH_CRC);
        }

        if (emvTypeB != 0)
            break;

        /* Wait for t_p. */
        emvHalSleepMilliseconds (3);
        package_monitor_save_time();
        printk("   B\r\n");	
        if (emvTypeBCardPresent())
        {
            /* ISO14443-B card(s) found. */
	   debug("emvTypeBCardPresent() \r\n");	 		
            emvTypeB = 1;
        }
    }

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
