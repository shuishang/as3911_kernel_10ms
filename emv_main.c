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
 * PROJECT: AS911 firmware
 * $Revision: $
 * LANGUAGE: ANSI C
 */

/*! \file emv_main.c
 *
 * \author Oliver Regenfelder
 *
 * \brief EMV terminal application.
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include <linux/stddef.h>



#include "emv_hal.h"
#include "emv_standard.h"
#include "emv_poll.h"
#include "emv_layer4.h"
#include "emv_display.h"
#include "sleep.h"
#include "emv_main.h"
#include "main.h"

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

/*! Indicates whether a stop request has been received from the GUI or not. */
static volatile bool_t emvStopRequestReceivedFlag = FALSE;

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

/*!
 *****************************************************************************
 * \brief Convert 13.56 MHz carrier cycle numbers to milliseconds.
 *
 * \note \a numCarrierCycles must be <= 888720133 (65535 ms).
 *
 * \param numCarrierCycles Number of carrier cycles.

 * \return \a numCarrierCycles converted to milliseconds.
 *****************************************************************************
 */
static u16 emvConvertCarrierCyclesToMilliseconds(u32 numCarrierCycles);

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

s16 emvStartTerminalApplication(s16 (*application)(void))
{
    s16 error = EMV_ERR_OK;

    /* Reset stop request received flag. */
    emvStopRequestReceivedFlag = FALSE;

    /* Implementation:
     * Error checking is done at the beginning of the while loop because
     * error handling is the same for all stages of the terminal main loop.
     * If any stage encounters an error it uses a continue statement to fall
     * through the rest of the while loop and reach the error handling code.
     */
    while(1)
    {
        EmvPicc_t picc;

        if (EMV_ERR_STOPPED == error)
        {
            /* Received stop request, stop terminal main loop. */
            return EMV_ERR_STOPPED;
        }
        if (EMV_ERR_OK != error)
        {
		//emvDisplayError(error);
		//SSelect();  
		/* Reset field and continue with polling. */
		emvHalResetField();
		//SDeselect();  
        }

        /* Polling. */
        emvDisplayMessage(EMV_M_POLLING);
        error = emvPoll();
        if (EMV_ERR_OK != error)
            continue;

        /* Anticollision. */
        sleepMilliseconds(EMV_T_P);
        error = emvCollisionDetection(&picc);
        if (EMV_ERR_OK != error)
            continue;

        /* Activation. */
        error = emvActivate(&picc);
        if (EMV_ERR_OK != error)
            continue;

        /* Wait for SFGT. */
        if(picc.sfgi > 0)
        {
            u32 sfgtCycles = (4096UL + 384) << picc.sfgi;
            u16 sfgtMilliseconds = emvConvertCarrierCyclesToMilliseconds(sfgtCycles);
            sleepMilliseconds(sfgtMilliseconds);
        }

        /* Initialize layer 4. */
        error = emvInitLayer4(&picc);
        if (EMV_ERR_OK != error)
            continue;

        /* Start terminal application. */
        if (application != NULL)
            error = application();
        else
            error = EMV_ERR_OK;

        if (EMV_ERR_OK != error)
            continue;

     //   emvHalResetField();
//test in 2014-1-9
        /* Card removal. */
        emvDisplayMessage(EMV_M_REMOVE_CARD);
        error = emvRemove(&picc);
      if (EMV_ERR_OK != error)
            continue;
	  quck_timer_count(1);
	//debug
	
    }
}

bool_t emvStopRequestReceived(void)
{

  if (QDeselect())
    {
        emvStopRequestReceivedFlag = FALSE;
        return TRUE;
    }
	
    return FALSE;
}

void emvStopTerminalApplication(void)
{
    emvStopRequestReceivedFlag = TRUE;
}


/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/

static u16 emvConvertCarrierCyclesToMilliseconds(u32 num_cycles)
{
    return (num_cycles / 13560) + 1;
}
