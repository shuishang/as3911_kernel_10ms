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

/*! \file as3911_modulation_adjustment.c
 *
 * \author Oliver Regenfelder
 *
 * \brief AS3911 modulation level adjustment
 *
 * Adjust the modulation level of the AS3911 based on the amplitude or phase of
 * the reader field.
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "aS3911.h"

/*
******************************************************************************
* DEFINES
******************************************************************************
*/

/*! Sanity timeout for the AS3911 direct command completed interrupt. */
#define AS3911_DCT_IRQ_SANITY_TIMEOUT   5

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

/*! Active modulation level adjustment mode. */
static AS3911GainMode_t as3911GainMode = AS3911_GAIN_FIXED;

/*! Lookup table for the active modulation level adjustment mode. */
static const AS3911GainTable_t *as3911GainTable = NULL;

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

void as3911SetGainMode(AS3911GainMode_t modulationLevelMode
    , const AS3911GainTable_t *modulationLevelTable)
{
    as3911GainMode = modulationLevelMode;
    as3911GainTable = modulationLevelTable;
}

void as3911GetGainMode(AS3911GainMode_t *modulationLevelMode
    , const AS3911GainTable_t *modulationLevelTable)
{
    *modulationLevelMode = as3911GainMode;
    modulationLevelTable = as3911GainTable;
}

void as3911AdjustGain()
{
    u32 irqs = 0;
    u8 measurementCommand = 0;
    u8 amplitudePhase = 0;
    u8 index = 0;

    if (AS3911_GAIN_FIXED == as3911GainMode)
        return;
    else if (AS3911_GAIN_FROM_AMPLITUDE == as3911GainMode)
        measurementCommand = AS3911_CMD_MEASURE_AMPLITUDE;
    else if (AS3911_GAIN_FROM_PHASE == as3911GainMode)
        measurementCommand = AS3911_CMD_MEASURE_PHASE;
    else
    {
        /* ToDo: enter debug code here. */
        return;
    }

    /* Measurement based modulation strength adjustment requires a modulation
     * level table with at least 2 entries to perform interpolation.
     */
    if ((as3911GainTable == NULL) || (as3911GainTable->length < 2))
    {
        /* ToDo: enter debug code here. */
        return;
    }

    as3911ClearInterrupts(AS3911_IRQ_MASK_DCT);
    as3911EnableInterrupts(AS3911_IRQ_MASK_DCT);
    as3911ExecuteCommand(measurementCommand);
    as3911WaitForInterruptTimed(AS3911_IRQ_MASK_DCT, AS3911_DCT_IRQ_SANITY_TIMEOUT, &irqs);
    if (irqs != AS3911_IRQ_MASK_DCT)
    {
        /* ToDo: enter debug code here. */
        return;
    }
    as3911DisableInterrupts(AS3911_IRQ_MASK_DCT);
    as3911ReadRegister(AS3911_REG_AD_RESULT, &amplitudePhase);

    for (index = 0; index < as3911GainTable->length; index++)
    {
        if (amplitudePhase <= as3911GainTable->x[index])
            break;
    }

	/* If amplitudePhase is greater than the last table entry, then use the
	 * gain reduction of the last table entry.
	 */
	if (index == as3911GainTable->length)
		index--;

    as3911WriteRegister(AS3911_REG_RX_CONF3, as3911GainTable->y[index]);
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
