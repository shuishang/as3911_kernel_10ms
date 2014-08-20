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
#include "logger.h"

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
static AS3911ModulationLevelMode_t as3911ModulationLevelMode = AS3911_MODULATION_LEVEL_FIXED;

/*! Lookup table for the active modulation level adjustment mode. */
static const AS3911ModulationLevelTable_t *as3911ModulationLevelTable = NULL;
static const AS3911ModulationLevelAutomaticAdjustmentData_t *as3911ModulationLevelAutomaticAdjustmentData = NULL;

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
 * \brief Calculate a linear interpolated value.
 *
 * \note \a x1 must be <= x2.
 *
 * Calculate the linear interpolated value at \a xi. The curve for the linear
 * interpolation is defined by (\a x1, \a y1) and (\a x2, \a y2).
 * For values \a xi below \a x1 the function will return \a y1, and for values
 * \a xi above \a x2 the function will return \a y2. For any intermediate value
 * the function will return y1 + (y2 - y1) * (xi - x1) / (x2 - x1).
 *
 * \param[in] x1 X-coordinate of the first point.
 * \param[in] y1 Y-coordinate of the first point.
 * \param[in] x2 X-coordinate of the second point.
 * \param[in] y2 Y-coordinate of the second point.
 * \param[in] xi X-coordinate of the interpolation point.
 *
 * \return The interpolated Y-value at \a xi.
 *****************************************************************************
 */
int as3911GetInterpolatedValue(int x1, int y1, int x2, int y2, int xi);

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

void as3911SetModulationLevelMode(AS3911ModulationLevelMode_t modulationLevelMode
    , const void *modulationLevelData)
{
    as3911ModulationLevelMode = modulationLevelMode;
	
    if (AS3911_MODULATION_LEVEL_FIXED == modulationLevelMode)
	{
		return;
	}
	else if (AS3911_MODULATION_LEVEL_AUTOMATIC == modulationLevelMode)
	{
		as3911ModulationLevelAutomaticAdjustmentData = modulationLevelData;
	}
	else if (  (AS3911_MODULATION_LEVEL_FROM_PHASE == modulationLevelMode)
	        || (AS3911_MODULATION_LEVEL_FROM_AMPLITUDE == modulationLevelMode))
	{
		as3911ModulationLevelTable = modulationLevelData;
	}
}

void as3911GetModulationLevelMode(AS3911ModulationLevelMode_t *modulationLevelMode
    , const void *modulationLevelData)
{
    *modulationLevelMode = as3911ModulationLevelMode;
    if (AS3911_MODULATION_LEVEL_FIXED == as3911ModulationLevelMode)
	{
		modulationLevelData = NULL;
	}
	else if (AS3911_MODULATION_LEVEL_AUTOMATIC == as3911ModulationLevelMode)
	{
		modulationLevelData = as3911ModulationLevelAutomaticAdjustmentData;
	}
	else if (  (AS3911_MODULATION_LEVEL_FROM_PHASE == as3911ModulationLevelMode)
	        && (AS3911_MODULATION_LEVEL_FROM_AMPLITUDE == as3911ModulationLevelMode))
	{
		modulationLevelData = as3911ModulationLevelTable;
	}
}

void as3911AdjustModulationLevel(void)
{
    u32 irqs = 0;
    u8 measurementCommand = 0;
    u8 amplitudePhase = 0;
    u8 antennaDriverStrength = 0;
    u8 index = 0;

    if (AS3911_MODULATION_LEVEL_FIXED == as3911ModulationLevelMode)
        return;
	else if (AS3911_MODULATION_LEVEL_AUTOMATIC == as3911ModulationLevelMode)
	{
		int count = 0;
		
		as3911ClearInterrupts(AS3911_IRQ_MASK_DCT);
		as3911EnableInterrupts(AS3911_IRQ_MASK_DCT);
		as3911ExecuteCommand(AS3911_CMD_CALIBRATE_MODULATION);
		as3911WaitForInterruptTimed(AS3911_IRQ_MASK_DCT, AS3911_DCT_IRQ_SANITY_TIMEOUT, &irqs);
		if (irqs != AS3911_IRQ_MASK_DCT)
		{
			/* ToDo: enter debug code here. */
			return;
		}
		as3911DisableInterrupts(AS3911_IRQ_MASK_DCT);

		for (count = 0; count < as3911ModulationLevelAutomaticAdjustmentData->delay; count++)
		{
			/* FIXME: replace with microseconds sleep from AMS base firmware. */

		}
	}
    else if (AS3911_MODULATION_LEVEL_FROM_AMPLITUDE == as3911ModulationLevelMode)
        measurementCommand = AS3911_CMD_MEASURE_AMPLITUDE;
    else if (AS3911_MODULATION_LEVEL_FROM_PHASE == as3911ModulationLevelMode)
        measurementCommand = AS3911_CMD_MEASURE_PHASE;
    else
    {
        /* ToDo: enter debug code here. */
        return;
    }

    /* Measurement based modulation strength adjustment requires a modulation
     * level table with at least 2 entries to perform interpolation.
     */
    if ((as3911ModulationLevelTable == NULL) || (as3911ModulationLevelTable->length < 2))
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

    for (index = 0; index < as3911ModulationLevelTable->length; index++)
    {
        if(amplitudePhase >= as3911ModulationLevelTable->x[index])
        {

			 antennaDriverStrength =as3911ModulationLevelTable->y[index];
			 break;
		}
           
    }
    /* Use the last interpolation level dataset for any values outside the highest.
     * x-value from the datasets.
     */

    as3911WriteRegister(AS3911_REG_RFO_AM_ON_LEVEL, antennaDriverStrength);
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/

int as3911GetInterpolatedValue(int x1, int y1, int x2, int y2, int xi)
{
    if (xi <= x1) {
        return y1;
    } else if (xi >= x2) {
        return y2;
    } else {
        return y1 + (((long) y2 - y1) * (xi - x1)) / (x2 - x1);
    }
}
