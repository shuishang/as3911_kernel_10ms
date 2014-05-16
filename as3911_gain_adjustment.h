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
 
/*! \file as3911_gain_adjustment.h
 *
 * \author Oliver Regenfelder
 *
 * \brief AS3911 gain reduction adjustment
 */

#ifndef AS3911_GAIN_ADJUSTMENT_H
#define AS3911_GAIN_ADJUSTMENT_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "ams_types.h"

/*
******************************************************************************
* DEFINES
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL DATA TYPES
******************************************************************************
*/

/*! \ingroup as3911RfidCom
 *****************************************************************************
 * Available gain adjustment modes.
 *****************************************************************************
 */
typedef enum _AS3911GainMode_
{
    AS3911_GAIN_FIXED /*!< No dynamic gain adjustment. */,
    AS3911_GAIN_FROM_AMPLITUDE /*!< Gain is adjusted based on RF amplitude. */,
    AS3911_GAIN_FROM_PHASE /*!< Gain is adjusted based on RFI, RFO signal phase difference. */
} AS3911GainMode_t;

/*! \ingroup as3911RfidCom
 *****************************************************************************
 * \brief Gain adjustment lookup table.
 *
 * Lookup table for gain adjustment. This table consists of x and
 * y values as separate arrays. So x[i], y[i] define the (x,y) coordinates of
 * point i of the lookup table.
 *
 * \note The x values must be monotonically increasing to produce correct
 * results during linear interpolation.
 *****************************************************************************
 */
typedef struct _AS3911GainTable_
{
    u8 length; /*!< Length of the \a x and \a y point arrays. */
    u8 *x; /*!< X data points of the modulation level table. */
    u8 *y; /*!< Y data points of the modulation level table. */
} AS3911GainTable_t;

/*
******************************************************************************
* GLOBAL VARIABLE DECLARATIONS
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/

/*! \ingroup as3911RfidCom
 *****************************************************************************
 * \brief Set the modulation level adjustment mode and modulation level
 * adjustment table.
 *
 * Set the modualtion level adjustment mode and configure additional data
 * needed by some of the adjustment modes. There are three modes available:
 *
 * AS3911_MODULATION_LEVEL_FIXED: no dynamic adjustment of the modulation level
 * is performed. The \a modulationLevelTable parameter will be ignored.
 *
 * AS3911_MODULATION_LEVEL_FROM_AMPLITUDE: The modulation level is adjusted
 * based on the RF amplitude and the lookup table values in
 * \a modulationLevelTable.
 *
 * AS3911_MODULATION_LEVEL_FROM_PHASE: The modulation level is adjusted based on
 * the phase difference between RFI and RFO signals and the lookup table values
 * in \a modulationLevelTable.
 *
 * \note Internally only a pointer to the modulationLevelTable will be stored.
 * The data structure pointed to by \a modulationLevelTable should therefore
 * not be modified while in use by this module.
 *
 * \param[in] modulationLevelMode Set the modulation level adjustment mode.
 * \param[in] modulationLevelTable Set the modulation level lookup table to
 * be used in the AS3911_MODULATION_LEVEL_FROM_AMPLITUDE and
 * AS3911_MODULATION_LEVEL_FROM_PHASE modes.
 *****************************************************************************
 */
void as3911SetGainMode(AS3911GainMode_t modulationLevelMode
    , const AS3911GainTable_t *modulationLevelTable);

/*! \ingroup as3911RfidCom
 *****************************************************************************
 * \brief Get the active modulation level adjustment mode and modulation level
 * adjustment table.
 *
 * \param[out] modulationLevelMode Set to the active modulation level adjustment
 * mode
 * \param[out] modulationLevelTable Set to the active modulation level
 * adjustment table.
 *****************************************************************************
 */
void as3911GetGainMode(AS3911GainMode_t *modulationLevelMode
    , const AS3911GainTable_t *modulationLevelTable);

/*! \ingroup as3911RfidCom
 *****************************************************************************
 * \brief Adjust the modulation level.
 *
 * Adjust the modualtion level based on the active modulation level
 * adjustment mode and the modulation level adjustment table.
 *
 * \note This method may block for up to 50 microseconds depending on the active
 * modulation level adjustment mode.
 *****************************************************************************
 */
void as3911AdjustGain();

#endif /* AS3911_GAIN_ADJUSTMENT_H */
