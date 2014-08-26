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

/*! \file emv_hal.c
 *
 * \author Oliver Regenfelder
 *
 * \brief EMV module RFID hardware abstraction layer.
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "emv_hal.h"

#include "errno.h"

#include "emv_standard.h"
#include "as3911.h"
#include "sleep.h"

/*
******************************************************************************
* DEFINES
******************************************************************************
*/

/*! AS3911 operation control register transmit enable bit mask. */
#define EMV_HAL_REG_OPCONTROL_TXEN_BIT    0x08

/*!
 *****************************************************************************
 * Receiver dead time (in carrier cycles) for the ISO14443A protocol.
 * See [CCP v2.01, PCD 4.8.1.3].
 * Calculation:
 *
 * 1108: Deaftime if the last transmitted data bis was logic '1'
 *
 *  276: Time from the rising pulse of the pause of the logic '1'
 *       (i.e. the timepoint to measure the deaftime from), to the
 *       actual end of the EOF sequence (the point where the MRT starts).
 *       Please note that the AS3911 uses the ISO14443-2 definition
 *       where the EOF consists of logic '0' followed by sequence Y.
 * 
 *   64: Adjustment for the MRT timer jitter. Currently the MRT timer
 *       will have any timeout between the set timeout and the set timout
 *       + 64 cycles.
 *****************************************************************************
 */
#define EMV_HAL_ISO14443A_RECEIVER_DEADTIME     (1108 - 276 - 64)

/*!
 *****************************************************************************
 * Receiver dead time (in carrier cycles) for the ISO14443B protocol.
 * See [CCP v2.01, PCD 4.8.1.3].
 *
 * 1024: TR0_MIN
 *
 *  340: Time from the rising edge of the EoS to the starting point
 *       of the MRT timer (sometime after the final high part of the
 *       EoS is completed).
 * 
 *   64: Adjustment for the MRT timer jitter. Currently the MRT timer
 *       will have any timeout between the set timeout and the set timout
 *       + 64 cycles.
 *****************************************************************************
 */
#define EMV_HAL_ISO14443B_RECEIVER_DEADTIME     (1024 - 340 - 64)

/*!
 *****************************************************************************
 * Adjustment from the timeout requested by the caller to the timeout
 * value which has to be used for the AS3911.
 *****************************************************************************
 */
#define EMV_HAL_TYPE_A_TIMEOUT_ADJUSTMENT    512

/*!
 *****************************************************************************
 * Adjustment from the timeout requested by the caller to the timeout
 * value which has to be used for the AS3911.
 *****************************************************************************
 */
#define EMV_HAL_TYPE_B_TIMEOUT_ADJUSTMENT    6360

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

/*! Stores the currently active standard. */
static EmvHalStandard_t emvioActiveStandard;

static AS3911ModulationLevelMode_t emvHalTypeBModulationLevelMode = AS3911_MODULATION_LEVEL_FIXED;
static const void *emvHalTypeBModulationLevelModeData = NULL;
/**@}*/

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
 * \brief Convert AS3911 module error codes to EMV IO module error codes.
 *
 * \param as3911errorCode AS3911 module error code.
 * \return EMV IO module error code.
 *****************************************************************************
 */
static s16 emvioConvertErrorCode(s16 as3911ErrorCode);

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

void emvHalSleepMilliseconds(u16 milliseconds)
{
    sleepMilliseconds(milliseconds);
}
#define AS3911_REG_AM_MOD_DEPTH_CONTROL_mod_8percent       (0xb<<1)	
#define AS3911_REG_AM_MOD_DEPTH_CONTROL_mod_10percent      (0xe<<1)	
#define AS3911_REG_AM_MOD_DEPTH_CONTROL_mod_14percent      (0x14<<1)	
#define AS3911_REG_AM_MOD_DEPTH_CONTROL_mod_20percent      (0x20<<1)	
#define AS3911_REG_AM_MOD_DEPTH_CONTROL_mod_30percent      (0x37<<1)	
#define AS3911_REG_AM_MOD_DEPTH_CONTROL_mod_33percent      (0x3f<<1)
#define AS3911_REG_AM_MOD_DEPTH_CONTROL_mod_100percent     (0x20<<1)
s8 as3911ExecuteCommandAndGetResult(u8 cmd, u8 resreg, u8 sleeptime, u8* result)
{
    s8 err;
    u32 tmp;
	
    if (cmd == AS3911_CMD_ADJUST_REGULATORS)
    {
        err = as3911ExecuteCommand(cmd);
        sleepMilliseconds(3);
    }
    else if (   (cmd >= AS3911_CMD_INITIAL_RF_COLLISION && cmd <= AS3911_CMD_RESPONSE_RF_COLLISION_0)
            || (cmd == AS3911_CMD_MEASURE_AMPLITUDE)
            || (cmd >= AS3911_CMD_ADJUST_REGULATORS && cmd <= AS3911_CMD_MEASURE_PHASE)
            || (cmd >= AS3911_CMD_CALIBRATE_C_SENSOR && cmd <= AS3911_CMD_MEASURE_VSS)
            || (cmd >= 0xFD && cmd <= 0xFE )
       )
    {
        as3911EnableInterrupts(AS3911_IRQ_MASK_DCT);
        as3911GetInterrupts(AS3911_IRQ_MASK_DCT,&tmp);
        err = as3911ExecuteCommand(cmd);
        as3911WaitForInterruptTimed(AS3911_IRQ_MASK_DCT, sleeptime,&tmp);
        as3911DisableInterrupts(AS3911_IRQ_MASK_DCT);
    }
    else
    {
        err = as3911ExecuteCommand(cmd);
        sleepMilliseconds(sleeptime);
    }

    /* read out the result if the pointer is not NULL */
    if (result)
        err |= as3911ReadRegister(resreg, result);

    return err;
}

s8 as3911CalibrateModulationDepth(u8* result)
{
    return as3911ExecuteCommandAndGetResult(AS3911_CMD_CALIBRATE_MODULATION,
                                        AS3911_REG_AM_MOD_DEPTH_RESULT,10,result);
}


s8 emvHalSetStandard(EmvHalStandard_t standard)
{
    if (EMV_HAL_TYPE_A == standard)
    {
        emvioActiveStandard = standard;

        /* Set the AS3911 to ISO14443-A, 106kBit/s rx/tx datarate. */
        as3911WriteRegister(AS3911_REG_MODE, 0x08);
        as3911WriteRegister(AS3911_REG_BIT_RATE, 0x00);

        /* Disable transmit without parity, disable receive without parity&crc, set
         * to default pulse width, and disable special anticollision mode.
         */
        as3911WriteRegister(AS3911_REG_ISO14443A_NFC, 0x00);


        /* OOK modulation, no tolerant processing of the first byte. */
        as3911WriteRegister(AS3911_REG_AUX, 0x00);

        /* Set the receiver deadtime. */
        as3911SetReceiverDeadtime(EMV_HAL_ISO14443A_RECEIVER_DEADTIME);

        /* Set the frame delay time to the minimum allowed PCD frame delay time. */
        as3911SetFrameDelayTime(EMV_FDT_PCD_MIN);

        /* Disable dynamic adjustment of the modulation level. */
        as3911SetModulationLevelMode(AS3911_MODULATION_LEVEL_FIXED, NULL);
    }
    else if (EMV_HAL_TYPE_B == standard)
    {
        emvioActiveStandard = standard;

        /* Set the AS3911 to ISO14443-B, 106kBit/s rx/tx datarate. */
        as3911WriteRegister(AS3911_REG_MODE, 0x10);
        as3911WriteRegister(AS3911_REG_BIT_RATE, 0x00);

        /* Configure AM modulation for ISO14443B. */
        as3911WriteRegister(AS3911_REG_AUX, 0x20);

        /* Set the receiver deadtime. */
        as3911SetReceiverDeadtime(EMV_HAL_ISO14443B_RECEIVER_DEADTIME);

        /* Set the frame delay time to the minimum allowed PCD frame delay time. */
        as3911SetFrameDelayTime(EMV_FDT_PCD_MIN);

        /* Enable dynamic adjustment of the modulation level. */
        as3911SetModulationLevelMode(emvHalTypeBModulationLevelMode, emvHalTypeBModulationLevelModeData);
		//u8 tmp;
      //  as3911WriteRegister(AS3911_REG_AM_MOD_DEPTH_CONF, AS3911_REG_AM_MOD_DEPTH_CONTROL_mod_10percent);
 	
 //       as3911CalibrateModulationDepth(&tmp);	
    }
    else
    {
        /* Handle unkown parameter error here. */
        return EMV_HAL_ERR_PARAM;
    }

    return EMV_HAL_ERR_OK;
}

s8 emvHalSetErrorHandling(EmvHalErrorHandling_t errorHandling)
{
    if (EMV_HAL_PREACTIVATION_ERROR_HANDLING == errorHandling)
    {
        /* Disable higher layer EMV exception processing. */
        as3911EnableEmvExceptionProcessing(FALSE);
        as3911SetTransmissionErrorThreshold(0);

        /* Enable detailed anticollision detection. */
        // as3911ModifyRegister(AS3911_REG_ISO14443A_NFC, 0x01, 0x01);
    }
    else if (EMV_HAL_LAYER4_ERROR_HANDLING == errorHandling)
    {
        /* Errornous frames <= 4 bytes in length shall not be considered
         * transmission errors.
         */
        as3911SetTransmissionErrorThreshold(4);
        as3911EnableEmvExceptionProcessing(TRUE);

        /* Disable detailed anticollision detction.
         * This autmatically enables the enhanced SOF detection.
         */ 
        // as3911ModifyRegister(AS3911_REG_ISO14443A_NFC, 0x01, 0x00);
    }
    else
    {
        /* Handle unkown parameter error here. */
        return EMV_HAL_ERR_PARAM;
    }

    return EMV_HAL_ERR_OK;
}

s8 emvHalActivateField(bool_t activateField)
{
    s8 error = ERR_NONE;

    if (activateField)
        error |= as3911ModifyRegister(AS3911_REG_OP_CONTROL, EMV_HAL_REG_OPCONTROL_TXEN_BIT, EMV_HAL_REG_OPCONTROL_TXEN_BIT);
    else
        error |= as3911ModifyRegister(AS3911_REG_OP_CONTROL, EMV_HAL_REG_OPCONTROL_TXEN_BIT, 0x00);

    if (ERR_NONE == error)
        return ERR_NONE;
    else
        return ERR_IO;
}

bool_t emvHalFieldIsActivated(void)
{
    u8 regOpcontrol = 0;

    if (ERR_NONE != as3911ReadRegister(AS3911_REG_OP_CONTROL, &regOpcontrol))
        return FALSE;

    if (regOpcontrol & EMV_HAL_REG_OPCONTROL_TXEN_BIT)
        return TRUE;
    else
        return FALSE;
}

s8 emvHalResetField(void)
{
    s8 error = ERR_NONE;

    error |= emvHalActivateField(FALSE);

    emvHalSleepMilliseconds(EMV_T_RESET);

    error |= emvHalActivateField(TRUE);

    if (ERR_NONE == error)
        return ERR_NONE;
    else
        return ERR_IO;
}

s8 emvHalTransceive(const u8 *request, u32 requestLength, u8 *response
    , u32 maxResponseLength, u32 *responseLength, u32 timeout, EmvHalTransceiveMode_t transceiveMode)
{
    s16 error = AS3911_NO_ERROR;

    /* Adjust timeout by the delay between the start of card modulation
     * and the SoR interrupt of the AS3911.
     */
    if (EMV_HAL_TYPE_A == emvioActiveStandard)
        timeout += EMV_HAL_TYPE_A_TIMEOUT_ADJUSTMENT;
    else if (EMV_HAL_TYPE_B == emvioActiveStandard)
        timeout += EMV_HAL_TYPE_B_TIMEOUT_ADJUSTMENT;

    if (EMV_HAL_TRANSCEIVE_WITH_CRC == transceiveMode)
    {
        error = as3911Transceive(request, requestLength, response, maxResponseLength,
                    responseLength, timeout, AS3911_CRC_TO_FIFO);
    }
    else if (EMV_HAL_TRANSCEIVE_WITHOUT_CRC == transceiveMode)
    {
        error = as3911Transceive(request, requestLength, response, maxResponseLength,
                    responseLength, timeout,
                    AS3911_IGNORE_CRC | AS3911_TRANSMIT_WITHOUT_CRC);
    }
    else if (EMV_HAL_TRANSCEIVE_WUPA == transceiveMode)
    {
        error = as3911Transceive(NULL, 0, response, maxResponseLength,
                    responseLength, timeout,
                    AS3911_TRANSMIT_WUPA);

    }
    else
    {
        /* Handle unkown parameter error here. */
        return EMV_HAL_ERR_PARAM;
    }

    return emvioConvertErrorCode(error);
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/

static s16 emvioConvertErrorCode(s16 as3911ErrorCode)
{
    if (AS3911_NO_ERROR == as3911ErrorCode)
        return EMV_HAL_ERR_OK;
    else if (AS3911_TIMEOUT_ERROR == as3911ErrorCode)
        return EMV_HAL_ERR_TIMEOUT;
    else if (AS3911_OVERFLOW_ERROR == as3911ErrorCode)
        return EMV_HAL_ERR_OVERFLOW;
    else
        return EMV_HAL_ERR_ENCODING;
}

s8 emvHalSetAs3911TypeBModulationMode(AS3911ModulationLevelMode_t modulationLevelMode, const void *modulationLevelModeData)
{
	emvHalTypeBModulationLevelMode = modulationLevelMode;
	emvHalTypeBModulationLevelModeData = modulationLevelModeData;
	
	return ERR_NONE;
}
