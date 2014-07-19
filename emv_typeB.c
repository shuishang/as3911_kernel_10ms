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

/*! \file emv_typeB.c
 *
 * \author Oliver Regenfelder
 *
 * \brief Type B specific PICC functions.
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include <linux/stddef.h>
#include "emv_hal.h"
#include "emv_prelayer4.h"
#include "emv_poll.h"
#include "emv_main.h"
#include "emv_typeB.h"
#include "emv_standard.h"
#include "emv_error_codes.h"

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

s16 emvTypeBCardPresent(void)
{
    u8 wupb[3];
    u8 atqb[13+2];
    s8 error = EMV_HAL_ERR_OK;
    u32 responseLength = 0;

    emvHalSetStandard(EMV_HAL_TYPE_B);
    emvHalSetErrorHandling(EMV_HAL_PREACTIVATION_ERROR_HANDLING);

    /* Setup WUPB command according to section 6.3.1 of the EMV standard. */
    wupb[0] = 0x05;
    wupb[1] = 0x00;
    wupb[2] = 0x08;

    error = emvHalTransceive(wupb, sizeof(wupb), atqb, sizeof(atqb), &responseLength, EMV_FWT_ATQB_PCD, EMV_HAL_TRANSCEIVE_WITH_CRC);

    /* Any response shall be taken as a card presence indication. */
    if (EMV_HAL_ERR_TIMEOUT == error)
        return 0;
    else
        return 1;
}

s16 emvTypeBAnticollision(EmvPicc_t *picc)
{
    u8 wupb[3];
    u8 atqb[13 + 2];
    s8 error = EMV_ERR_OK;
    u32 index = 0;
    u32 responseLength = 0;

    emvHalSetStandard(EMV_HAL_TYPE_B);
    emvHalSetErrorHandling(EMV_HAL_PREACTIVATION_ERROR_HANDLING);

    /* Setup WUPB command according to section 6.3.1 of the EMV standard. */
    wupb[0] = 0x05;
    wupb[1] = 0x00;
    wupb[2] = 0x08;

    error = emvPrelayer4Transceive(wupb, sizeof(wupb), atqb, sizeof(atqb), &responseLength,
                EMV_FWT_ATQB_PCD, EMV_HAL_TRANSCEIVE_WITH_CRC);

    if (error == EMV_ERR_TIMEOUT)
        return EMV_ERR_TIMEOUT;
    else if (error != EMV_ERR_OK)
        return EMV_ERR_COLLISION;
    else if (responseLength != (12 + 2))
        return EMV_ERR_PROTOCOL;

    if (atqb[0] != 0x50)
        return EMV_ERR_PROTOCOL;

    /* Copy the received PUPI into the EMVPicc uid field. */
    for (index = 0; index < 4; index++)
        picc->uid[index] = atqb[1 + index];
    picc->uidLength = 4;

    /* Parse bitrate definition fields. */
    picc->dPiccPcd = (atqb[9] & 0x70) >> 4;
    picc->dPcdPicc = (atqb[9] & 0x07);

    /* Parse and check FSC field. */
    picc->fsci = atqb[10] >> 4;

    if (picc->fsci < EMV_FSCI_MIN_PCD)
        return EMV_ERR_PROTOCOL;
    else if (picc->fsci > 8)
        picc->fsci = 8;

    /* Parse and check ISO14443-4 conformance bit. */
    if ((atqb[10] & 0x01) != 0x01)
        return EMV_ERR_PROTOCOL;

    /* Parse and check FWI. */
    picc->fwi = atqb[11] >> 4;
    if (picc->fwi > EMV_FWI_MAX_PCD)
        return EMV_ERR_PROTOCOL;

    /* Parse and check SFGI. */
    if (responseLength == (13 + 2))
    {
        /* Received extented ATQB. */
        picc->sfgi = atqb[12] >> 4;

        if (picc->sfgi > EMV_SFGI_MAX_PCD)
            return EMV_ERR_PROTOCOL;
    }
    else
        picc->sfgi = EMV_SFGI_DEFAULT;

    picc->activate = emvTypeBActivation;
    picc->remove = emvTypeBRemove;

    return 0;
}

int emvTypeBActivation(EmvPicc_t *picc)
{
    u8 attrib[9];
    u8 response[1+2];
    s8 error = EMV_ERR_OK;
    u32 responseLength = 0;
    u32 timeoutInCarrierCycles = 0;
    u32 index = 0;
    
    emvHalSetErrorHandling(EMV_HAL_LAYER4_ERROR_HANDLING);

    attrib[0] = 0x1D;

    for (index = 0; index < 4; index++)
        attrib[1 + index] = picc->uid[index];

    attrib[5] = 0x00;

    /* Set FSDI and select 106 kBit/s datarate in both directions. */
    attrib[6] = EMV_FSDI_MIN_PCD;
    attrib[7] = 0x01;
    attrib[8] = 0x00;

    /* Calculate timeout from FWI in milliseconds. */
    timeoutInCarrierCycles = (U32_C(4096) + 384) << picc->fwi;

    error = emvPrelayer4Transceive(attrib, 9, response, sizeof(response), &responseLength,
                timeoutInCarrierCycles, EMV_HAL_TRANSCEIVE_WITH_CRC);

    if (error != EMV_ERR_OK)
        return error;

    /* The Attrib response must have at least 1 databyte and 2 crc bytes. Any
     * shorter response without transmission errors (but with possible CRC errors)
     * must be considered a protocl error accroding to FIME testcase TB 306.12
     * failure description.
     */
    if (responseLength < 3)
        return EMV_ERR_PROTOCOL;

    /* The CID must be 0000b. */
    if ((response[0] & 0x0F) != 0x00)
        return EMV_ERR_PROTOCOL;

    return EMV_ERR_OK;
}

s16 emvTypeBRemove(EmvPicc_t *picc)
{
    u8 numWupbWithoutResponse;

    /* Do not ignore any transmission errors. */
    emvHalSetErrorHandling(EMV_HAL_PREACTIVATION_ERROR_HANDLING);

    /* Reset operating field. */
    emvHalResetField();

    /* Wait t_p milliseconds. */
    emvHalSleepMilliseconds(EMV_T_P);
    
    numWupbWithoutResponse = 0;
    while (numWupbWithoutResponse < 3)
    {
        if (emvStopRequestReceived())
            return EMV_ERR_STOPPED;

        if (emvTypeBCardPresent())
        {
            /* Wait t_p milliseconds. */
            emvHalSleepMilliseconds(EMV_T_P);

            numWupbWithoutResponse = 0;
        }
        else
            numWupbWithoutResponse++;
    }

    return EMV_ERR_OK;
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
