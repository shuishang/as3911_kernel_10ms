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
 
/*! \file emv_layer4.h
 *
 * \author Oliver Regenfelder
 *
 * \brief EMV compliant ISO14443-4 data transmission.
 */

#ifndef EMV_LAYER4_H
#define EMV_LAYER4_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include <stddef.h>

#include "ams_types.h"

#include "emv_picc.h"
#include "emv_error_codes.h"

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

/*! \ingroup emvMain
 *****************************************************************************
 * \brief Initializes the ISO14443-4 protocol layer (EMV layer 4).
 *
 * Resets the ISO14443-4 protocoll stack and initializes it for communicaiton
 * with a specific card.
 *
 * \note The PICC must be in the active state for layer-4 data transmissions
 * to work.
 *
 * \param[in] picc The target PICC for ISO14443-4 data transmissions.
 *
 * \return EMV_ERR_OK: No error, operation successful.
 *****************************************************************************
 */
s16 emvInitLayer4(EmvPicc_t *picc);

/*! \ingroup emvMain
 *****************************************************************************
 * \brief Transeive an APDU.
 *
 * Send a command APDI and receives the response APDU from the PICC.
 *
 * \param[in] apdu Command APDU.
 * \param[in] apduLength Length of the command APDI in bytes.
 * \param[out] response Buffer for the response APDU.
 * \param[in] maxResponseLength Size of the response APDI buffer in bytes.
 * \param[out] responseLength The length of the received response APDU in
 * bytes.
 *
 * \return EMV_ERR_OK: No error, response APDU received.
 * \return EMV_ERR_PROTOCOL: Protocl error during reception of the response
 * APDU.
 * \return EMV_ERR_TRANSMISSION: Transmission error during reception of the
 * response APDU.
 * \return EMV_ERR_TIMEOUT: No response APDU receied, or a timeout occured during
 * reception of the response APDU.
 * \return EMV_ERR_INTERNAL: Internal buffer overflow during reception of the
 * response APDU.
 *****************************************************************************
 */
s16 emvTransceiveApdu(const u8 *apdu, size_t apduLength, u8 *response,
        size_t maxResponseLength, size_t *responseLength);

#endif /* EMV_LAYER4_H */
