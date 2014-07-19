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
 
/*! \file emv_typeB.h
 *
 * \author Oliver Regenfelder
 *
 * \brief Type B specific PICC functions.
 */

#ifndef EMV_TYPEB_H
#define EMV_TYPEB_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "ams_types.h"

#include "emv_picc.h"

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

/*! \ingroup emvCardTechnologySpecific
 *****************************************************************************
 * \brief Check for the presence of ISO14443-B cards.
 *
 * \return 0: No type B card(s) detected.
 * \return 1: Type B card(s) detected.
 *****************************************************************************
 */
s16 emvTypeBCardPresent(void);

/*! \ingroup emvCardTechnologySpecific
 *****************************************************************************
 * \brief Perform an ISO14443-B anticollision.
 *
 * \note: This function does NOT include the required wait of T_P milliseconds
 * before the first WUPB command is send (see [CCP, PCD 9.3.2.1]).
 *
 * \note The content of \a picc maybe altered even if no card is singulated or
 * an error occures.
 *
 * \param[out] picc The PUPI and all other data fields are initialized based
 * on the ATTRIB response received if a single card is found.
 *
 * \return EMV_ERR_OK: A single card has been found and selected.
 * \return EMV_ERR_STOPPED: An asynchronous stop request has been received.
 * \return EMV_ERR_TIMEOUT: A timeout occured during the anticollsion.
 * \return EMV_ERR_COLLISION: A collison occured during the anticollison.
 * \return EMV_ERR_PROTOCOL: A protocol error occured during the anticollision.
 *****************************************************************************
 */
s16 emvTypeBAnticollision(EmvPicc_t *picc);

/*! \ingroup emvCardTechnologySpecific
 *****************************************************************************
 * \brief Perform an ISO14443-B activation.
 *
 * \note This function does return immediately after the ATS has been
 * processed. If an SFGT is requested by the card then this must be taken care
 * of by the caller.
 *
 * \param[in] picc The PICC to activate.
 * 
 * \return EMV_ERR_OK: The PICC has been successfully activated.
 * \return EMV_ERR_STOPPED: An asynchronous stop request has been received.
 * \return EMV_ERR_TIMEOUT: A timeout occured during the activation.
 * \return EMV_ERR_TRANSMISSION: A transmission error occured during the
 * activation.
 * \return EMV_ERR_PROTOCOL: A protocol error occured during the activation.
 *****************************************************************************
 */
int emvTypeBActivation(EmvPicc_t *picc);

/*! \ingroup emvCardTechnologySpecific
 *****************************************************************************
 * \brief Perform an ISO14443-B PICC removal.
 *
 * Performs an ISO14443-B PICC removal in accordance with [CCP v2.01,
 * section 9.5].
 *
 * \note Blocks until the card is physically removed from the field, or a
 * stop request is received (emvStopTerminalApplication()).
 *
 * \param[in] picc PICC to remove from the field.
 *
 * \return EMV_NO_ERROR: No error, Card removal successful.
 * \return EMV_ERR_STOPPED: An asynchronous stop request has been received.
 *****************************************************************************
 */
s16 emvTypeBRemove(EmvPicc_t *picc);

#endif /* EMV_TYPEB_H */
