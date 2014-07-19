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
  
/*! \file emv_poll.h
 *
 * \author Oliver Regenfelder
 *
 * \brief EMV compliant polling and collision detection.
 */

#ifndef EMV_POLL_H
#define EMV_POLL_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

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

/*!
 *****************************************************************************
 * \brief Perform a single iteration of the polling loop without field reset.
 *
 * \note This function is needed for some of the EMV L1 certification analog
 * tests and should not be used to implement the polling loop of an EMV
 * compliant terminal.
 *****************************************************************************
 */
void emvPollSingleIteration(void);

/*!
 *****************************************************************************
 * \brief Perform an EMV polling loop as defined in [CCP, section 9.2].
 *
 * This function polls until the presence of one or more cards
 * is detected or a stop request is received. The polling flags are set
 * according to the detected cards prior to returning.
 *
 * \note This function blocks until at least one card is detected or a
 * stop is requested (emvStopTerminalApplication()).
 *
 * \note emvCollisionDetection() must be called after emvPoll() to
 * perform the collision detection for the cards found.
 *
 * \return EMV_ERR_OK: At least one card is present in the field.
 * \return EMV_ERR_STOPPED: A stop request has been received.
 *****************************************************************************
 */
s16 emvPoll(void);

/*!
 *****************************************************************************
 * \brief Perform collision detection.
 *
 * \note This function does NOT include the required wait of T_P milliseconds
 * before the first PCD request is send (see [CCP v2.01, PCD 9.3.2.1]). So the
 * caller must assure this wait is performed between the return of emvPoll()
 * and the call to emvCollisionDetection().
 *
 * \param[out] picc The picc struct will be initialized with the uid
 * of the detected card if only one card is present in the field.
 *
 * \return EMV_ERR_OK: No error, a single card has been found and selected.
 * \return EMV_ERR_COLLISION: A collision occured during card singulation.
 * \return EMV_ERR_TIMEOUT: No card found, or a timeout occured at some point
 * during the collision detection.
 *****************************************************************************
 */
s16 emvCollisionDetection(EmvPicc_t *picc);

#endif /* EMV_POLL_H */
