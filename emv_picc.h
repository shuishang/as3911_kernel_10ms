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
 
/*! \file emv_picc.h
 *
 * \author Oliver Regenfelder
 *
 * \brief EMV PICC abstraction.
 */

#ifndef EMV_PICC_H
#define EMV_PICC_H

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

/*! \ingroup emvMain
 *****************************************************************************
 * \brief Representaiton of a PICC for the EMV software stack.
 * 
 * Representation of a PICC for the EMV software stack. The variables \a uid
 * and \a uidLength contain valid data after successful anticollision
 * (emvCollisionDetection()). Other data members are only guaranteed to hold
 * valid data after successful activation (emvActivate()).
 *****************************************************************************
 */
typedef struct EmvPicc_s
{
    u8 uid[10]; /*!< UID or PUPI of the PICC. */
    u8 uidLength; /*!< Length of the UID/PUPI in bytes. */
    u8 fwi; /*!< Frame wait integer of the PICC. */
    u8 fsci; /*!< Frame size integer of the PICC. */
    u8 sfgi; /*!< Special frame guard time of the PICC. */
    u8 dPiccPcd; /*!< Datarate bits PICC->PCD. */
    u8 dPcdPicc; /*!< Datarate bits PCD->PICC. */

    int (*activate)(struct EmvPicc_s *picc); /*!< Activation function callback. */
    s16 (*remove)(struct EmvPicc_s *picc); /*!< Card removal function callback. */
} EmvPicc_t;

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
 * \brief Activate the PICC.
 *
 * \param[in] picc The PICC to activate.
 *
 * \return EMV_ERR_OK: No error, operation successfull.
 * \return EMV_ERR_STOPPED: A stop request was received during activation.
 * \return EMV_ERR_TIMEOUT: Timeout during PICC activation.
 * \return EMV_ERR_TRANSMISSION: Transmission error during PICC activation.
 * \return EMV_ERR_PROTOCOL: Protocol error during PICC activation.
 *****************************************************************************
 */
s16 emvActivate(EmvPicc_t *picc);


/*! \ingroup emvMain
 *****************************************************************************
 * \brief Remove the PICC.
 *
 * \note This function blocks until the PICC is physically removed from the
 * reader field or a stop request is received via emvStopTerminalApplication().
 *
 * \param[in] picc The PICC to remove.
 *
 * \return EMV_ERR_OK: No error, operation successfull.
 * \return EMV_ERR_STOPPED: A stop request was received during activation.
 *****************************************************************************
 */
s16 emvRemove(EmvPicc_t *picc);

#endif /* EMV_PICC_H */
