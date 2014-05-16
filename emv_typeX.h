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
 
/*! \file emv_doc.h
 *
 * \author Oliver Regenfelder
 *
 * \brief Contains the doxygen documentation for the EMV card technology
 * specific submodule.
 */

#ifndef EMV_TYPEX_H
#define EMV_TYPEX_H

/*! \defgroup emvCardTechnologySpecific EMV Card Technology Specific Functions
 *****************************************************************************
 * \ingroup emv
 * 
 * \brief This part contains RFID card operations which are in general
 * available on every card technology but the implementation of which depends
 * on the card technology (ISO14443-A or ISO14443-B) in use.
 *
 * This module contains RFID operations which are typically available on
 * ISO14443-A and ISO14443-B cards. But, due to differences between these
 * two card technologies it is not possible to provide a single implementation
 * for both technologies.
 * 
 * This operations include:
 * - Presence detection: Used by the polling loop to detect the presence
 * of cards from a specific technology (emvTypeACardPresent(),
 * emvTypeBCardPresent()).
 * - Singulation: Used after cars of one technology only have been found
 * to ensure that only one card of that technology is present in the reader
 * field (emvTypeAAnticollision(), emvTypeBAnticollision()).
 * - Card activation: Used to activate the ISO14443-4 layer on an already
 * singulated card (emvTypeAActivation(), emvTypeBActivation).
 * - Card removal: Used to detect phydical removal of the card from the
 * reader field after a payment transaction is completed
 * (emvTypeARemoval(), emvTypeBRemoval()).
 *
 * The activation and removal operations can also be used via the callbacks
 * provided by ::EmvPicc_t after anticollision has been completed successfully.
 *
 * Additionally, this module includes a function to communicate with a
 * type A or type B card with error handling as required by the EMV specifciaton
 * prior to card activation (emvPrelayer4Transceive()).
 *****************************************************************************
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

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

#endif /* EMV_TYPEX_H */
