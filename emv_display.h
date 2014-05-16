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
  
/*! \file emv_display.h
 *
 * \author Oliver Regenfelder
 *
 * \brief Methods to display EMV data on the GUI.
 */

#ifndef EMV_DISPLAY_H
#define EMV_DISPLAY_H

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

/** EMV message code for the polling status message */
#define EMV_M_POLLING       1
/** EMV message code for the remove card status message */
#define EMV_M_REMOVE_CARD   2

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
 * \brief Display a C string on the GUI.
 *
 * \param string Pointer to a C string.
 *****************************************************************************
 */
void emvDisplayString(const char *string);

/*!
 *****************************************************************************
 * \brief Display an array of byte values.
 *
 * \param array Pointer to the array.
 * \param length Length of the array in bytes.
 *****************************************************************************
 */
void emvDisplayByteArray(const u8 *array, size_t length);

/*!
 *****************************************************************************
 * \brief Display a card UID (or PUPI) on the GUI.
 *
 * Card UIDs are displayed as byte arrays with the most significant byte
 * first.
 *
 * \param uid Pointer to the UID.
 * \param length Length of the UID in bytes.
 *****************************************************************************
 */
void emvDisplayUid(const u8 *uid, size_t length);

/*!
 *****************************************************************************
 * \brief Display an EMV error code converted to readable text on the GUI.
 *
 * \param errorCode EMV error code defined in emv_error_codes.h.
 *****************************************************************************
 */
void emvDisplayError(s16 errorCode);

/*!
 *****************************************************************************
 * \brief Display a standardized EMV Message on the GUI.
 *
 * \param messageCode Message code of the message to display on the GUI.
 *****************************************************************************
 */
void emvDisplayMessage(s16 messageCode);

/*!
 *****************************************************************************
 * \brief Display a command APDU on the GUI.
 *
 * \param apdu Command APDU to display.
 * \param length Length of the command APDU in bytes.
 *****************************************************************************
 */
void emvDisplayCAPDU(const u8 *apdu, size_t length);

/*!
 *****************************************************************************
 * \brief Display a response APDU on the GUI.
 *
 * \param apdu Response APDU to display.
 * \param length Length of the response APDU in bytes.
 *****************************************************************************
 */
void emvDisplayRAPDU(const u8 *apdu, size_t length);

#endif /* EMV_DISPLAY_H */
