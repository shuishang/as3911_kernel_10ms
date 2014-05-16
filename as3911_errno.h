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
 
/*! \file as3911_errno.h
 *
 * \author Oliver Regenfelder
 *
 * \brief AS3911 module error codes.
 */

#ifndef AS3911_ERRNO_H
#define AS3911_ERRNO_H

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

/*! \name AS3911 error codes
 * This are the error codes used by the AS3911 module and its submodules.
 */

/**@{*/
/*! \ingroup as3911 */
#define AS3911_NO_ERROR                  0 /*!< Operation completed successfully. */
#define AS3911_RECEIVE_ERROR            -1 /*!< Unspecified reception error. Wherever possible a more detailed error code is returned. */
#define AS3911_COLLISION_ERROR          -2 /*!< Bit collision error. */
#define AS3911_CRC_ERROR                -3 /*!< CRC incorrect. */
#define AS3911_PARITY_ERROR             -4 /*!< Parity bit incorrect. */
 /*!
  * \brief Soft framing error.
  *
  * Some frame timing parameters have been violated, but the data has been
  * correctly received.
  */
#define AS3911_SOFT_FRAMING_ERROR       -5
/*!
 * \brief Hard framing error.
 *
 * Some frame timing parameters or signal shape requirements have been violated,
 * and the received data has been corrupted.
 */
#define AS3911_HARD_FRAMING_ERROR       -6
#define AS3911_TIMEOUT_ERROR            -7 /*!< Operation timeout out. */
#define AS3911_OVERFLOW_ERROR           -8 /*!< AS3911 FIFO or internal buffer overflow error. */
#define AS3911_UNKOWN_ERROR             -9 /*!< An unspecified error has occured. */
#define AS3911_INTERNAL_ERROR           -10 /*!< Internal error of the AS3911 software stack. */
#define AS3911_NFC_EVENT                -11 /*!< An NFC event (RF field on/off, collision) occured. */

/**}*/

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

#endif /* AS3911_ERRNO_H */
