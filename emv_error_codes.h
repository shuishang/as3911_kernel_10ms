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
 
/*! \file emv_error_codes.h
 *
 * \author Oliver Regenfelder
 *
 * \brief Error codes specific to the EMV code.
 */

#ifndef EMV_ERROR_CODES_H
#define EMV_ERROR_CODES_H

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

/*! \name EMV module error codes
 *****************************************************************************
 * Error codes used by the EMV module.
 *****************************************************************************
 */
/**@{*/
/*! \ingroup emv
 */
#define EMV_ERR_OK               0 /*!< No error occured. */
#define EMV_ERR_COLLISION       -1 /*!< Received a card response with a bit collision. */
#define EMV_ERR_PROTOCOL        -2 /*!< Received a card response with a protocol error. */
#define EMV_ERR_TRANSMISSION    -3 /*!< Received a card response with a transmission error. */
#define EMV_ERR_TIMEOUT         -4 /*!< Timeout occured while waiting for a card response. */
#define EMV_ERR_INTERNAL        -5 /*!< EMV software stack internal error. */
#define EMV_ERR_STOPPED         -6 /*!< Stop current operation request received. */
/**@}*/

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

#endif /* EMV_ERROR_CODES_H */
