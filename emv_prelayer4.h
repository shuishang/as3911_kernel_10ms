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
 
/*! \file emv_prelayer4.h
 *
 * \author Oliver Regenfelder
 *
 * \brief EMV compliant data transmission prior to activation of the PICC.
 */

#ifndef EMV_PRELAYER4_H
#define EMV_PRELAYER4_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "ams_types.h"

#include "emv_hal.h"

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
 * \brief Send a request to a type A or B card.
 *
 * Send a request to a type A or B card with exception handling according to
 * [CCP v2.01, 9.6 Exception Processing].
 *
 * \param request Pointer to the request.
 * \param requestLength Length of the request in bytes.
 * \param response Pointer to the array storing the response.
 * \param maxResponseLength Maximum length of the response in bytes.
 * \param responseLength Set to the length of the received response in bytes.
 * \param timeout Timeout in carrier cycles.
 * \param transceiveMode EMV IO layer transceive mode to use for the request.
 *
 * \return EMV_ERR_OK: An error free response has been successfully received.
 * \return EMV_ERR_STOPPED: An asynchronous stop request has been received.
 * \return EMV_ERR_TIMEOUT: No card response has been received.
 * \return EMV_ERR_TRANSMISSION: An errornous card response has been received.
 *****************************************************************************
 */
s8 emvPrelayer4Transceive(const u8 *request, u32 requestLength,
        u8 *response, u32 maxResponseLength, u32 *responseLength,
        u32 timeout, EmvHalTransceiveMode_t transceiveMode);

#endif /* EMV_PRELAYER4_H */
