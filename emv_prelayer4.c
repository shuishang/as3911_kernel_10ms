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

/*! \file emv_prelayer4.c
 *
 * \author Oliver Regenfelder
 *
 * \brief EMV compliant data transmission prior to activation of the PICC.
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "emv_prelayer4.h"

#include "emv_error_codes.h"
#include "emv_main.h"

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

s8 emvPrelayer4Transceive(const u8 *request, size_t requestLength, u8* response, size_t maxResponseLength,
    size_t *responseLength, u32 timeout, EmvHalTransceiveMode_t transceiveMode)
{
    int numTries = 0;

    while (numTries < 3)
    {
        s8 error = EMV_ERR_OK;

        if (emvStopRequestReceived())
            return EMV_ERR_STOPPED;

        error = emvHalTransceive(request, requestLength, response, maxResponseLength
            , responseLength, timeout, transceiveMode);

        if (EMV_HAL_ERR_OK == error)
            return EMV_ERR_OK;
        else if (EMV_HAL_ERR_TIMEOUT == error)
            numTries++;
        else
            return EMV_ERR_TRANSMISSION;
    }

    /* Three retries without a proper response are a timeout error. */
    return EMV_ERR_TIMEOUT;
}
