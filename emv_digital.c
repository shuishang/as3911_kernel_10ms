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

/*! \file emv_digital.c
 *
 * \author Oliver Regenfelder
 *
 * \brief EMV TTA L1 digital application callback.
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include <stddef.h>

#include "emv_display.h"
#include "emv_layer4.h"
#include "emv_response_buffer.h"

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

/*! EMV digital test application initial SELECT PPSE command APDU. */
static const u8 emvSelectppseApdu[] = { 0x00, 0xA4, 0x04, 0x00, 0x0E
    , '2', 'P', 'A', 'Y', '.', 'S', 'Y', 'S', '.', 'D', 'D', 'F', '0', '1', 0x00 };

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

s16 emvDigitalApplication()
{
    s16 error = EMV_ERR_OK;
    size_t responseLength = 0;

    emvDisplayCAPDU(emvSelectppseApdu, sizeof(emvSelectppseApdu));

    error = emvTransceiveApdu(emvSelectppseApdu, sizeof(emvSelectppseApdu)
        , emvResponseBuffer, EMV_RESPONSE_BUFFER_SIZE, &responseLength);

    if (EMV_ERR_OK == error)
        emvDisplayRAPDU(emvResponseBuffer, responseLength);
    else
        emvDisplayError(error);

    while ((responseLength >= 3) && (emvResponseBuffer[1] != 0x70))
    {
        emvDisplayCAPDU(emvResponseBuffer, responseLength - 2);

        error = emvTransceiveApdu(emvResponseBuffer, responseLength - 2
            , emvResponseBuffer, EMV_RESPONSE_BUFFER_SIZE, &responseLength);

        if (EMV_ERR_OK == error)
            emvDisplayRAPDU(emvResponseBuffer, responseLength);
        else {
            emvDisplayError(error);
            return error;
        }
    }

    return error;
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
