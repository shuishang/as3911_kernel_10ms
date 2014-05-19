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
 *      PROJECT:   AS1130 MCU board firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file logger.c
 *
 *  \author Wolfgang Reichart (based on work of Christian Eisendle)
 *
 *  \brief debug log output utility.
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "logger.h"

#include "errno.h"
#include <stdarg.h>
#include <stdarg.h>

#include <stdio.h>
/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/
#define NO_OF_NIBBLES 4
#define NULL_CHARACTER '\0'
#define ALPHA_LC_TO_INTEGER 87  /*!< small case conversion value              */
#define ALPHA_UC_TO_INTEGER 55  /*!< upper case conversion value              */
#define NUMCHAR_TO_INTEGER 48
#define CON_putchar uartTxByte

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/

//#if (USE_LOGGER == LOGGER_ON)

void dbgHexDump(unsigned char *buffer, u16 length)
{
    u16 i, rest;

    rest = length % 8;

    if (length >= 8)
    {
        for (i = 0; i < length/8; i++)
        {
            dbgLog("%hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx\n",
                     buffer[i*8],
                     buffer[i*8+1],
                     buffer[i*8+2],
                     buffer[i*8+3],
                     buffer[i*8+4],
                     buffer[i*8+5],
                     buffer[i*8+6],
                     buffer[i*8+7]
                    );
        }
    }
    if (rest > 0)
    {
        for (i = length/8 * 8; i < length; i++)
        {
            dbgLog("%hhx ", buffer[i]);
        }
        dbgLog("\n");
    }
}

//#endif //#if USE_LOGGER == LOGGER_ON

