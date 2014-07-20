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

/*! \file as3911_io.c
 *
 * \author Oliver Regenfelder
 *
 * \brief AS3911 SPI communication.
 *
 * Implementation of the AS3911 SPI communication. The PIC is set to IPL 7 to disable
 * interrupts while accessing the SPI.
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include <linux/string.h>
#include "as3911_io.h"
#include "as3911_def.h"
#include "errno.h"
#include "main.h"
//#include "spi_driver.h"

/*
******************************************************************************
* DEFINES
******************************************************************************
*/
#define AS3911_SPI_ADDRESS_MASK         (0x3F)
#define AS3911_SPI_CMD_READ_REGISTER    (0x40)
#define AS3911_SPI_CMD_WRITE_REGISTER   (0x00)
#define AS3911_SPI_CMD_READ_FIFO        (0xBF)
#define AS3911_SPI_CMD_WRITE_FIFO       (0x80)
#define AS3911_SPI_CMD_DIREC_CMD        (0xC0)

/*
******************************************************************************
* MACROS
******************************************************************************
*/
//#define AS3911_SEN_ON() { _LATB8 = 0; }
//#define AS3911_SEN_OFF() { _LATB8 = 1; }

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
#define  I_AS3911_FD 0
/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/


s8 as3911WriteRegister(u8 address, u8 data)
{
	s8 error = ERR_NONE;
	u8 as3911WriteCommand[2] = { address & AS3911_SPI_ADDRESS_MASK, data };

	error = quck_write_printk( I_AS3911_FD, as3911WriteCommand, 2 );	

	if (ERR_NONE != error)
	{
		return ERR_IO;
	}
	else
	{
		return ERR_NONE;
	}
	
}

s8 as3911ReadRegister(u8 address, u8 *data)
{
	s8 error = ERR_NONE;
	u8 as3911ReadCommand[ 2 ] = { ( AS3911_SPI_CMD_READ_REGISTER | (address & AS3911_SPI_ADDRESS_MASK)),0 };
	
	error = quck_read_printk( I_AS3911_FD, as3911ReadCommand, 1 );	
	if (ERR_NONE != error)
	{
		//printk( "error as3911ReadCommand: %X\n",as3911ReadCommand[1] );
		return ERR_IO;
	}
	else
	{	
		//printk( "as3911ReadCommand: %X\n",as3911ReadCommand[1] );
		*data = as3911ReadCommand[ 1 ];
		return ERR_NONE;
	}
}

s8 as3911WriteTestRegister(u8 address, u8 data)
{
    s8 error = ERR_NONE;
//    int current_cpu_ipl = 0;
 //   u8 as3911WriteCommand[3] = { AS3911_SPI_CMD_DIREC_CMD | AS3911_CMD_TEST_ACCESS, address & AS3911_SPI_ADDRESS_MASK, data };


    if (ERR_NONE != error)
        return ERR_IO;
    else
        return ERR_NONE;
}

s8 as3911ReadTestRegister(u8 address, u8 *data)
{
    s8 error = ERR_NONE;
 //   int current_cpu_ipl = 0;
//    u8 as3911ReadCommand[2] = { AS3911_SPI_CMD_DIREC_CMD | AS3911_CMD_TEST_ACCESS, AS3911_SPI_CMD_READ_REGISTER | (address & AS3911_SPI_ADDRESS_MASK) };

    if (ERR_NONE != error)
        return ERR_IO;
    else
        return ERR_NONE;
}

s8 as3911ModifyRegister(u8 address, u8 mask, u8 data)
{
    s8 error = ERR_NONE;
    u8 registerValue = 0;

    error |= as3911ReadRegister(address, &registerValue);
    registerValue = (registerValue & ~mask) | data;
    error |= as3911WriteRegister(address, registerValue);

    if (ERR_NONE == error)
        return ERR_NONE;
    else
        return ERR_IO;
}

s8 as3911ContinuousWrite(u8 address, const u8 *data, u8 length)
{
	s8 error = ERR_NONE;
	u8 ucaDataBuffer[ 1024 ] = { 0 };
	u8 as3911WriteCommand = AS3911_SPI_CMD_WRITE_REGISTER | (address & AS3911_SPI_ADDRESS_MASK);
	
	if ( length == 0 )
	{
		return ERR_NONE;
	}
	
	ucaDataBuffer[ 0 ] = as3911WriteCommand;
	memcpy( &ucaDataBuffer[ 1 ], data, length );
	
	error = quck_write_printk( I_AS3911_FD, ucaDataBuffer, length+1 );

	if (ERR_NONE != error)
	{
		return ERR_IO;
	}
	else
	{
		return ERR_NONE;
	}
    
}

s8 as3911ContinuousRead(u8 address, u8 *data, u8 length)
{
	s8 error = ERR_NONE;
	u8 as3911ReadCommand = AS3911_SPI_CMD_READ_REGISTER | (address & AS3911_SPI_ADDRESS_MASK);
	u8 ucaDataBuffer[ 1024 ] = { 0 };

	ucaDataBuffer[ 0 ] = as3911ReadCommand;
	error = quck_read_printk( I_AS3911_FD, ucaDataBuffer, length);

	if (ERR_NONE != error)
	{
		return ERR_IO;
	}
	else
	{
		memcpy( data, &ucaDataBuffer[ 1 ] , length );
		return ERR_NONE;
	}
}

s8 as3911WriteFifo(const u8 *data, u8 length)
{
	s8 error = ERR_NONE;
	u8 as3911WriteFifoCommand = AS3911_SPI_CMD_WRITE_FIFO;
	u8 ucaDataBuffer[ 1024 ] = { 0 };
	
	if ( 0 == length )
	    return ERR_NONE;

	ucaDataBuffer[ 0 ] = as3911WriteFifoCommand;
	memcpy( ucaDataBuffer + 1, data, length );
	error = quck_write_printk( I_AS3911_FD, ucaDataBuffer, length+1 );

	if ( ERR_NONE != error )
	    return ERR_IO;
	else
	    return ERR_NONE;

}

s8 as3911ReadFifo(u8 *data, u8 length)
{
	s8 error = ERR_NONE;
	u8 as3911ReadFifoCommand = AS3911_SPI_CMD_READ_FIFO;
	u8 ucaDataBuffer[ 1024 ] = { 0 };
	
	if (0 ==  length  )
	{
		return ERR_NONE;
	}

	ucaDataBuffer[ 0 ] = as3911ReadFifoCommand;
	error = quck_read_printk( I_AS3911_FD, ucaDataBuffer, length);

	if (ERR_NONE != error)
	{
		return ERR_IO;
	}
	else
	{	
		if (  data != NULL )
		{
			memcpy( data, &ucaDataBuffer [ 1 ], length );
			return ERR_NONE;
		}
	}
	return ERR_NONE;
}

s8 as3911ExecuteCommand(u8 directCommand)
{
    s8 error = ERR_NONE;
    u8 as3911DirectCommand = AS3911_SPI_CMD_DIREC_CMD | (directCommand & AS3911_SPI_ADDRESS_MASK);

    error = quck_write_printk( I_AS3911_FD, &as3911DirectCommand, 1 );

    if (ERR_NONE != error)
        return ERR_IO;
    else
        return ERR_NONE;
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
