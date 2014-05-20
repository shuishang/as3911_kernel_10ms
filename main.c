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
 *      PROJECT:   AS3911 firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file main.c
 *
 *  \author F. Lobmaier
 *
 *  \brief main application file
 *
 *  This is the implementation file for the main loop
 *
 */

/*! \mainpage
 *
 * \section mainpage_sec1 Introduction
 *
 * The AS3911 EMV L1 certification firmware provides an example implementation
 * of the EMV contactless communication protocol specification. It was used to
 * get an EMV L1 certification for the AS3911 EMV demo system from an accredited
 * test laboratory and can be used directly as a software module for your own
 * EMV L2 applications or as a guideline to implement your own EMV
 * communication protocol stack.
 *
 * \section mainpage_sec2 Software Structure
 *
 * \subsection mainpage_sec2_1 Main Modules
 *
 * The firmware consists of two main modules, the AS3911 module and the EMV
 * module, respectively. The AS3911 module provides a driver for the AS3911
 * chip with support for special features required to implement EMV (e.g.
 * dynamic modulation depth adjustment). The EMV module provides an
 * implementation of the EMV contactless communication protocol stack on top
 * of the AS3911 driver module. The following picutre gives an overview off
 * the internal structure of these two modules.
 *
 * <table align="center" cellpadding = 20 cellspacing = 0 rules="none">
 * <tr>
 *   <td align="center" colspan="4" bgcolor="#AAAAFC">\link emvMain EMV Layer 4 and Terminal Main Loop</td>
 * </tr>
 * <tr>
 *   <td align="center" colspan="3" bgcolor="#78AAFC">\link emv EMV Card Technology Specific Functions</td>
 *   <td align="center" colspan="1" bgcolor="#AAAAFC">&nbsp;</td>
 * </tr>
 * <tr>
 *   <td align="center" colspan="4" bgcolor="#34AAFF">\link emvHal EMV Hardware Abstraction Layer</td>
 * </tr>
 * <tr>
 *   <td align="center" colspan="1" bgcolor="#34AAFF">&nbsp;</td>
 *   <td align="center" colspan="3" bgcolor="#ACFC9C">\link as3911RfidCom AS3911 RFID Communication</td>
 * </tr>
 * <tr>
 *   <td align="center" colspan="1" bgcolor="#34AAFF">&nbsp;</td>
 *   <td align="center" colspan="1" bgcolor="#ACFC9C">&nbsp;</td>
 *   <td align="center" colspan="1" bgcolor="#7AFF14">&nbsp;</td>
 *   <td align="center" colspan="1" bgcolor="#7AFF14">\link as3911IrqHandling AS3911 Interrupt Handling</td>
 * </tr>
 * <tr>
 *   <td align="center" colspan="3" bgcolor="#1AAF14">\link as3911SpiIo AS3911 SPI IO</td>
 *   <td align="center" colspan="1" bgcolor="#7AFF14">&nbsp;</td>
 * </tr>
 * <tr>
 *   <td align="center" colspan="4" bgcolor="#1A7F14">AS3911 Hardware</td>
 * </tr>
 * </table>
 *
 * \subsection mainpage_sec2_2 Support Modules
 *
 * Additionally to the modules depicted above, the firmware also has some
 * modules which are not directly related to the EMV protocol stack but provide
 * utility functions which are used in various parts of the firmware. These
 * modules are:
 * \li LED driver
 * \li sleep module
 * \li USB communication with the PC
 *
 * \section mainpage_sec3 Control Flow
 *
 * After the initialization of the PIC microcontroller and the SPI driver
 * is finished the firmware will wait for commands from the PC GUI. If a
 * command is received it will be processed by the firmware and once that
 * is done the firmware will wait for the next USB command.
 */

/*! \defgroup usb USB Application Comands
 *****************************************************************************
 * \brief Handle commands received via USB
 *
 * This module implements a callback (applProcessCmd()) for the AMS base
 * firmware layer. This callback is called any time a firmware specific USB
 * command is received. The
 * callback is implemented as a dispatcher which identifies the command send by
 * the GUI, extracts any additional parameter and then executes that command
 * or calls a function which implements that command.
 *****************************************************************************
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "emv_gui.h"
#include "main.h"

#include "logger.h"
#include "as3911.h"
#include "as3911_api.h"
#include "emv_hal.h"
#include "sleep.h"

//#include "emv_hal.h"
#include "emv_typeA.h"
#include "mifare.h"
/** CONFIGURATION **************************************************/

/*
******************************************************************************
* DEFINES
******************************************************************************
*/

/*! SPI device ID of the AS3911. */
#define SPI_DEVICE_ID_AS3911 (0)
/*! Maximum size of the user supplied modulation level table. */
#define MAX_MODULATION_TABLE_SIZE 10
/*! Maximum size of the user supplied gain table. */
#define MAX_GAIN_TABLE_SIZE 10

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/

static u8 mainModulationTableX[MAX_MODULATION_TABLE_SIZE];
static u8 mainModulationTableY[MAX_MODULATION_TABLE_SIZE];
static AS3911ModulationLevelTable_t mainModulationTable = {
	0,
	&mainModulationTableX[0],
	&mainModulationTableY[0]
	};
	
static AS3911ModulationLevelAutomaticAdjustmentData_t mainModulationAutomaticAdjustmentData;

static u8 mainGainTableX[MAX_GAIN_TABLE_SIZE];
static u8 mainGainTableY[MAX_GAIN_TABLE_SIZE];
static AS3911GainTable_t mainGainTable = {
	0,
	&mainGainTableX[0],
	&mainGainTableY[0]
	};
	

static u8 emvGuiTestApplicationRequested = APPL_COM_EMV_DIGITAL;


#if (USE_LOGGER == LOGGER_ON)
u32 gBaudRate = 0UL;
#endif

/*! Version information */
const char gAS3911FwVersion[] = AS3911_FW_VERSION;

/*
******************************************************************************
* GLOBAL FUNCTION
******************************************************************************
*/

/*!
 *****************************************************************************
 * \brief Initialize the IO ports of the PIC.
 *****************************************************************************
 */
s8 picInitialize (void);


int main()
{

    u8 tbuf[32];
	u8 i,j;
	u8 err = ERR_NONE;
	EmvPicc_t picc;
    picInitialize();

    /* Reset the AS3911 */
    as3911ExecuteCommand(AS3911_CMD_SET_DEFAULT);

    /* MCU_CLK and LF MCU_CLK off, 27MHz XTAL */
    as3911WriteRegister(AS3911_REG_IO_CONF1, 0x0F);

    /* Enable Oscillator, Transmitter and receiver. */
    as3911WriteRegister(AS3911_REG_OP_CONTROL, 0xC8);
    sleepMilliseconds(5);

    /* Enable AM/PM receiver signal output on CSI/CSO. */
    // as3911WriteTestRegister(0x01, 0x03);

    /* Enable AS3911 IRQ handling. */
    AS3911_IRQ_ON();

    LOG("\n");
    LOG((const char*)gAS3911FwVersion);
    LOG("\n");


    
    while(1)
    {

        // Application-specific tasks.
        if (APPL_COM_EMV_DIGITAL == emvGuiTestApplicationRequested)
        {
            emvGuiDigital();
            emvGuiTestApplicationRequested = 0;
        }
        if (APPL_COM_EMV_PREVALIDATION == emvGuiTestApplicationRequested)
        {
            emvGuiPrevalidation();
            emvGuiTestApplicationRequested = 0;
        }
    }//end while

    return 0;
}

/*!
 *****************************************************************************
 * \brief Get the version string of the firmware.
 *
 * \return The version string of the firmware.
 *****************************************************************************
 */
const char * applGetFirmwareVersion(void)
{
    return gAS3911FwVersion;
}



/*!
 *****************************************************************************
 * \brief Display the value of an AS3911 test register on the debug output.
 *
 * \param[in] address Address of the register whose value shall be display.
 *****************************************************************************
 */


/*! \ingroup usb
 *****************************************************************************
 * \brief Process application commands received from the GUI via USB.
 *
 * \note This function is called by the USB stack when a command of type
 * applicaiton command is received via USB.
 * 
 * \param[in] rxData Application command data received from the GUI.
 * \param[in] rxSize Length of the application command data in bytes.
 * \param[out] txData Set to the data which shall be send back to the GUI as
 * response.
 * \param[out] txSize Number of bytes stored in \a txData as response for the
 * GUI.
 *
 * \return ERR_PARAM: Data contained in \a rxData is invalid.
 * \return ERR_NONE: No Error, eventual response data is available in
 * \a txData.
 *****************************************************************************
 */
u8 applProcessCmd (u8 const * rxData, const u8 rxSize, u8 * txData, u8 * txSize)
{
    s8 retVal = ERR_NONE;
	const u8 *rxByte;
	u8 modulationDepthMode = 0;
	u8 gainMode = 0;
	int index = 0;

	*txSize = 0;
    switch (rxData[0])
    {
    case APPL_COM_EMV_TOGGLE_CARRIER:
        emvGuiToggleCarrier();
        break;
    case APPL_COM_EMV_POLL:
        emvGuiPoll();
        break;
    case APPL_COM_EMV_RESET:
        emvGuiReset();
        break;
    case APPL_COM_EMV_WUPA:
        emvGuiWupa();
        break;
    case APPL_COM_EMV_WUPB:
        emvGuiWupb();
        break;
    case APPL_COM_EMV_RATS:
        emvGuiRats();
        break;
    case APPL_COM_EMV_ATTRIB:
        emvGuiAttrib();
        break;
    case APPL_COM_EMV_PREVALIDATION:
        emvGuiTestApplicationRequested = APPL_COM_EMV_PREVALIDATION;
        break;
    case APPL_COM_EMV_DIGITAL:
        emvGuiTestApplicationRequested = APPL_COM_EMV_DIGITAL;
        break;
    case APPL_COM_EMV_STOP:
        emvGuiStop();
        break;
    case APPL_COM_EMV_INIT:
        /* EMV Mode initialization command. */
		LOG("EMV: analog settings: ");
		for (index = 0; index < rxSize; index++)
			LOG("%hhx", rxData[index]);
		LOG("\n");

		/* Voltage Regulator setup. */
        as3911ModifyRegister(AS3911_REG_IO_CONF2, 0x80, (rxData[1] & 0x10) << 3);
        if (0x00 == (rxData[1] & 0x0F))
        {
            /* Disable the voltage regulator. */
            as3911ModifyRegister(AS3911_REG_IO_CONF2, 0x40, 0x40);
        }
        else
        {
            /* Enable the voltage regulator. */
            as3911ModifyRegister(AS3911_REG_IO_CONF2, 0x40, 0x00);
            as3911ModifyRegister(AS3911_REG_VSS_REGULATOR_CONF, 0xF8, 0x80 | ((rxData[1] & 0x0F) << 3));
        }

        /* Antenna trim setup. */
        as3911ModifyRegister(AS3911_REG_ANT_CAL_CONF, 0xF8, 0x80 | ((rxData[2] & 0x0F) << 3));

        /* Receive channel setup. */
        as3911ModifyRegister(AS3911_REG_OP_CONTROL, 0x30, ((rxData[3] & 0x06) << 3));
        as3911ModifyRegister(AS3911_REG_RX_CONF1, 0x80, ((rxData[3] & 0x01) << 7));

        /* First stage gain reduction. */
        as3911ModifyRegister(AS3911_REG_RX_CONF3, 0xFC, ((rxData[4] & 0x07) << 5) | ((rxData[5] & 0x07) << 2));

        /* Second/Third stage gain reduction. */
        as3911WriteRegister(AS3911_REG_RX_CONF4, ((rxData[6] & 0x0F) << 4) | (rxData[7] & 0x0F));
        as3911ExecuteCommand(AS3911_CMD_CLEAR_SQUELCH);
		
        /* Test output. */
        as3911WriteTestRegister(AS3911_REG_ANALOG_TEST, rxData[8] & 0x0F);

		/* Automatic gain control and squelch. */
		as3911ModifyRegister(AS3911_REG_RX_CONF2, 0x1F, rxData[9]);
		        
		/* Gain adjustment based on lookup table. */
		/* Readout gain lookup table. */
		rxByte = &rxData[10];
		gainMode = *rxByte++;
		mainGainTable.length = *rxByte++;
		for (index = 0; index < mainGainTable.length; index++)
		{
			mainGainTableX[index] = *rxByte++;
			mainGainTableY[index] = *rxByte++;
		}
		LOG("EMV: gain reduction table length %hhd\n", mainGainTable.length);
		for (index = 0; index < mainGainTable.length; index++)
			LOG("EMV: gainTable[%d] = 0x%hhx, 0x%hhx\n", index, mainGainTable.x[index], mainGainTable.y[index]);

		if (0x00 == gainMode)
		{
			as3911SetGainMode(AS3911_GAIN_FIXED, NULL);
			LOG("EMV: using fixed gain reduction\n");
		}
		else if (0x01 == gainMode)
		{
                    int index;
			as3911SetGainMode(AS3911_GAIN_FROM_AMPLITUDE, &mainGainTable);
			LOG("EMV: using table based gain reduction\n");
		}
		else
			LOG("EMV: Error: unkown adaptive gain mode byte: 0x%hhx\n", gainMode);
		
		/* Read ISO14443B modulation depth mode byte. */
		modulationDepthMode = *rxByte++;
		if (0x00 == modulationDepthMode)
		{
			u8 modulationDepth = *rxByte++;
			as3911WriteRegister(AS3911_REG_AM_MOD_DEPTH_CONF, 0x80);
			as3911WriteRegister(AS3911_REG_RFO_AM_ON_LEVEL, modulationDepth);
			LOG("EMV: using fixed am driver strength %hhx\n", modulationDepth);
			emvHalSetAs3911TypeBModulationMode(AS3911_MODULATION_LEVEL_FIXED, NULL);
		}
		else if (0x01 == modulationDepthMode)
		{
			u8 adjustmentTargetValue = *rxByte++;
			u8 adjustmentDelay = *rxByte++;
			
			as3911WriteRegister(AS3911_REG_AM_MOD_DEPTH_CONF, 0x7F & adjustmentTargetValue);
			LOG("EMV: using automatic modulation depth adjustment\n");
			LOG("EMV: adjustment target value: %hhX\n", adjustmentTargetValue);
			LOG("EMV: post adjustment delay: %hhx\n", adjustmentDelay);
			mainModulationAutomaticAdjustmentData.targetValue = adjustmentTargetValue;
			mainModulationAutomaticAdjustmentData.delay = adjustmentDelay;
			emvHalSetAs3911TypeBModulationMode(AS3911_MODULATION_LEVEL_AUTOMATIC, &mainModulationAutomaticAdjustmentData);
		}
		else if (0x02 == modulationDepthMode)
		{
			mainModulationTable.length = *rxByte++;
			for (index = 0; index < mainModulationTable.length; index++)
			{
				mainModulationTable.x[index] = *rxByte++;
				mainModulationTable.y[index] = *rxByte++;
			}
			
			LOG("EMV: using table based modulation depth adjustment\n");
			LOG("EMV: modulation depth adjustment table length %hhd\n", mainModulationTable.length);
			for (index = 0; index < mainModulationTable.length; index++)
				LOG("EMV: modulationTable[%d] = 0x%hhx, 0x%hhx\n", index, mainModulationTable.x[index], mainModulationTable.y[index]);
			
			/* FIXME: configuration of the mod depth conf register should be done inside the
			 * modulation level adjustment module.
			 */
			as3911WriteRegister(AS3911_REG_AM_MOD_DEPTH_CONF, 0x80);
			emvHalSetAs3911TypeBModulationMode(AS3911_MODULATION_LEVEL_FROM_AMPLITUDE, &mainModulationTable);
		}
		else
		{
			LOG("Error: unkown ISO14443B modulation depth mode byte: 0x%hhx\n", modulationDepthMode);
		}
		
        LOG("EMV: settings applied\n");
        displayRegisterValue(AS3911_REG_IO_CONF2);
        displayRegisterValue(AS3911_REG_VSS_REGULATOR_CONF);
        displayRegisterValue(AS3911_REG_ANT_CAL_CONF);
        displayRegisterValue(AS3911_REG_OP_CONTROL);
        displayRegisterValue(AS3911_REG_RX_CONF1);
        displayRegisterValue(AS3911_REG_RX_CONF2);
        displayRegisterValue(AS3911_REG_RX_CONF3);
        displayRegisterValue(AS3911_REG_RX_CONF4);
		displayRegisterValue(AS3911_REG_AM_MOD_DEPTH_CONF);
		displayRegisterValue(AS3911_REG_RFO_AM_ON_LEVEL);
        displayTestRegisterValue(AS3911_REG_ANALOG_TEST);
        break;
    case APPL_COM_REG_DUMP:
    {
        u8 regAddress = 0;
        for (regAddress = rxData[1]; regAddress < rxData[1] + rxData[2]; regAddress++)
            displayRegisterValue(regAddress);
    }
        break;
    case APPL_COM_REG_WRITE:
		LOG("REG: 0x%hhx: set to 0x%hhx\n", rxData[1], rxData[2]);
        as3911WriteRegister(rxData[1], rxData[2]);
        break;
	case APPL_COM_DIRECT_COMMAND:
		LOG("Executing direct command 0x%hhx\n", rxData[1]);
		as3911ExecuteCommand(rxData[1]);
		break;
    default:
        retVal = ERR_PARAM;
        break;
    }

    return retVal;
}

s8 picInitialize (void)
{

   
    return ERR_NONE;
}
