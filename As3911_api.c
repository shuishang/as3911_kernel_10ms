/* /////////////////////////////////////////////////////////////////////////////////////////////////
//                     Copyright (c) Strong Lion
//
//         All rights are reserved. Reproduction in whole or in part is
//        prohibited without the written consent of the copyright owner.
//    Philips reserves the right to make changes without notice at any time.
//   Philips makes no warranty, expressed, implied or statutory, including but
//   not limited to any implied warranty of merchantability or fitness for any
//  particular purpose, or that the use will not infringe any third party patent,
//   copyright or trademark. Philips must not be liable for any loss or damage
//                            arising from its use.
///////////////////////////////////////////////////////////////////////////////////////////////// */
#include <stdio.h>
#include <fcntl.h> 
#include <errno.h>
#include <fcntl.h>
#include <termio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <sys/timeb.h>
#include <termios.h>
#include <signal.h>
#include <sys/types.h>
#include  <linux/fs.h>


/*AS3911应用调用接口*/
//#include "emv_picc.h"
#include "AS3911_api.h"
#include "as3911_gain_adjustment.h"
#include "as3911_def.h"
#include "ams_types.h"
#include "mifare.h"
#include "emv_error_codes.h"
#include "emv_hal.h"
#include "emv_response_buffer.h"
#include "emv_typeA.h"
#include "emv_typeB.h"
#include "emv_standard.h"
#include "emv_main.h"
#include "sleep.h"
#include "emv_picc.h"

#define AS3911_PW_CTL	"/proc/strong_lion/power_manager/rfid"

/**************************************************************************
*                            模块内部宏定义                               *
**************************************************************************/
#define FALSE                               0
#define TRUE                                 1
#define MIFARE_DEFAULT_READER_NONCE             0xAA55AA55

#define EMV_TYPEA_FDT_9     1250

/*! Bit mask for the UID size field of the first ATQA byte. */
#define EMV_ATQA_UID_SIZE_MASK      0xC0
/*! ATQA UID size field value for a single size UID. */
#define EMV_ATQA_UID_SIZE_SINGLE    0x00
/*! ATQA UID size field value for a double size UID. */
#define EMV_ATQA_UID_SIZE_DOUBLE    0x40
/*! ATQA UID size field value for a triple size UID. */
#define EMV_ATQA_UID_SIZE_TRIPLE    0x80

/*!
 *****************************************************************************
 * Invalid ATQA UID size field value. A standard conforming card
 * should never return this value.
 *****************************************************************************
 */
#define EMV_ATQA_UID_SIZE_INVALID   0xC0

/*! Value of the SEL byte of a cascade level 1 anticollision or select request.*/
#define EMV_SEL_CL1     0x93
/*! Value of the SEL byte of a cascade level 2 anticollision or select request.*/
#define EMV_SEL_CL2     0x95
/*! Value of the SEL byte of a cascade level 3 anticollision or select request.*/
#define EMV_SEL_CL3     0x97

/*! NVB (number of valid bits) byte value for an ISO14443-A anticollision request. */
#define EMV_ANTICOLLISION_NVB   0x20
/*! NVB (number of valid bits) byte value for an ISO14443-A select request. */
#define EMV_SELECT_NVB          0x70

/*! ISO14434-A anticollision response cascade tag value. */
#define EMV_CASCADE_TAG         0x88
/*! Mask for the cascade bit of an ISO14443-A SAK. */
#define EMV_SAK_CASCADE_BIT_MASK                0x04
/*! Mask for the ISO14443-4 compliance bit fo an ISO14443-A SAK. */
#define EMV_SAK_ISO144434_COMPLIANT_BIT_MASK    0x20

//#define DEBUG_PBOC                     
typedef unsigned char                    Bool;




EmvPicc_t picc;

int iAS3911_Fd = -1;       /*AS3911设备ID*/
                                                                                       	                                         
/**************************************************************************
*                            模块内部变量                        *
**************************************************************************/
 /* Buffer for data transfer (send/receive) */
 unsigned char buffer[256];
 unsigned char rcvBuf[256];

 /**************************************************************************
 *                           局部函数原型                                  *
 **************************************************************************/


/**************************************************************************
 *                          全局函数实现                                      *
 **************************************************************************/
extern void as3911Isr( int t );


static int as3911startsignal( void )
{
	/*int oflags = 0;

	signal( SIGIO, as3911Isr );
	fcntl(iAS3911_Fd,F_SETOWN,getpid());
	oflags = fcntl( iAS3911_Fd, F_GETFL);
	fcntl(iAS3911_Fd,F_SETFL,oflags |FASYNC );*/
	return 0;
}

 
 

static int AS3911_Spi_close(void)
{
	if  ( iAS3911_Fd  >  0 )
	{
		close( iAS3911_Fd );
		iAS3911_Fd = -1;
		return 0;
	}
	else
	{
		return -1;
	}
}


static int AS3911_Spi_open(void)
{
	if (( iAS3911_Fd  =  open( "/dev/spi_rfid", O_RDWR ) )  <  0 ) 
	{
		myTACE("open %s failed \n", "/dev/spi_rfid");
		return -1;	
	}
	
	return 0;
	
}

int AS3911_open(void)
{
	return AS3911_Spi_open();
}

int AS3911_close(void)
{
	return AS3911_Spi_close();
}


int AS3911_init(void)
{		
	/* Reset the AS3911 */
	as3911ExecuteCommand(AS3911_CMD_SET_DEFAULT);//c1
	
	/* MCU_CLK and LF MCU_CLK off, 27MHz XTAL */
	as3911WriteRegister(AS3911_REG_IO_CONF1, 0x0F);
	
	/* Enable Oscillator, Transmitter and receiver. */
	as3911WriteRegister(AS3911_REG_OP_CONTROL, 0xC8);
	
	sleepMilliseconds(5);//5
	
	 /* Reset field and continue with polling. */
	emvHalResetField();
	 
	if (!emvHalFieldIsActivated())
	{
	    emvDisplayString("EMV: activating carrier\n");
	    emvHalActivateField(TRUE);
	}
	//if(Get_Test_Wupa_Quck())
	//{
		appTestCmd2();
	//}
	
	return 0;	
}

#define MAX_GAIN_TABLE_SIZE 9
static u8 mainGainTableX[MAX_GAIN_TABLE_SIZE];
static u8 mainGainTableY[MAX_GAIN_TABLE_SIZE];
static AS3911GainTable_t mainGainTable = {
	0,
	&mainGainTableX[0],
	&mainGainTableY[0]
	};
#define MAX_MODULATION_TABLE_SIZE 9
static u8 mainModulationTableX[MAX_MODULATION_TABLE_SIZE];
static u8 mainModulationTableY[MAX_MODULATION_TABLE_SIZE];
static AS3911ModulationLevelTable_t mainModulationTable = {
	0,
	&mainModulationTableX[0],
	&mainModulationTableY[0]
	};


u8 data_quck[]={ 0xef,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x00,0x00,0x02,0x05,0x66,0x74,0x9a,0x80,0xbd,0x88,0xcc,0x8e,0xd3,0x8e	};
   AS3911ModulationLevelAutomaticAdjustmentData_t  mainModulationAutomaticAdjustmentData;
 //  AS3911ModulationLevelTable_t mainModulationTable ;
#define LOG myTACE
#define myTACE printf
   void displayRegisterValue(unsigned char  address)
{
    u8 value = 0;
    as3911ReadRegister(address, &value);
  LOG("REG: 0x%x: 0x%x\r\n", address, value);
   // printf("\033[40;44mREG: 0x%x: 0x%x\r\n\033[5m", address, value);
}
   
void displayTestRegisterValue(unsigned char address)
{
    u8 value = 0;
    LOG("Test REG: 0x%x: 0x%x\r\n", address, value);
}
void show3911Reg()
{
        LOG("EMV: show3911Reg()\r\n");
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
		
}
 void  appTestCmd()
{
	u8  * rxData; u8 rxSize;
	rxData=data_quck;
	rxSize=sizeof(data_quck);
         s8 retVal = 0;
	const u8 *rxByte;
	u8 modulationDepthMode = 0;
	u8 gainMode = 0;
	int index = 0;
	show3911Reg();
        /* EMV Mode initialization command. */
	LOG("EMV: analog settings: ");
	for (index = 0; index < rxSize; index++)
		LOG("%x ", rxData[index]);
	LOG("\r\n");

	/* Voltage Regulator setup. */
        as3911ModifyRegister(AS3911_REG_IO_CONF2, 0x80, (rxData[1] & 0x10) << 3);
        if (0x00 == (rxData[1] & 0x0F))
        {
            /* Disable the voltage regulator. */
	   LOG("EMV:  Disable the voltage regulator 2: \r\n");
            as3911ModifyRegister(AS3911_REG_IO_CONF2, 0x40, 0x40);
        }
        else
        {
            /* Enable the voltage regulator. */
	   LOG("EMV:  Enable the voltage regulator2: \r\n");		
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
	LOG("EMV: gain reduction table length %d\r\n", mainGainTable.length);
	for (index = 0; index < mainGainTable.length; index++)
		LOG("EMV: gainTable[%d] = 0x%x, 0x%x\r\n", index, mainGainTable.x[index], mainGainTable.y[index]);

	if (0x00 == gainMode)
	{
		as3911SetGainMode(AS3911_GAIN_FIXED, NULL);
		LOG("EMV: using fixed gain reduction\r\n");
	}
	/*else if (0x01 == gainMode)
	{
		int index;
		
		as3911SetGainMode(AS3911_GAIN_FROM_AMPLITUDE, &mainGainTable);
		LOG("EMV: using table based gain reduction\n");
	}
	else
		LOG("EMV: Error: unkown adaptive gain mode byte: 0x%x\n", gainMode);
	*/
	
	//Read ISO14443B modulation depth mode byte.
	modulationDepthMode = *rxByte++;
	LOG("EMV: modulationDepthMode: 0x%x\r\n", modulationDepthMode);
	if (0x00 == modulationDepthMode)
	{
		
		u8 modulationDepth = *rxByte++;
		as3911WriteRegister(AS3911_REG_AM_MOD_DEPTH_CONF, 0x80);
		as3911WriteRegister(AS3911_REG_RFO_AM_ON_LEVEL, modulationDepth);
		LOG("EMV: using fixed am driver strength %x\r\n", modulationDepth);
		emvHalSetAs3911TypeBModulationMode(AS3911_MODULATION_LEVEL_FIXED, NULL);
	}
	else if (0x01 == modulationDepthMode)
	{
		u8 adjustmentTargetValue = *rxByte++;
		u8 adjustmentDelay = *rxByte++;
		
		as3911WriteRegister(AS3911_REG_AM_MOD_DEPTH_CONF, 0x7F & adjustmentTargetValue);
		LOG("EMV: using automatic modulation depth adjustment\r\n");
		LOG("EMV: adjustment target value: %X\r\n", adjustmentTargetValue);
		LOG("EMV: post adjustment delay: %x\r\n", adjustmentDelay);
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
		
		LOG("EMV: using table based modulation depth adjustment\r\n");
		LOG("EMV: modulation depth adjustment table length %d\r\n", mainModulationTable.length);
		for (index = 0; index < mainModulationTable.length; index++)
			LOG("EMV: modulationTable[%d] = 0x%x, 0x%x\r\n", index, mainModulationTable.x[index], mainModulationTable.y[index]);
		
		// FIXME: configuration of the mod depth conf register should be done inside the
		 //modulation level adjustment module.
		//
		as3911WriteRegister(AS3911_REG_AM_MOD_DEPTH_CONF, 0x80);
		emvHalSetAs3911TypeBModulationMode(AS3911_MODULATION_LEVEL_FROM_AMPLITUDE, &mainModulationTable);
	}
	else
	{
		LOG("Error: unkown ISO14443B modulation depth mode byte: 0x%x\r\n", modulationDepthMode);
	}
		
	show3911Reg();
	return;
}

 
//u8 data_quck2[]={ 0x06,0x66,0x74,0x9a,0x80,0xbd,0x88,0xcc,0x8e,0xd3,0x8e,0x5c, 0x7c	};
//4号板.
//u8 data_quck2[]={ 0x06,0x43,0xdc,0xb3,0xd8,0xc1,0xe0,0xc4,0xe2,0xc9,0xe3,0xca, 0xe3	};
//5号板.
//u8 data_quck2[]={ 0x06,0xe,0xd4,0x2f,0xe4,0x74,0xe4,0xa1,0xe5,0xae,0xec,0xb1, 0xf0	};
//1号板.
//u8 data_quck2[]={ 0x06,0x6d,0xd8,0xdb,0xd6,0xe6,0xde,0xe8,0xe2,0xeb,0xe5,0xed, 0xe4	};
//3号板.
u8 data_quck2[]={ 0x06,0x3f,0xe4,0x6f,0xe4,0x85,0xe5,0x8f,0xe6,0x96,0xe6,0x98, 0xe6	};

 void  appTestCmd2()
{
	u8  * rxData; u8 rxSize;
	rxData=data_quck2;
	rxSize=sizeof(data_quck);
         s8 retVal = 0;
	const u8 *rxByte;
	u8 modulationDepthMode = 0;
	u8 gainMode = 0;
	int index = 0;
	//show3911Reg();
        /* EMV Mode initialization command. */
	LOG("EMV: analog settings: \r\n");
	rxByte = &rxData[0];

	LOG("EMV: modulationDepthMode: 0x%x\r\n", modulationDepthMode);

	mainModulationTable.length = *rxByte++;
	for (index = 0; index < mainModulationTable.length; index++)
	{
		mainModulationTable.x[index] = *rxByte++;
		mainModulationTable.y[index] = *rxByte++;
	}
	
	LOG("EMV: using table based modulation depth adjustment\r\n");
	LOG("EMV: modulation depth adjustment table length %d\r\n", mainModulationTable.length);
	for (index = 0; index < mainModulationTable.length; index++)
		printf("EMV: modulationTable[%d] = 0x%x, 0x%x\r\n", index, mainModulationTable.x[index], mainModulationTable.y[index]);
	
	// FIXME: configuration of the mod depth conf register should be done inside the
	 //modulation level adjustment module.
	//
	as3911WriteRegister(AS3911_REG_AM_MOD_DEPTH_CONF, 0x80);
	emvHalSetAs3911TypeBModulationMode(AS3911_MODULATION_LEVEL_FROM_AMPLITUDE, &mainModulationTable);
	displayRegisterValue(AS3911_REG_RFO_AM_ON_LEVEL);
	displayRegisterValue(AS3911_REG_RFO_AM_OFF_LEVEL);
	//show3911Reg();
	return;
}

