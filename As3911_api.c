

/*AS3911Ó¦ÓÃµ÷ÓÃ½Ó¿Ú*/

#include "As3911_api.h"
#include "as3911_gain_adjustment.h"
#include "as3911_def.h"
#include "ams_types.h"
#include "emv_error_codes.h"
#include "emv_hal.h"
#include "emv_response_buffer.h"
#include "emv_typeA.h"
#include "emv_typeB.h"
#include "emv_standard.h"
#include "emv_main.h"
#include "sleep.h"
#include "emv_picc.h"
#include "as3911_io.h"
#include "main.h"

#define api_debug(...) 
/*#define FALSE                               0
#define TRUE                                 1
#define MIFARE_DEFAULT_READER_NONCE             0xAA55AA55

//#define EMV_TYPEA_FDT_9     1250

#define EMV_ATQA_UID_SIZE_MASK      0xC0
#define EMV_ATQA_UID_SIZE_SINGLE    0x00
#define EMV_ATQA_UID_SIZE_DOUBLE    0x40
#define EMV_ATQA_UID_SIZE_TRIPLE    0x80


 *****************************************************************************
 * Invalid ATQA UID size field value. A standard conforming card
 * should never return this value.
 *****************************************************************************

#define EMV_ATQA_UID_SIZE_INVALID   0xC0

#define EMV_SEL_CL1     0x93
#define EMV_SEL_CL2     0x95
#define EMV_SEL_CL3     0x97

#define EMV_ANTICOLLISION_NVB   0x20
#define EMV_SELECT_NVB          0x70

#define EMV_CASCADE_TAG         0x88
#define EMV_SAK_CASCADE_BIT_MASK                0x04
#define EMV_SAK_ISO144434_COMPLIANT_BIT_MASK    0x20

typedef unsigned char                    Bool;
*/


/*
#define MAX_GAIN_TABLE_SIZE 9
static u8 mainGainTableX[MAX_GAIN_TABLE_SIZE];
static u8 mainGainTableY[MAX_GAIN_TABLE_SIZE];
static AS3911GainTable_t mainGainTable = {
	0,
	&mainGainTableX[0],
	&mainGainTableY[0]
	};*/
#define MAX_MODULATION_TABLE_SIZE 9
static u8 mainModulationTableX[MAX_MODULATION_TABLE_SIZE];
static u8 mainModulationTableY[MAX_MODULATION_TABLE_SIZE];
static AS3911ModulationLevelTable_t mainModulationTable = {
	0,
	&mainModulationTableX[0],
	&mainModulationTableY[0]
	};


 //  AS3911ModulationLevelTable_t mainModulationTable ;

void displayRegisterValue(unsigned char  address)
{
    u8 value = 0;
    as3911ReadRegister(address, &value);
    api_debug("REG: 0x%x: 0x%x\r\n", address, value);
   // printk("\033[40;44mREG: 0x%x: 0x%x\r\n\033[5m", address, value);
}
   
void displayTestRegisterValue(unsigned char address)
{
    u8 value = 0;
    api_debug("Test REG: 0x%x: 0x%x\r\n", address, value);
}
void show3911Reg(void)
{
	displayRegisterValue(0);	
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
       displayRegisterValue(0x27);	
        //displayTestRegisterValue(AS3911_REG_ANALOG_TEST);	
		
}
 
	
 //·Ç0x9e°æ±¾
//4ºÅ°å.
//u8 data_quck2[]={ 0x06,0x43,0xdc,0xb3,0xd8,0xc1,0xe0,0xc4,0xe2,0xc9,0xe3,0xca, 0xe3	};
//5ºÅ°å.
//u8 data_quck2[]={ 0x06,0xe,0xd4,0x2f,0xe4,0x74,0xe4,0xa1,0xe5,0xae,0xec,0xb1, 0xf0	};
//1ºÅ°å.
//u8 data_quck2[]={ 0x06,0x6d,0xd8,0xdb,0xd6,0xe6,0xde,0xe8,0xe2,0xeb,0xe5,0xed, 0xe4	};
//3ºÅ°å.
//u8 data_quck2[]={ 0x06,0x3f,0xe4,0x6f,0xe4,0x85,0xe5,0x8f,0xe6,0x96,0xe6,0x98, 0xe6	};

//0x9e°æ±¾
//4ºÅ°å.
//u8 data_quck2[]={ 0x06,0x43,0xdc,0xb3,0xd8,0xc1,0xe0,0xc4,0xe2,0xc9,0xe3,0xca, 0xe3	};
//5ºÅ°å.
 u8 data_quck2[]={ 0x06,0xee,0xbf,0xb6,0xc8,0x96,0xce,0x88,0xd1,0x7e,0xd2,0x7a, 0xd2 };

//1ºÅ°å.
//u8 data_quck2[]={ 0x06,0x72,0xdc,0xdb,0xd8,0xe5,0xde,0xe8,0xe3,0xea,0xe4,0xeb, 0xe4	};
//3ºÅ°å.
//u8 data_quck2[]={ 0x06,0x3c,0xe8,0x6e,0xe6,0x81,0xe7,0x8d,0xe6,0x93,0xe8,0x98, 0xe8	};

 void  appTestCmd2(void)
{
	u8 rxSize;
	u8 *rxByte;
	
	int index = 0;
	rxSize=sizeof(data_quck2);
	rxByte = &data_quck2[0];

	mainModulationTable.length = *rxByte++;
	for (index = 0; index < mainModulationTable.length; index++)
	{
		mainModulationTable.x[index] = *rxByte++;
		mainModulationTable.y[index] = *rxByte++;
	}
	
	api_debug("EMV: modulation depth adjustment table length %d\r\n", mainModulationTable.length);
	for (index = 0; index < mainModulationTable.length; index++)
		api_debug("EMV: modulationTable[%d] = 0x%x, 0x%x\r\n", index, mainModulationTable.x[index], mainModulationTable.y[index]);
	
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




