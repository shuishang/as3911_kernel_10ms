#ifndef __AS3911_API_H__
#define __AS3911_API_H__

/*NFC宏*/
#define PH_LOOPBACK_TYPE_A 	0x01
#define PH_LOOPBACK_TYPE_B 	0x02

#define CARD_A                             0x01
#define CARD_B                             0x02
#define P2P                                   0x04
#define P2P_A                               0x08
#define P2P106                             0x10

#define HALTONDETECT		      0
#define NOHALTONDETECT 		1

#define P2P_DSL                            0
#define P2P_RLS                            1

/**************************************************************************
*                            模块内部数据类型                          *
**************************************************************************/

/*射频参数设置数据结构*/
typedef struct 
{    	
	unsigned char ucOpType;           /*卡片操作类型*/
	unsigned char ucRwFlag;            /*0:读  1: 写*/
	unsigned char ucInRxGain;         /*  接收增益配置参数*/
	unsigned char ucInGsnReg; 	 /*  发射端调制参数GsNOnReg:bit7-4:CWGsnOn  bit3-0:ModGsnOn*/
	unsigned char ucInCWGsPReg; 	 /*  CWGsPReg :bit5-0:CWGsP*/
	unsigned char ucInModGsPReg;  /*  ModGsPReg :bit5-0:ModGsP*/
	unsigned char ucInRxTresholdReg; /*  bit2-0:CollLevel :bit7-4:MinLevel*/
	unsigned char aucOutData[ 5 ];  /*[0]:读取的ucInRxGain [1]:读取的ucInGsnReg*/
}TAS3911_ANTENNAPARM, *PTAS3911_ANTENNAPARM; 


typedef enum
{
	CardType_Unknown,
	CardType_ContactlessA,
	CardType_ContactlessB,
} mlsCardType_t;


int AS3911_init(void);


void displayTestRegisterValue(unsigned char  address);
   void displayRegisterValue(unsigned char  address) ;
//#define myTACE  printk
#endif//
