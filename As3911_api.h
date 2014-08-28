#ifndef __AS3911_API_H__
#define __AS3911_API_H__

/*NFC��*/
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
*                            ģ���ڲ���������                          *
**************************************************************************/

/*��Ƶ�����������ݽṹ*/
typedef struct 
{    	
	unsigned char ucOpType;           /*��Ƭ��������*/
	unsigned char ucRwFlag;            /*0:��  1: д*/
	unsigned char ucInRxGain;         /*  �����������ò���*/
	unsigned char ucInGsnReg; 	 /*  ����˵��Ʋ���GsNOnReg:bit7-4:CWGsnOn  bit3-0:ModGsnOn*/
	unsigned char ucInCWGsPReg; 	 /*  CWGsPReg :bit5-0:CWGsP*/
	unsigned char ucInModGsPReg;  /*  ModGsPReg :bit5-0:ModGsP*/
	unsigned char ucInRxTresholdReg; /*  bit2-0:CollLevel :bit7-4:MinLevel*/
	unsigned char aucOutData[ 5 ];  /*[0]:��ȡ��ucInRxGain [1]:��ȡ��ucInGsnReg*/
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
#endif//
