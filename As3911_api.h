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

/**************************************************************************
*                                       ����ԭ��                  *
**************************************************************************/
/*NFC����ԭ��*/
unsigned char NFC_Detect(unsigned char TypeMask,PTAS3911_ANTENNAPARM  ptAntennaParm);
void NFC_Reset(void);
void NFC_RemovalProcess(unsigned char bType);

/*
    return value :
    0  ok
    -1 failed
  */ 
  
int AS3911_open(void);

/*
    return value :
    0  ok
    -1 failed
  */ 
int AS3911_close(void);


/*
    return value :
    0  ok
    -1 failed
  */ 
int AS3911_init(void);


/*
	mode [IN]:
		'm' or 'M'		M1
		'a' or 'A'  		A�Ϳ�
		'b' or 'B'  		B�Ϳ�
	
	serial_no [OUT]:
		the serial number of the RF card.
		serial_no[0] is length of the serial number
		serial_no[1]..[N] is the serial number

    return value :
    0  ok
    -1 failed
*/
//int AS3911_check(unsigned char mode, unsigned char *serial_no);
int AS3911_check(unsigned char mode, unsigned char *serial_no, PTAS3911_ANTENNAPARM  ptAntennaParm );/*liuwei2012/11/17*/
 

/*
	type		[IN]	: password type  'a' 'A' or 'b' 'B'
	blk_No	[IN]	: the number of M1 block
	pwd		[IN]	: password
	serial_No [IN]	: the serial number of the M1
*/
int AS3911_M1_authority(unsigned char type, unsigned char blk_No, unsigned char *pwd, unsigned char *serial_No);


/*
    return value :
    0  ok
    -1 failed
  */ 
int AS3911_M1_read(unsigned char blk_No, unsigned char *blk_vlalue);


/*
    return value :
    0  ok
    -1 failed
  */ 
int AS3911_M1_write(unsigned char blk_No, unsigned char *blk_vlalue);


/*
	init the block to the struct for money bag
	value [IN] : the initial value for money

	return value :
	0  ok
	-1 failed	
*/

/*
	blk_No : the block number of source 

	return value :
	0  ok
	-1 failed		
*/
int AS3911_M1_Restore(unsigned char blk_No);


/*
	blk_No : the block number of destination
	
	return value :
	0  ok
	-1 failed		
*/
int AS3911_M1_Transfer(unsigned char blk_No);


int AS3911_M1_init_block(unsigned char blk_no ,int value);


/*
	type : operate type '+' or '-'
	value: the value which will added to or sub from the old value 

	return value :
	0  ok
	-1 failed		
*/
int AS3911_M1_operate(unsigned char type, unsigned char blk_No, int value);


/*
    return value :
    0  ok
    -1 failed
  */ 
int AS3911_iso14443a_reset(void);

int AS3911_iso14443a_reset_Rats( unsigned char * PucRats );

int AS3911_iso14443b_reset(void);

/*
	�������ƣ�	icc_command
	����������	CPU�������������,����ISO7816/PBOC��׼CPU�������
	��ڲ�����	
	cla  ---   	��������
	ins  ---   	����ֵ
	p1   ---   	����1
	p2   ---   	����2
	in_len(p3)   ---   	�������ݳ���
	inbuf	   --   	��������
	out_len(le)   ---   	���������������
	outdata ----   	���������

	inbuf�ĸ�ʽ --- 	p3���ȵ�����
	outbuf�ĸ�ʽ---	1�ֽڳ���(ʵ�ʿ����ص����ݳ���)+����            
	���ڲ����� 0, �ɹ�
					-1,ʧ��

	ע�ͣ����������ݲ�����ʱ����inlen = 0;
			��������ݲ�����ʱ����outlen = 0;
*/
int AS3911_iso14443a_APDU(unsigned char cla, unsigned char ins, unsigned char p1, unsigned char p2, 
			unsigned short  in_len,const unsigned char * inbuf, unsigned short  out_len, unsigned char *outbuf, 
			unsigned char *sw1, unsigned char *sw2);

/*
	halt a selected card.

	return value :
	0  ok
	-1 failed	
*/
int AS3911_halt( unsigned char mode );

int AS3911_rw_AntennaParm( PTAS3911_ANTENNAPARM ptAntennaParam );
int AS3911_read_libCBFLVersion(  char  *pchVersion );
void show3911Reg();
void  appTestCmd ();
void displayTestRegisterValue(unsigned char  address);
   void displayRegisterValue(unsigned char  address) ;
#define myTACE  printf
#endif//
