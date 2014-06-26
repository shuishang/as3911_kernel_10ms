/* /////////////////////////////////////////////////////////////////////////////////////////////////
//                     Copyright (c) Philips Semiconductors
//
//				         test for RF card
///////////////////////////////////////////////////////////////////////////////////////////////// */


#include <stdio.h>
#include <fcntl.h> 
#include <stdio.h>
#include <errno.h> 
#include <fcntl.h>
#include <sys/types.h>
#include <termio.h>
#include <unistd.h> 
#include <string.h>
#include <math.h>
#include <pthread.h>

#include <sys/ioctl.h>
#include <linux/errno.h>
#include <sys/stat.h>     
#include <termios.h> 


#include <stdarg.h>
#include <time.h>
#include <sys/timeb.h>
#include <pthread.h>

#include "As3911_api.h"
#include "emv_main.h"
#include "emv_picc.h"
#include "As3911_def.h"
/*rfid ��ͷ�ļ�*/



/**************************************************************************
*                            ģ���ڲ��궨��                               *
**************************************************************************/
#define   READONLY                      ( (unsigned char) 0 )
#define   WRITEONLY                    ( (unsigned char) 1 )
#define   READWRITE                    ( (unsigned char) 2 ) 

#define   SUCCESS                           (  0  )      /* ���سɹ�*/
#define	EC_FAILED			( -1 )      /* ���д������ͷ���*/
#define	EC_CANCELLED		( -2 )      /*  */
#define   EC_CMD_FAILED              ( -5 )     /* �������*/

#define   CARDUTILS_PN5XX_MAX_BUFFSIZE			272

#define  DEBUG                                0
#define  false                                   0


#define  START                                  0
#define  POLLING                              1
#define  CARRIERSWITCH                   2
#define  WUPA                                  3
#define  WUPARATS                           4
#define  WUPB                                  5
#define  WUPBATTRIB                        6
#define  RESET                                  7
#define  EMVDIGITAL                       8
#define  STOP_DIGITAL                    9
#define APPL_COM_REG_DUMP           0x31/*! 'A'-0x30< Dump register content. */
#define APPL_COM_REG_WRITE          0x32 /*!'B'-0x30< Write register. */
#define APPL_COM_REG_READ           0x33 /*!'C'-0x30 < Read register. */
#define APPL_NO_CARD_WUPA           0x34  //'D'-0x30
#define APPL_SET_GAIN_ADD      0x35  //'E'-0x30
#define APPL_SET_GAIN_SIN     0x36 //'F'-0x30
#define APPL_SET_TEST_MODUL 0x37 //'g'-0x30
//quck -2014��4��22��9:27:56
#define G_RESPONSE_BUFFER_LEN_MAX 100
unsigned char g_Response_Buffer[G_RESPONSE_BUFFER_LEN_MAX];
unsigned short g_Index=0; //��Զֻ��g_Response_Bufferһ�������õ�����.

#define CMD_FRAME_LEN 19
/**************************************************************************
*                            ģ���ڲ���������                          *
**************************************************************************/
typedef struct 
{ 
	unsigned char ucCLA; 
	unsigned char ucINS; 
	unsigned char ucP1; 
	unsigned char ucP2; 
	unsigned char ucLc; 
	unsigned char aucDataIn[300]; 
	unsigned char ucLeFlag; 
	unsigned char ucLe; 
}TAPDU_SEND; 

typedef struct 
{ 
	unsigned int   wLenOut; 
	unsigned char aucDataOut[300]; 
	unsigned char ucSWA; 
	unsigned char ucSWB; 
}TAPDU_RESP;

typedef unsigned char                    Bool;

/**************************************************************************
*                           ȫ�ֱ���                        *
**************************************************************************/

/**************************************************************************
*                            ģ���ڲ�����                        *
**************************************************************************/
char      g_aucParmBuf[ 100 ]        = {  0  };    /*�����в���������*/
TAS3911_ANTENNAPARM   tAntennaParam;
PTAS3911_ANTENNAPARM  ptAntennaParam = &tAntennaParam;
EmvPicc_t EmvPicc;

unsigned char serial_no[ 64 ] = { 0 };
unsigned char g_serial_flag= 0;

/*������������*/
int speed_arr[] = {B115200, B57600,B38400, B19200, B9600, B4800, B2400, B1200, B300 };

int name_arr[] = {115200,  57600, 38400,  19200,  9600,  4800,  2400,  1200,  300 };

int md_fd                             = 0;



/**************************************************************************
*                            �ֲ���������                                      *
**************************************************************************/
//����
//���� src Ϊ����, len Ϊ���峤��
void mem_recover(char *src  ,unsigned char len)
{
	unsigned char t, i;
	char *dest =src+len-1 ;
	for(i=0;i<len/2;i++)
	{
		t=*dest ;
		*dest=*src;
		*src=t;
		
		--dest;
		++src;
	}

}
/*
 ���� value  ������ʮ������Ϊ 1004
 ��� ��str��  ����Ϊ  str[0] '1', str[1] '0', str[2] '0', str[3] '4'��str[i]=0;	
 buf_len Ϊ����str�ĳ���
 */
void int2string_dec(int value, char *str, char buf_len)
{
   unsigned char  temp,i;
   const char Hex_Code[]={"0123456789"};
   if(value == 0)
   {
	str[0]='0';
	str[1]=0;
	return ;
   }
   for(i=0;i<buf_len;i++)
   {
   	if(value)
   	{
		temp=value%10;
		value/=10;	
		str[i]=Hex_Code[temp];
	}
	else
	{
		str[i]=0;	
	}
	
   }
   str[buf_len-1]=0;

   mem_recover(str,strlen(str));
   return ;
}

void int2string_hex(int num, char *buffer, char buf_len)
{
		const char Hex_Code[]={"0123456789ABCDEF"};
		//printf("  quck_to_prin:%lx ",num);
		buffer[3]=(num&0xff000000)>>24;
		if(buffer[3])
		{
			write( md_fd, &Hex_Code[(buffer[3]>>4)], 1 );	
			write( md_fd, &Hex_Code[(buffer[3]&0x0f)], 1 );	
		}
		buffer[2]=(num&0x00ff0000)>16;
		if(buffer[2])
		{
			write( md_fd, &Hex_Code[(buffer[2]>>4)], 1 );	
			write( md_fd, &Hex_Code[(buffer[2]&0x0f)], 1 );	
		}
		
		
		buffer[1]=(num>>8)&0xff;
		if(buffer[1])
		{
			write( md_fd, &Hex_Code[(buffer[1]>>4)], 1 );	
			write( md_fd, &Hex_Code[(buffer[1]&0x0f)], 1 );	
		}		
		buffer[0]=num&0xff;
		write( md_fd, &Hex_Code[(buffer[0]>>4)], 1 );	
		write( md_fd, &Hex_Code[(buffer[0]&0x0f)], 1 );	
	
}
/*
 num , data to print
 sign ,hex ,or Dex .  1Ϊ 
�ǽ��ƣ� 0Ϊ16���ơ�
 base , no use.
*/
void printNum(long int num,int base,int sign)
{
	char buffer[10];
	int n;
	if(sign)
	{
		//printf("  quck_to_prin:%d ",num);
		int2string_dec(num,buffer,sizeof(buffer));
		write( md_fd, buffer, strlen(buffer) );	
	}
	else  //0xfe12 buffer[3]=0x0 ,buffer[2]=0x0 ,buffer[1]=0xfe ,buffer[0]=0x12 ,
	{
		int2string_hex(num,buffer,sizeof(buffer));
	}


}
/*
    ��ʽ�������֧�� %d �� %x 2�ָ�ʽ��
    �����������λ���ܳ���9λ��,֧��999999999���������ֽ����ԣ�
 	16����������󲻳���4�ֽڡ��������ֽ����ԣ�
 	��֧�� %02x  ��ʽ��
 	��֧�� %02d��ʽ
 	%D ��%X ֧�֡�

*/

void myTACE( char * fmt, ...)
{
	unsigned char c;
	unsigned char base,sign;
	va_list ap;
	va_start(ap, fmt);
	
	
	for(;*fmt!='\0';fmt++)
	{
		base=0;sign=0;
		if(*fmt=='%')
		{
			fmt++;
			c=*fmt;
			switch(c)
			{
				case 'd':case 'D': base=10;sign=1;break;
				case 'x':case 'X': base=16;sign=0;break;
				default:
				write( md_fd, &c, 1 );	
			}
			if(base)
			{
				printNum(( long int)va_arg(ap,long int),base,sign);
				
			}			
		}
		else
		{
			write( md_fd, fmt, 1 );	
		}

	}
 	va_end(ap);
}


/**************************************************************************
*                            �ֲ���������                                      *
**************************************************************************/


/**************************************************************************
* �������ƣ�void set_speed(int fd, int speed)
* �������������ô������� 
* ��������� 
* ��������� 
* �� �� ֵ�� 
* ����˵����
* �޸�����    �汾��     �޸���	     �޸�����
* -----------------------------------------------
* 2013/03/06	         V1.0	          liuwei	               XXXX
**************************************************************************/
void set_speed(int fd, int speed)
{
	unsigned int   i           =  0; 
	int   status   =  0; 
	struct termios   Opt;
	
	tcgetattr( fd, &Opt ); 
	
	for ( i= 0;  i < sizeof( speed_arr ) / sizeof( int );  i++ ) 
	{ 
		if  ( speed == name_arr[ i ] ) 
		{     
			tcflush( fd, TCIOFLUSH );     
			cfsetispeed( &Opt, speed_arr[i] );  
			cfsetospeed( &Opt, speed_arr[i] );   
			status = tcsetattr( fd, TCSANOW, &Opt );  
			
			if  (status != 0) 
			{        
				printf("tcsetattr failed\n");
				return;     
			}    
			
			tcflush(fd,TCIOFLUSH);   
		}  
	}
}

/**************************************************************************
* �������ƣ�int set_Parity(int fd,int databits,int stopbits,int parity)
* ������������������λ,��żУ��λ,ֹͣλ 
* ��������� 
* ��������� 
* �� �� ֵ�� 
* ����˵����
* �޸�����    �汾��     �޸���	     �޸�����
* -----------------------------------------------
* 2013/03/06	         V1.0	          liuwei	               XXXX
**************************************************************************/
int set_Parity( int fd, int databits, int stopbits, int parity )
{
	struct termios options; 
	if  ( tcgetattr( fd, &options)  !=  0) 
	{ 
		printf("tcgetattr failed\n");     
		return -1;  
	}

	options.c_cflag &= ~CSIZE; 
	switch ( databits ) 
	{
		case 7:		
			options.c_cflag |= CS7; 
			break;
			
		case 8:     
			options.c_cflag |= CS8;
			break;   
			
		default:    
			printf("Unsupported data size\n"); 
			return -1;  
	}

	switch (parity) 
	{   
		case 'n':
		case 'N':    
			options.c_cflag &= ~PARENB;   /* Clear parity enable */
			options.c_iflag &= ~INPCK;     /* Enable parity checking */ 
			break;  
			
		case 'o':   
		case 'O':     
			options.c_cflag |= (PARODD | PARENB); 
			options.c_iflag |= INPCK;             /* Disnable parity checking */ 
			break;  
			
		case 'e':  
		case 'E':   
			options.c_cflag |= PARENB;     /* Enable parity */    
			options.c_cflag &= ~PARODD;       
			options.c_iflag |= INPCK;       /* Disnable parity checking */
			break;
			
		case 'S': 
		case 's':  /*as no parity*/   
	    		options.c_cflag &= ~PARENB;
			options.c_cflag &= ~CSTOPB;
			break;  
			
		default:   
			printf("Unsupported parity\n");
			return -1;  
			
	}  

	switch (stopbits)
	{   
		case 1:    
			options.c_cflag &= ~CSTOPB;  
			break;  
			
		case 2:    
			options.c_cflag |= CSTOPB;  
	  		break;
		
		default:    
		 	fprintf(stderr,"Unsupported stop bits\n");  
		 	return -1; 
	} 
	
	/* Set input parity option */ 
	if (parity != 'n')   
		options.c_iflag |= INPCK; 

	tcflush(fd,TCIFLUSH);
	options.c_cc[VTIME] = 5;
	options.c_cc[VMIN] = 0; 

	if (tcsetattr(fd,TCSANOW,&options) != 0)   
	{ 
		printf("tcsetattr failed\n");   
		return -1;  
	} 
	
	return 0;  

}
/**************************************************************************
* �������ƣ�int set_raw_mode(int fd)
* ���������� 
* ��������� 
* ��������� 
* �� �� ֵ�� 
* ����˵����
* �޸�����    �汾��     �޸���	     �޸�����
* -----------------------------------------------
* 2013/03/06	         V1.0	          liuwei	               XXXX
**************************************************************************/
int set_raw_mode(int fd)
{
	struct termios options; 
	
	if  ( tcgetattr( fd,&options )  !=  0) 
	{ 
		printf("tcgetattr failed\n");     
		return -1;  
	}

	options.c_iflag &= ~(IGNBRK | IGNCR | INLCR | ICRNL | IUCLC | IXANY | IXON | IXOFF | INPCK | ISTRIP);
	options.c_iflag |= (BRKINT | IGNPAR);
	options.c_oflag &= ~OPOST;
	options.c_lflag &= ~(XCASE|ECHONL|NOFLSH);
	options.c_lflag &= ~(ICANON | ISIG | ECHO);
	options.c_cflag |= (CLOCAL | CREAD);

	//software flow control
	#if 0
	options.c_iflag &line;= (IXON &line; IXOFF &line; OXANY );
	#endif

	if ( tcsetattr( fd,TCSANOW,&options ) != 0 )   
	{ 
		printf("tcsetattr failed\n");   
		return -1;  
	} 
	
	return -1;  	
}

/**************************************************************************
* �������ƣ�inline void TRACE_TIME(char * fmt, ...)
* ���������� 
* ��������� 
* ��������� 
* �� �� ֵ�� 
* ����˵����
* �޸�����    �汾��     �޸���	     �޸�����
* -----------------------------------------------
* 2013/03/06	         V1.0	          liuwei	               XXXX
**************************************************************************/
inline void TRACE_TIME(char * fmt, ...)
{
	char buf[1024];
	va_list argptr;
	struct timeb     tp;
	struct tm	*tm;

	ftime(&tp);
	tm = localtime(&(tp.time));

	sprintf(buf,"[%02d:%02d:%02d:%03d] : ",tm->tm_hour, tm->tm_min, tm->tm_sec, tp.millitm );

	printf("%s",buf);

	va_start(argptr, fmt);
	
	vprintf(fmt, argptr);
	
	va_end(argptr);
}



void usage(char **argv)
{
	printf("Usage:");
	printf("\n\rf_id_test [-h] [-t N]");
	printf("\n");
	printf("\n\t-h : help information just like this");
	printf("\n\t-t : the type of  wireless cpu card [N,default=0]");
	printf("\n\t     0 : Mifare card");
	printf("\n\t     1 : 14443 Type A");
	printf("\n\t     2 : TongBao Card");
	printf("\n\t     3 : Rfid AntennaParm Config");
	printf("\n\n");
}


/**************************************************************************
* �������ƣ�void set_speed(int fd, int speed)
* �������������ô������� 
* ��������� 
* ��������� 
* �� �� ֵ�� 
* ����˵����
* �޸�����    �汾��     �޸���	     �޸�����
* -----------------------------------------------
* 2013/03/06	         V1.0	          liuwei	               XXXX
**************************************************************************/
int SendSerialData(int fd,   char * ucInString, int Num,  char ucFlag )
{
         char ucSendStr[ 100 ]              = { '\0' };
	int ApduLength                                           = strlen( "C-APDU -> : " );
         int i                                                                  = 0;

	switch ( ucFlag )
	{
		case 'c' :
		case 'C' :
			memcpy( ucSendStr,"C-APDU ->  ", ApduLength );
			break;
			
		case 'r' :
		case 'R' :
			memcpy( ucSendStr,"R-APDU <-  ", ApduLength );
			break;
			
		case WUPA :
			ApduLength = strlen( "WUPA -> \r\nATQA<- " );
			memcpy( ucSendStr, "WUPA -> \r\nATQA<- " , ApduLength );
			break;	

		case WUPARATS:
			ApduLength = strlen( "WUPARATS ->\r\nWUPARATS <- " );
			memcpy( ucSendStr, "WUPARATS ->\r\nWUPARATS <- ", ApduLength );
			break;	
			
		case WUPB :
			ApduLength = strlen( "WUPB ->  " );
			memcpy( ucSendStr,"WUPB ->  ", ApduLength );
			break;	

		case WUPBATTRIB :
			ApduLength = strlen( "ATTRIB <- " );
			memcpy( ucSendStr,"ATTRIB <- ", ApduLength );
			break;
		case 'D' :
		case 'd' :
			ApduLength = strlen( " " );
			memcpy( ucSendStr," ", ApduLength );
			break;					
		case 'p' :
		case 'P' :
			myTACE(ucInString);
			return 0;

	}

	myTACE(ucSendStr);
	for ( i = 0; i < Num; i++ )
	{

		myTACE("%x ",ucInString[ i ] );
	}
	myTACE("\r\n");

	return 0;
}
/*
ʧ��,����-1
�ɹ�, ����  SUCCESS
/*
���� 20���ֽ�,{0x00 ,' ',0xff ,' '  . ... .0xXX ,' '}
pucOutParm  ���10���ֽ�����.

*/
int TransCmdParm(  unsigned char  *pcInStr,  unsigned char  *pucOutParm )
{
	unsigned char   i ;
	int   iColumn ;
	if ( NULL == pcInStr )//|| 0 == ( iNum = strlen( pcInStr )) )
	{
		//printf( " The input parm is wrong" );
		return EC_FAILED;
	}
	iColumn=0;
	
	/*for(i = 0 ;i<CMD_FRAME_LEN;i++)
	{
		if(i%2 == 0)
		{
			pucOutParm[ iColumn] =   pcInStr[i] -0x30;
			SendSerialData( md_fd,pucOutParm[ iColumn],1, 'p' );
			iColumn++;
		}

	}*/
	for(i = 0 ;i<CMD_FRAME_LEN;i+=2)
	{

		pucOutParm[ iColumn] =   pcInStr[i] -0x30;
		
		iColumn++;

	}
	
	return SUCCESS;
}

/*
����1�� ����ʧ�ܡ�
����0 �����ǳɹ���
���������Ƿ��յ�һ֡���
�յ����������&g_Response_Buffer[g_Index-CMD_FRAME_LEN]λ��.
*/

int ReadCmdParm()
{
	int   iNum          = 0;
	int   iRow           = 0;
	int   iColumn      = 0;
	char response[ 200 ] ;
	unsigned short n,i;
	n = read( md_fd, response, 100 );
	//�ж��Ƿ�nΪ0
	if(0==n)
	{
		return 1;
	}
	
	SendSerialData( md_fd, response, n, 'p' );
	SendSerialData( md_fd, "+.\r\n",4, 'p' );
	//�ж��Ƿ������
	if((n+g_Index)>G_RESPONSE_BUFFER_LEN_MAX)
	{
		g_Index=0;
		memset(g_Response_Buffer,0,sizeof(g_Response_Buffer));
	}
	memcpy(&g_Response_Buffer[g_Index],&response[0],n);
	g_Index+=n;
	//�ж��Ƿ����19���ֽڡ�
	if((g_Index)<CMD_FRAME_LEN)
	{
		SendSerialData( md_fd, "+2\r\n",4, 'p' );
		return 1;
	}
	
	//�ж��Ƿ�ͷΪ'0'; '2';  �Ƿ��пո�
	for(i=0;i< g_Index;i++)
	{
		if(('0' ==response[i] )&& (' ' ==response[i+1]) && ('2' ==response[i+2]) && (' ' ==response[i+3]))
		{
			if((i+CMD_FRAME_LEN) <= g_Index )//&& response[i+CMD_FRAME_LEN-2] == ' ')
			{
				
				return 0; //�ɹ�
			}
			else
			{
				SendSerialData( md_fd, "+3\r\n",4, 'p' );
				memcpy(&g_Response_Buffer[0],&g_Response_Buffer[i],g_Index-i);
				g_Index=g_Index-i;
			}
		}
		else
		{
			g_Index=0;
			memset(g_Response_Buffer,0,sizeof(g_Response_Buffer));

		}
	}
	SendSerialData( md_fd, "+5\r\n",4, 'p' );
	return 1;
}

void clear_serial_buffer()
{
		memset( g_Response_Buffer, 0,sizeof( g_Response_Buffer ));
		memset( g_aucParmBuf, 0,sizeof( g_aucParmBuf ));	
		g_Index=0;
}
/*�߳�1ִ�к���
   //�����Ƿ��յ�һ������
   //����ת��
   //��־����1
   //�ӳټ�������.
*/
void *create(void *arg)
{
   g_serial_flag=0;
   while(1)
   {
	sleep(1);
	if(g_serial_flag)
	{
		continue;
	}
	if(ReadCmdParm())
	{
		continue;
	}
	memset(g_aucParmBuf,0,sizeof(g_aucParmBuf));
	TransCmdParm( &g_Response_Buffer[g_Index-CMD_FRAME_LEN], (unsigned char *)g_aucParmBuf );
	SendSerialData( md_fd,g_aucParmBuf,10, 'd' );   
	if(STOP_DIGITAL == g_aucParmBuf[ 3 ] )
	{
		emvStopTerminalApplication();
		clear_serial_buffer();
		continue;
	}
	g_serial_flag=1;
   }
		
    return (void *)0;
}
const char version[]={ "as3911_src_v2[ "__TIME__"]\r\n"};
int main(int argc, char **argv)
{
	int i =0;
	int n                                     =  0;
	int PacketLength                    = 0;
	int res                                   =  0;
	mlsCardType_t          CardType = CardType_ContactlessA;
	unsigned char  ucCarrierOffOn          = false;
	pthread_t tidp;
	 
	md_fd = open("/dev/ttySAC1" ,O_RDWR);
	if (md_fd < 0)
	{    
		printf("open Serial commuication port failed,errno:%d\n",md_fd);
		return -1;
	}

	set_raw_mode( md_fd );
	set_speed( md_fd, 115200 );

	if ( set_Parity( md_fd, 8, 1, 'N' ) == -1 )  
	{
		printf("Set Parity Error\n");
		return -1;
	}

	res =AS3911_open();
	if ( 0 != res )
	{
		printf( "RF open failed\n" );
		return -1;
	}
	
	AS3911_init();
	clear_serial_buffer();
	myTACE((char*)version);
	if(   pthread_create(&tidp, NULL, create, (void *)0))
	{
		printf("phread is not created...\n");
		return -1;
	}	
	 emvGuiDigital( );
	
	while( 1 )
	{
		while(1)
		{
			sleep(1);
			if(g_serial_flag)
			{
				g_serial_flag=0;
				break;
			}	
		}
		res    = 0;
		memset( serial_no, 0,sizeof( serial_no ));
/* #ifdef  DEBUG
		g_aucParmBuf[ 0 ] = 0x00;
		g_aucParmBuf[ 1 ] = 02; 
		g_aucParmBuf[ 2]  = 0x06;
		g_aucParmBuf[ 3 ] = POLLING;
#endif */
		PacketLength = g_aucParmBuf[ 2 ] ;
		memcpy( ptAntennaParam, &g_aucParmBuf[ 3 ], PacketLength );
		  switch(g_aucParmBuf[ 3 ] )
		  {
			case START :			
				SendSerialData( md_fd, (char*)version, strlen(version), 'p' );
				break;
				
			case POLLING :
				SendSerialData( md_fd, "begi1 \r\n", strlen("begin \r\n"), 'p' );
				CardType = emvGuiPoll();
				if ( CardType == 1) SendSerialData( md_fd, "CardType_ContactlessA", strlen("CardType_ContactlessA"), 'p' );
				else if ( CardType == 2 ) SendSerialData( md_fd, "CardType_ContactlessB", strlen("CardType_ContactlessB"), 'p' );
				else SendSerialData( md_fd, "CardType_Unknown\t\n", strlen("CardType_Unknown"), 'p' );
				break;
				
			case CARRIERSWITCH:
				SendSerialData( md_fd, "begi1 \r\n", strlen("begin \r\n"), 'p' );
				res = emvGuiToggleCarrier();
				if ( res == 0) SendSerialData( md_fd, "EMV: deactivating carrier", strlen("EMV: deactivating carrier"), 'p' );
				else if ( res == 1 ) SendSerialData( md_fd, "EMV: activating carrier", strlen("EMV: activating carrier\rn"), 'p' );
				break;
				
			case WUPA:
				res = emvGuiWupa( &serial_no[ 1 ], &serial_no[ 0 ]);
				if (  res == 0  ) SendSerialData( md_fd, (char*)&serial_no[ 1 ], serial_no[ 0 ], WUPA );	
				else if (  res == -1  ) SendSerialData( md_fd, "EMV: timeout error", strlen("EMV: timeout error"), WUPA );
				else if (  res == -2  ) SendSerialData( md_fd, "EMV: buffer overflow error", strlen("EMV: buffer overflow error"), WUPA );
				else if (  res == -3  ) SendSerialData( md_fd, "EMV: transmission error", strlen("EMV: transmission error"), WUPA );
				else if (  res == -4  ) SendSerialData( md_fd, "EMV: function call parameter error", strlen("EMV: function call parameter error"), WUPA );
				else	if ( res == -5 ) SendSerialData( md_fd, "EMV: carrier not activated", strlen("EMV: carrier not activated"), WUPA );
				break;
				
			case WUPARATS:
				res =  emvGuiRats( &EmvPicc);
				if (  res == 0  ) SendSerialData( md_fd, (char*)&EmvPicc.dPcdPicc, EmvPicc.uidLength, WUPARATS );				
				else	if ( res == -7 ) SendSerialData( md_fd, "EMV: carrier not activated", strlen("EMV: carrier not activated"), WUPARATS );
				else SendSerialData( md_fd, (char*)&res, sizeof(res), WUPARATS );
				break;

			case WUPB:
				res =  emvGuiWupb(&serial_no[ 1 ], (char*)&serial_no[ 0 ]);
				if (  res == 0  ) SendSerialData( md_fd, (char*)&serial_no[ 1 ], serial_no[ 0 ], WUPB );	
				else if (  res == -1  ) SendSerialData( md_fd, "EMV: timeout error", strlen("EMV: timeout error"), WUPB );
				else if (  res == -2  ) SendSerialData( md_fd, "EMV: buffer overflow error", strlen("EMV: buffer overflow error"), WUPB );
				else if (  res == -3  ) SendSerialData( md_fd, "EMV: transmission error", strlen("EMV: transmission error"), WUPB );
				else if (  res == -4  ) SendSerialData( md_fd, "EMV: function call parameter error", strlen("EMV: function call parameter error"), WUPB );
				else	if ( res == -5 ) SendSerialData( md_fd, "EMV: carrier not activated", strlen("EMV: carrier not activated"), WUPB );
				break;
				
			case WUPBATTRIB:
				res =  emvGuiAttrib( &EmvPicc );
				if (  res == 0  ) SendSerialData( md_fd, (char*)&EmvPicc.dPcdPicc, EmvPicc.uidLength, WUPBATTRIB );				
				else	if ( res == -7 ) SendSerialData( md_fd, "EMV: carrier not activated", strlen("EMV: carrier not activated"), WUPBATTRIB );
				else SendSerialData( md_fd, (char*)&res, sizeof(res), WUPARATS );
				break;
				
			case RESET:
				res = emvGuiReset();
				if (  res == 0  ) SendSerialData( md_fd, "EMV: reset operating field succesed", strlen("EMV: reset operating field succesed"), 'p' );
				else	if ( res == -7 ) SendSerialData( md_fd, "EMV: carrier not activated", strlen("EMV: carrier not activated"), 'p'  );
				else SendSerialData( md_fd, (char*)&res, sizeof(res), 'p' );
				break;

			case EMVDIGITAL:
				SendSerialData( md_fd, "begi8 \r\n", strlen("begin \r\n"), 'p' );
				res = emvGuiDigital( );
				if (  res == 0  ) SendSerialData( md_fd, "EMV: digital application finished.", strlen("EMV: digital application finished."), 'p'  );	
				else	if ( res == -6 ) SendSerialData( md_fd, "EMV: digital application stopped", strlen("EMV: digital application stopped"), 'p'  );
				else SendSerialData( md_fd, (char*)&res, sizeof(res), 'p' );
				break;

			case STOP_DIGITAL: //����ֹͣ digital.
				
				break;
				/*���������һ����ȥ�� .
				res = emvGuiPrevalidation();
				if (  res == 0  ) SendSerialData( md_fd, "EMV: prevalidation application finished.", strlen("EMV: prevalidation application finished."), 'p'  );	
				else	if ( res == -6 ) SendSerialData( md_fd, "EMV: prevalidation application stopped", strlen("EMV: prevalidation application stopped"), 'p'  );
				else SendSerialData( md_fd, &res, sizeof(res), 'p' );
				break;*/
		    case APPL_COM_REG_DUMP:
			    {
			        u8 regAddress = 0;
				g_aucParmBuf[ 4]+=0x30;	
				g_aucParmBuf[ 5]+=0x30;	
			        myTACE("READ: 0x%x:  COUNT: 0x%x\n", g_aucParmBuf[ 4], g_aucParmBuf[ 5 ]);
			        for (regAddress = g_aucParmBuf[ 4]; regAddress < g_aucParmBuf[ 4 ] + g_aucParmBuf[ 5 ]; regAddress++)
			            displayRegisterValue(regAddress);
			    }
			    break;
		    case APPL_COM_REG_WRITE:
				g_aucParmBuf[ 4]+=0x30;	
				g_aucParmBuf[ 5]+=0x30;
				myTACE("REG: 0x%x: set to 0x%x\n", g_aucParmBuf[ 4], g_aucParmBuf[ 5 ]);
		     		as3911WriteRegister(g_aucParmBuf[ 4 ], g_aucParmBuf[ 5 ]);
		    		break;		
		    case APPL_COM_REG_READ:
				show3911Reg();
		    		break;		
		    case APPL_SET_GAIN_ADD:
			{
				u8 temp0;//AS3911_REG_RX_CONF3
				as3911ReadRegister(AS3911_REG_RX_CONF3, &temp0);
				temp0=temp0>>5;
				temp0+=1;
				as3911WriteRegister(AS3911_REG_RX_CONF3, (temp0)<<5); 
				displayRegisterValue(AS3911_REG_RX_CONF3);
		    		break;	
			}
		    case APPL_SET_GAIN_SIN:
			{
				u8 temp0;//AS3911_REG_RX_CONF3
				as3911ReadRegister(AS3911_REG_RX_CONF3, &temp0);
				temp0=temp0>>5;
				temp0-=1;
				as3911WriteRegister(AS3911_REG_RX_CONF3, (temp0)<<5); 
				displayRegisterValue(AS3911_REG_RX_CONF3);
		    		break;	
			}	
		    case APPL_SET_TEST_MODUL:
			{
				appTestCmd ();
		    		break;	
			}					
		    default:
				//AS3911_close();
				printf("unkonw command type\r\n");
				break;
						
		}

		clear_serial_buffer();
	}
	
	close(md_fd);
	return 0;    
		
}


