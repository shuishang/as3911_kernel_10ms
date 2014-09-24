


#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h> 
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/fcntl.h> 
#include <linux/proc_fs.h>  
#include <linux/fs.h>   
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/sched.h>  // current and everything //
#include <linux/platform_device.h>
#include <linux/irq.h>

#include <mach/hardware.h>
#include <asm/uaccess.h> //copy_to_user()//
#include <asm/io.h>
#include <asm/atomic.h>
#include <asm/irq.h>

#include <asm/mach/map.h>
/*
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-gpioj.h>

*/
//2014年5月22日13:43:22  by quck
#include <mach/reg_gpio.h>

#include "As3911_api.h"
#include "emv_main.h"
#include "emv_picc.h"
#include "As3911_def.h"
#include <linux/strong_lion_def.h> 
#include "emv_gui.h"
//#include "main.h"
#include "sleep.h"
#include "as3911_interrupt.h"
/**mach-smdk2416.c*/
#define SPI_RFID_NAME		          "spi_rfid"

#define  SPI_RFID_IOC_MAGIC 	          'w'

#define IOC_SPI_ENABLE_IRQ        _IOW(SPI_RFID_IOC_MAGIC, 1, int )
#define IOC_SPI_STAUS_IRQ           _IOWR(SPI_RFID_IOC_MAGIC, 2, int )
//---------------------------------------------
#define SCL_VERSION "1.0"
#define MACHINE "Arm"
#define COMPILE_TIME "2014-01-08 11:28:06"
#define COMPILE_BY "ljj"
#define COMPILE_HOST "localhost.localdomain"
#define COMPILE_DOMAIN "localdomain"
#define COMPILER "gcc version 4.0.0 (DENX ELDK 4.1 4.0.0)"
#define KERNEL_VERSION 140108

#define ID_SCL_VERSION     "SCL_VERSION   :"
#define ID_MACHINE         "MACHINE       :"
#define ID_COMPILE_TIME    "COMPILE_TIME  :"
#define ID_COMPILE_BY      "COMPILE_BY    :"
#define ID_COMPILE_HOST    "COMPILE_HOST  :"
#define ID_COMPILE_DOMAIN  "COMPILE_DOMAIN:"
#define ID_COMPILER        "COMPILER      :"
#define ID_KERNEL_VERSION  "KERNEL_VERSION:"


/****************************************
RF_NSS :   GPD5   AS3911  主从信号,低电平从机
RF_MISO:  GPE15  读信号线
RF_MOSI : GPE12  写信号线
RF_SCLK:  GPE14  时钟线
RF_IRQ:    GPF3    中断信号线

SPI 时钟极性0   CPOL = 0  正向电平
       时钟相位1   CPHA = 1  时钟前沿数据输出，后沿采样
*****************************************/
#define SPI_CS_LOW        0
#define SPI_CS_HIGH       1

#define SPI_CLK_LOW       0
#define SPI_CLK_HIGH      1

#define SPI_DATA_RD        0
#define SPI_DATA_WRITE  1

#define SPI_BIT_LOW         0
#define SPI_BIT_HIGH        1

#define SPI_RFID_MAJOR             235

#define AS3911_STATUS_IRQ     IRQ_EINT3

#define SPI_PARA_CO		1
#define SPI_DELAY_BASE	1

#define BUF_SIZE		240


//--------------2014年8月13日11:02:09
#define RF_CS    BCM5892_GPA7  
#define RF_MISO    BCM5892_GPA5    
#define RF_MOSI  BCM5892_GPA6
#define RF_SCL     BCM5892_GPA4
#define RF_POWER     BCM5892_GPB31


static struct proc_dir_entry *Spi_rfid_ver;
const char version[]={ "as3911_src_v2[ "__TIME__"]\r\n"};
int opend  = 0;
int Spi_Status_Irq = 0;

unsigned char Snd_data_buf[ BUF_SIZE ];
unsigned char Rcv_data_buf[ BUF_SIZE ];

struct Spi_dev {
	struct cdev Spi_rfid_cdev;
	struct fasync_struct *async_queue;
};

struct Spi_dev *spi_devp; 

struct fasync_struct *async_queue;
#define quck_udelay(x)  quck_udelay_sub(x)
volatile int quck_time;
volatile int quck_time2;
//1000次大概是延迟0.97毫秒
inline void quck_udelay_sub(unsigned int t)
{
	
/*	int i;
	for(i=0;i<t;i++)
	{
	udelay(1);
	}*/
		
	unsigned int i;
	quck_time=t;
	for(i=0;i<quck_time;i++)
	{
		//for(quck_time2=0;quck_time2<37;quck_time2++);//这个大概是1us
		for(quck_time2=0;quck_time2<15;quck_time2++);
	}
}
void sleepMilliseconds(unsigned int t)
{
	//quck_udelay(t*1000);
	unsigned int i;
	
	for(i=0;i<t*1000;i++)
	{
		udelay(1); 
	 }
	//mdelay(milliseconds);
}

int Spi_rfid_fasync( int fd, struct file *filp, int mode )
{
	struct Spi_dev *dev = filp->private_data; 
	return fasync_helper( fd,filp,mode,&dev->async_queue );
}

/*
static irqreturn_t AS3911_Interrupt( int irq, void * dev_id )
{
	Spi_Status_Irq = 1;
	//printk(KERN_ERR " AS3911_Interrupt in\n" );
	kill_fasync(&spi_devp->async_queue,SIGIO,POLL_IN);
	return IRQ_HANDLED;			
}
*/
inline static void Spi_Set_Scl( int state )
{
	//change to output pin
	gpio_set_pin_val(RF_SCL,state);		

}

inline static void Spi_Write_Bit( int state )
{
	gpio_set_pin_val(RF_MOSI,state);	
	
	
}

inline static int Spi_Read_Bit( void )
{
	// GPE15 [31:30] 00= INPUT 01=OUTPUT 10=  用于读取数据//
	return reg_gpio_get_pin(RF_MISO);	
}

void Spi_Select(void)
{
	gpio_set_pin_val(RF_CS,0);	
}


void Spi_Deselect(void)
{
	gpio_set_pin_val(RF_CS,1);		
	return;
}
//TP8 ,B26.
 void SSelect(void)
{
	gpio_set_pin_type(BCM5892_GPB26, GPIO_PIN_TYPE_OUTPUT );
	gpio_set_pin_val(BCM5892_GPB26,0);	
}

 void SDeselect(void)
{
	gpio_set_pin_type(BCM5892_GPB26, GPIO_PIN_TYPE_OUTPUT );	
	gpio_set_pin_val(BCM5892_GPB26,1);		
	return;
}
//TP9,B27.
 void QSelect(void)
{
		gpio_set_pin_type(BCM5892_GPB27, GPIO_PIN_TYPE_OUTPUT );
	 	gpio_set_pin_val(BCM5892_GPB27,1);	
	 	reg_gpio_set_pull_up_down_enable(BCM5892_GPB27);
	 	gpio_set_pin_type(BCM5892_GPB27, GPIO_PIN_TYPE_INPUT );

}

 u8 QDeselect(void)
{
	  gpio_set_pin_type(BCM5892_GPB27, GPIO_PIN_TYPE_INPUT );
	  return reg_gpio_get_pin(BCM5892_GPB27);

}

/**************************************************************************
* 函数名称：int Spi_Write_Byte( unsigned char mode )
* 功能描述：
* 输入参数： 
* 输出参数： 0:成功 -1:失败
* 返 回 值   ： 
* 其它说明：
* 修改日期    版本号     修改人	     修改内容
* -----------------------------------------------
* 2014/01/02         V1.0	                liuwei	               XXXX
**************************************************************************/
int Spi_Write_Byte( unsigned char ucData )
{
	unsigned char ucTemp     = 0;
	unsigned char ucNumBit  = 0;

	ucTemp = ucData;  
	for ( ucNumBit = 0; ucNumBit < 8; ucNumBit++ )
	{
		Spi_Set_Scl( SPI_CLK_HIGH );

		if( ( ucTemp & 0x80 ) == 0x80 )
		{
			Spi_Write_Bit( SPI_BIT_HIGH );
		}
		else
		{
			Spi_Write_Bit( SPI_BIT_LOW );
		}

		quck_udelay( SPI_PARA_CO * SPI_DELAY_BASE );
		Spi_Set_Scl( SPI_CLK_LOW );
		quck_udelay( SPI_PARA_CO * SPI_DELAY_BASE );

		ucTemp <<= 1;
	}
	
	return 0;
}
/**************************************************************************
* 函数名称：int Spi_Write_Byte( unsigned char mode )
* 功能描述：
* 输入参数： 
* 输出参数： 0:成功 -1:失败
* 返 回 值   ： 
* 其它说明：
* 修改日期    版本号     修改人	     修改内容
* -----------------------------------------------
* 2014/01/02         V1.0	                liuwei	               XXXX
**************************************************************************/
int Spi_Read_Byte( void )
{
	unsigned char ucTemp            = 0;
	unsigned char ucNumBit         = 0;
	unsigned char ucReceiveBit   = 0;
	          
	for ( ucNumBit = 0; ucNumBit < 8; ucNumBit++ )
	{
		ucTemp <<= 1;
		Spi_Set_Scl( SPI_CLK_HIGH );       
		quck_udelay( SPI_PARA_CO * SPI_DELAY_BASE );


		  
        Spi_Set_Scl( SPI_CLK_LOW );
		ucReceiveBit = Spi_Read_Bit();
		if ( ucReceiveBit ) 
		{
			ucTemp++;         
		}		
		quck_udelay( SPI_PARA_CO * SPI_DELAY_BASE );
	}

	return( ucTemp );

}
static  int  Spi_rfid_open(struct inode *inode, struct file *filp)   
{	

	if ( opend )
	{
		return -EBUSY;
	}
	else
	{	
		printk("opend = 1;\n");
		opend = 1;
		filp->private_data = spi_devp;
	}
	
  	return 0;
}

static int  Spi_rfid_release(struct inode *inode, struct file *filp)
{	
	opend = 0;
	printk("opend = 0;\n");
	Spi_rfid_fasync( -1, filp, 0 );
  	return 0;
}

static ssize_t Spi_rfid_write(struct file *filp, const char *buf, u32 count, loff_t *f_pos)
{
	int ret     = 0;
	int iNum = 0;

	if ( count > BUF_SIZE )
	{
		return -EINVAL;
	}
	
	if ( !count )
	{
		return 0;	
	}
	
	if ( !( access_ok( VERIFY_READ, (void *)buf, count )))
	{
		return -EFAULT;
	}
	
	if ( copy_from_user( Snd_data_buf, buf, count ) )
	{
		printk("get user data from user failed.\n");
		return  -EFAULT;
	}
	
	//local_irq_save(flags);
	Spi_Select();
	for ( iNum = 0;iNum < count; iNum++ )
	{
		ret = Spi_Write_Byte( Snd_data_buf[ iNum ]);
		if ( ret )
		{
			return -EIO;
		}
	}
	
	Spi_Deselect();
	//local_irq_restore(flags);
	
	return 0;
}

static ssize_t  Spi_rfid_read(struct file *filp, char *buf, u32 count,loff_t *f_pos)
{
	int iNum                      = 0;
	int ret                           = 0;
	
	if ( count > BUF_SIZE )
	{
		return -EINVAL;
	}
	
	if ( !count )
	{
		return 0;
	}
	
	if ( ! access_ok( VERIFY_WRITE, ( void *)buf, count ))
	{
		return -EFAULT;
	}
	
	if ( copy_from_user( Snd_data_buf, buf, 1 ) )
	{
		printk("get user data from user failed.\n");
		return  -EFAULT;
	}

	//local_irq_save(flags);
	
	Spi_Select();
	ret = Spi_Write_Byte( Snd_data_buf[ 0 ] );
	if ( ret )
	{
		return -EIO;
	}
	
	for ( iNum = 0;iNum < count; iNum++ )
	{
		Rcv_data_buf[ iNum ] = Spi_Read_Byte( );
	}
	
	Spi_Deselect();
	//local_irq_restore(flags);
	
	if ( copy_to_user( buf + 1,Rcv_data_buf,  count ) )
	{
		printk("send user data from kernel failed.\n");
		return  -EFAULT;
	}
	
	return 0;
}

static int  Spi_rfid_ioctl(struct inode *inode,struct file *filp,unsigned int cmd, unsigned long arg)
{
	int ret=0;
	unsigned int ucValue = 0;
	unsigned int ucValue2 = 0;
	unsigned long flags;	
	if ( _IOC_TYPE(cmd) != SPI_RFID_IOC_MAGIC ) 
	{
		return -ENOTTY;
	}
	
	printk("  Spi_rfid_ioctl(%x);\n",cmd);
	switch ( cmd )
	{
		case IOC_SPI_ENABLE_IRQ :
			
			AS3911_init();
			get_user( ucValue, (unsigned char *) arg );
			if ( ucValue == 0 ) 
			{
				printk("  emvGuiDigital();4\n");
			//	local_irq_save(quck_InterruptStatus);
			//	emvGuiDigital();	
			}
			else 
			{
			} 

	while(1)
	{

		displayRegisterValue(0x3f);
		displayRegisterValue(01);
		udelay(77);
		if (QDeselect())break;

	}
						
	/*		
			while(1)
			{
				if (QDeselect())break;
 				SSelect();  //6000ê?1oá??.
	 			TimerStart( 0, 10 );	
 				while(TimerCheck( 0 )){   }	
 				SDeselect();
	 			TimerStart( 0, 10 );	
 				while(TimerCheck( 0 )){   }	
			}
				*/		
			break;
		case IOC_SPI_STAUS_IRQ:
			printk("  IOC_SPI_STAUS_IRQ\n");
			get_user( ucValue, (unsigned char *) arg );
			if ( 0 == ucValue )
			{
				Spi_Status_Irq = 0;
			}
			else 
			{
				put_user( Spi_Status_Irq,(unsigned char *) arg);
			}
			break;
		default:
			printk("SPI_RFID: no such command\n");
                 	return -EINVAL;							
	}

	return ret;
}

//0成功  ,其他失败.

u8 quck_read_printk(unsigned int fd,u8 *buf,u8 count)
{

	//Spi_Select();
	quck_ssp_read_printk(buf,count);
//	Spi_Deselect();
	return 0;

}
//0成功  ,其他失败.
//static ssize_t Spi_rfid_write(struct file *filp, const char *buf, u32 count, loff_t *f_pos)
u8 quck_write_printk(unsigned int fd, u8 * buf ,u8 count )
{

	

//	Spi_Select();
	quck_ssp_write_printk(buf,count);
	//Spi_Deselect();

	return 0;

}

struct file_operations  Spi_rfid_fops =
{
	.owner		= THIS_MODULE,
	.read		= Spi_rfid_read,
	.write	= Spi_rfid_write,
	.open		= Spi_rfid_open,
	.release		= Spi_rfid_release,
	.fasync               = Spi_rfid_fasync,
	.ioctl		         = Spi_rfid_ioctl,
};

static int proc_read_Spi_rfid_ver(char *page,char **start,off_t off, 
			                                                        int count,int *eof,void *data)
{
	int len;

	len = sprintf(page, "%s%s\n", ID_SCL_VERSION,SCL_VERSION);
	len += sprintf(page+len,"%s%s\n",ID_MACHINE,MACHINE);
	len += sprintf(page+len,"%s%s\n", ID_COMPILE_TIME,COMPILE_TIME);
	len += sprintf(page+len,"%s%s\n", ID_COMPILE_BY,COMPILE_BY);
	len += sprintf(page+len,"%s%s\n", ID_COMPILE_HOST,COMPILE_HOST);
	len += sprintf(page+len,"%s%s\n", ID_COMPILE_DOMAIN,COMPILE_DOMAIN);
	len += sprintf(page+len,"%s%s\n", ID_COMPILER,COMPILER);
	len += sprintf(page+len,"%s%d\n", ID_KERNEL_VERSION,KERNEL_VERSION);
	
	*eof = 1;

	return len;
}

static void hareware_init(void)
{

	gpio_set_pin_type(RF_POWER, GPIO_PIN_TYPE_OUTPUT );
        reg_gpio_set_pull_up_down_disable(RF_POWER);
	gpio_set_pin_val(RF_POWER,1);	

	//RF_CS	 BCM5892_GPA7   
	gpio_set_pin_type(RF_CS, GPIO_PIN_TYPE_OUTPUT );
    reg_gpio_set_pull_up_down_disable(RF_CS);
	gpio_set_pin_val(RF_CS,0);
	/*	

	quck_udelay(50000);
	//RF_MISO    BCM5892_GPA5    
	gpio_set_pin_val(RF_MISO,0);
	gpio_set_pin_type(RF_MISO, GPIO_PIN_TYPE_INPUT );
    reg_gpio_set_pull_up_down_disable(RF_MISO);

	//RF_MOSI  BCM5892_GPA6
	gpio_set_pin_type(RF_MOSI, GPIO_PIN_TYPE_OUTPUT );
    reg_gpio_set_pull_up_down_disable(RF_MOSI);
	gpio_set_pin_val(RF_MOSI,0);
	
	//RF_SCL     BCM5892_GPA4
	gpio_set_pin_type(RF_SCL, GPIO_PIN_TYPE_OUTPUT );
    reg_gpio_set_pull_up_down_disable(RF_SCL);
	gpio_set_pin_val(RF_SCL,0);
*/
	//RF_IRQ:   BCM5892_GPB12
	gpio_set_pin_type(BCM5892_GPB12, GPIO_PIN_TYPE_INPUT );
    reg_gpio_set_pull_up_down_disable(BCM5892_GPB12);
	gpio_set_pin_val(BCM5892_GPB12,0);
	
	QSelect();
	measure_counter_setup();
	measure_counter_stop();
	measure_counter_start();
	as3911InterruptInit();
	quck_ssp_start();	
	quck_ssp_setup();

}

static int Spi_rfid_probe(struct platform_device *pdev)
{
	int ret=0;

 	dev_t dev = MKDEV(SPI_RFID_MAJOR, 0);	

	hareware_init();	

	spi_devp = kmalloc(sizeof(struct Spi_dev),GFP_KERNEL);
	memset(spi_devp, 0, sizeof (struct Spi_dev));
	
	cdev_init(&spi_devp->Spi_rfid_cdev, &Spi_rfid_fops);	
	
	spi_devp->Spi_rfid_cdev.owner = THIS_MODULE;	
	spi_devp->Spi_rfid_cdev.ops = &Spi_rfid_fops;

	if ( cdev_add(&spi_devp->Spi_rfid_cdev, dev, 1) )	
	{		
		printk("Couldn't register Spi rfid driver\n");		
		ret = -ENOMEM;		
		goto out1;	
	}	

	if ( (ret = register_chrdev_region(dev, 1, SPI_RFID_NAME)) < 0)	
	{		
		printk("Spi RFID  register_chrdev_region error\n");		
		ret = -ENOMEM;		
		goto out2;	
	}	


	Spi_rfid_ver = create_proc_entry("strong_lion/version/Spi_rfid",0444,NULL);
	if (Spi_rfid_ver == NULL)
	{
		ret = -ENOMEM;
		goto out3;
	}
   
	Spi_rfid_ver->data = NULL;
	Spi_rfid_ver->read_proc = &proc_read_Spi_rfid_ver;
	//Spi_rfid_ver->owner = THIS_MODULE;

	printk(SPI_RFID_NAME " initialized\n");
	ret = 0;
	
	goto out0;

out3:	
	unregister_chrdev_region(dev, 1);
		
out2:	
	cdev_del(&spi_devp->Spi_rfid_cdev);

out1:	
	kfree(spi_devp);
out0:
	return ret;
}

static int Spi_rfid_remove(struct platform_device *pdev )
{
	dev_t dev = MKDEV(SPI_RFID_MAJOR, 0);	

	remove_proc_entry("strong_lion/version/Spi_rfid",NULL);	
	
	cdev_del(&spi_devp->Spi_rfid_cdev);	
	
	kfree(spi_devp);
	
	unregister_chrdev_region(dev, 1);
	free_irq(gpio_to_irq(BCM5892_GPB12),0);
	quck_ssp_stop();
	return 0;
}

#ifdef CONFIG_PM
static int Spi_rfid_suspend(struct platform_device *dev,pm_message_t state)
{
	printk("Spi_rfid_suspend\n");

	return 0;
}

static int Spi_rfid_resume(struct platform_device *dev)
{
	printk("Spi_rfid_resume\n");

	hareware_init();

	return 0;
}
#else
#define Spi_rfid_suspend NULL
#define Spi_rfid_resume NULL
#endif

static struct platform_driver Spi_rfid_driver = {
	.probe		= Spi_rfid_probe,
	.remove	 	= Spi_rfid_remove,
	.suspend	 	= Spi_rfid_suspend,
	.resume	 	= Spi_rfid_resume,
	.driver	 	= {
		.name	 = "spi_rfid",
	},
};


static void idtech_enmsr_release(struct device *dev)
{

}

static struct platform_device idtech_enmsr_device = {
	.name = "spi_rfid",
	.dev = {
		//.platform_data = &idtech_pdata,
		.release = idtech_enmsr_release,
	},
};

static int __init init_Spi_rfid( void )
{
    int ret ;
	ret = platform_device_register(&idtech_enmsr_device);
	if (ret) {
		printk("enmsr: register platform device failed: %d\n", ret);
	}
	printk((char*)version);
	printk(" %x \n",HZ);
	printk(" %lx \n",jiffies);
	return platform_driver_register(&Spi_rfid_driver);
}

static void __exit cleanup_Spi_rfid( void )
{
	platform_driver_unregister(&Spi_rfid_driver);
	platform_device_unregister(&idtech_enmsr_device);
	printk("unregister SPI RFID module\n");
}

module_init(init_Spi_rfid);
module_exit(cleanup_Spi_rfid);

MODULE_AUTHOR("lxyvslyr@yahoo.com.cn");
MODULE_DESCRIPTION(" Spi_rfid  for strong lion pos");
MODULE_LICENSE("GPL");


