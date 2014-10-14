# Comment/uncomment the following line to disable/enable debugging
#DEBUG = y

# Add your debugging flag (or not) to CFLAGS
# "-O" is needed to expand inlines


ifneq ($(KERNELRELEASE),)
EXTRA_CFLAGS += $(DEBFLAGS) -I..
# call from kernel build system  
  #obj-m	:= spi_rfid.o
  #obj-m := mymodule.o
  #mymodule-objs := file1.o file2.o file3.o
obj-m := spi_rfid.o
spi_rfid-y := main.o      
spi_rfid-y += As3911_api.o          
spi_rfid-y += as3911.o  
spi_rfid-y += as3911_com.o  
spi_rfid-y += as3911_gain_adjustment.o  
spi_rfid-y += as3911_interrupt.o 
spi_rfid-y += as3911_io.o     
spi_rfid-y += as3911_modulation_adjustment.o 
spi_rfid-y += crc.o             
spi_rfid-y += emv_digital.o             
spi_rfid-y += emv_display.o             
spi_rfid-y += emv_gui.o                  
spi_rfid-y += emv_hal.o  
spi_rfid-y += emv_layer4.o       
spi_rfid-y += emv_main.o        
spi_rfid-y += emv_picc.o      
spi_rfid-y += emv_poll.o       
spi_rfid-y += emv_prelayer4.o   
spi_rfid-y += emv_prevalidation.o
spi_rfid-y += emv_response_buffer.o
spi_rfid-y += emv_typeA.o         
spi_rfid-y += emv_typeB.o 
spi_rfid-y += logger.o 
spi_rfid-y += platform.o
spi_rfid-y += sleep.o 

else

  KERNELDIR="/home/ljj/work/scl_85000/linux-2.6.32.9"
  PWD       := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
	cp -f spi_rfid.ko /mnt/hgfs/wintolinux/lw_ftp

endif

clean:
	rm -rf *.o *~ core .depend .*.cmd modules.order *.ko *.mod.c *.symvers .tmp_versions 

depend .depend dep:
	$(CC) $(CFLAGS) -M *.c > .depend


ifeq (.depend,$(wildcard .depend))
include .depend
endif
