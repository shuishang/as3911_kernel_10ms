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

/*! \file main.h
 *
 *  \author F. Lobmaier
 *
 *  \brief Application header file
 *
 */

#ifndef MAIN_H
#define MAIN_H

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
//2014Äê5ÔÂ22ÈÕ13:43:22  by quck
#include <mach/reg_gpio.h>
#include "emv_display.h"
#include "sleep.h"
#include "As3911_def.h"
#include "platform.h"




void Spi_Select(void);
void Spi_Deselect(void);

void SSelect(void);
 
void SDeselect(void);
void QSelect(void);
 

 u8 QDeselect(void);
#define quck_read_printk(databuf,i)  quck_ssp_read_printk(databuf,i)
#define  quck_write_printk(databuf,i)   quck_ssp_write_printk(databuf,i)

#endif /* MAIN_H */
