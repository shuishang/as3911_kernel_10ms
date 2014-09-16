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

/*! \file sleep.h
 *
 *  \author Oliver Regenfelder
 *
 *  \brief Sleep function.
 */

#ifndef SLEEP_H
#define SLEEP_H

/*! \defgroup sleep Sleep
 *****************************************************************************
 * \brief Provides a sleep function.
 *****************************************************************************
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "ams_types.h"

/*
******************************************************************************
* DEFINES
******************************************************************************
*/

/*
******************************************************************************
* MACROS
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/

/*! \ingroup sleep
 *****************************************************************************
 * \brief Sleep for a number of miliseconds.
 *
 * \param milliseconds Milliseconds to sleep.
 *****************************************************************************
 */
void sleepMilliseconds(unsigned int milliseconds);
void measure_counter_setup(void);
void measure_counter_start(void);
void measure_counter_stop(void);
unsigned int get_timer_count(void);
void quck_timer_count(u8 flag);


  void quck_ssp_setup(void);
 void quck_ssp_start(void);
 void quck_ssp_stop(void);
unsigned char  quck_ssp_read_printk(u8 *buf,u8 count);
u8 quck_ssp_write_printk( u8 * buf ,u8 count );




  
#endif /* SLEEP_H */
