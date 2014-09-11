
#ifndef AS3911_INTERRUPT_H
#define AS3911_INTERRUPT_H




#include "ams_types.h"
#define TIMER_INTERRUPT_STATUS 0
/*
******************************************************************************
* DEFINES
******************************************************************************
*/

/*! \name Interrupt masks */
/**@{*/
/*! \ingroup as3911IrqHandling */
/* Main interrupt register. */
#define AS3911_IRQ_MASK_ALL             U32_C(0xFFFFFF)  /*!< All AS3911 interrupt sources. */
#define AS3911_IRQ_MASK_NONE            U32_C(0)        /*!< No AS3911 interrupt source. */
#define AS3911_IRQ_MASK_OSC             U32_C(0x80)     /*!< AS3911 oscillator stable interrupt. */
#define AS3911_IRQ_MASK_WL              U32_C(0x40)     /*!< AS3911 FIFO water level inerrupt. */
#define AS3911_IRQ_MASK_RXS             U32_C(0x20)     /*!< AS3911 start of receive interrupt. */
#define AS3911_IRQ_MASK_RXE             U32_C(0x10)     /*!< AS3911 end of receive interrupt. */
#define AS3911_IRQ_MASK_TXE             U32_C(0x08)     /*!< AS3911 end of transmission interrupt. */
#define AS3911_IRQ_MASK_COL             U32_C(0x04)     /*!< AS3911 bit collision interrupt. */

/* Timer and NFC interrupt register. */
#define AS3911_IRQ_MASK_DCT             U32_C(0x8000)   /*!< AS3911 termination of direct command interrupt. */
#define AS3911_IRQ_MASK_NRE             U32_C(0x4000)   /*!< AS3911 no-response timer expired interrupt. */
#define AS3911_IRQ_MASK_GPE             U32_C(0x2000)   /*!< AS3911 general purpose timer expired interrupt. */
#define AS3911_IRQ_MASK_EON             U32_C(0x1000)   /*!< AS3911 external field on interrupt. */
#define AS3911_IRQ_MASK_EOF             U32_C(0x0800)   /*!< AS3911 external field off interrupt. */
#define AS3911_IRQ_MASK_CAC             U32_C(0x0400)   /*!< AS3911 collision during RF collision avoidance interrupt. */
#define AS3911_IRQ_MASK_CAT             U32_C(0x0200)   /*!< AS3911 minimum guard time expired interrupt. */
#define AS3911_IRQ_MASK_NFCT            U32_C(0x0100)   /*!< AS3911 initiator bit rate recognized interrupt. */

/* Error and wake-up interrupt register. */
#define AS3911_IRQ_MASK_CRC             U32_C(0x800000) /*!< AS3911 CRC error interrupt. */
#define AS3911_IRQ_MASK_PAR             U32_C(0x400000) /*!< AS3911 parity error interrupt. */
#define AS3911_IRQ_MASK_ERR2            U32_C(0x200000) /*!< AS3911 soft framing error interrupt. */
#define AS3911_IRQ_MASK_ERR1            U32_C(0x100000) /*!< AS3911 hard framing error interrupt. */
#define AS3911_IRQ_MASK_WT              U32_C(0x080000) /*!< AS3911 wake-up interrupt. */
#define AS3911_IRQ_MASK_WAM             U32_C(0x040000) /*!< AS3911 wake-up due to amplitude interrupt. */
#define AS3911_IRQ_MASK_WPH             U32_C(0x020000) /*!< AS3911 wake-up due to phase interrupt. */
#define AS3911_IRQ_MASK_WCAP            U32_C(0x010000) /*!< AS3911 wake-up due to capacitance measurement. */

/**@}*/

/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/

/*! \ingroup as3911IrqHandling
 *****************************************************************************
 * \brief Enable processing of AS3911 interrupts by the PIC.
 *****************************************************************************
 */
//#define AS3911_IRQ_ON()  {; }

/*! \ingroup as3911IrqHandling
 *****************************************************************************
 *  \brief Disable processing of AS3911 interrupts by the PIC.
 *****************************************************************************
 */
//#define AS3911_IRQ_OFF() { ; }

/*
******************************************************************************
* GLOBAL DATA TYPES
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL VARIABLE DECLARATIONS
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/

/*! \ingroup as3911IrqHandling
 *****************************************************************************
 * \brief Enable certain AS3911 interrupts.
 *
 * \param[in] mask The interrupts to enable.
 *
 * \return ERR_IO: Error during communication.
 * \return ERR_NONE: No error, the given interrupts are enabled.
 *****************************************************************************
 */
s8 as3911EnableInterrupts(u32 mask);

/*! \ingroup as3911IrqHandling
 *****************************************************************************
 * \brief Disable certain AS3911 interrupts.
 *
 * \param[in] mask The interrupts to enable.
 *
 * \return ERR_IO: Error during communication.
 * \return ERR_NONE: No error, the given interrupts are disabled.
 *****************************************************************************
 */
s8 as3911DisableInterrupts(u32 mask);

/*! \ingroup as3911IrqHandling
 *****************************************************************************
 * \brief Clear pending AS3911 interrupts.
 *
 * \param[in] mask The interrupts to clear.
 *
 * \return ERR_IO: Error during communication.
 * \return ERR_NONE: No error, the given interrupts are cleard.
 *****************************************************************************
 */
s8 as3911ClearInterrupts(u32 mask);

/*! \ingroup as3911IrqHandling
 *****************************************************************************
 * \brief Wait for certain AS3911 interrupts.
 *
 * \param[in] mask The interrupts to wait for.
 * \param[in] timeout
 * \li timeout > 0: Wait operation timeout (milliseconds).
 * \li timeout == 0: No timeout, the function call blocks until one
 * of the interrupts defined by \a mask occur.
 * \param[out] irqs Set to the interrupt(s) wich occured.
 *
 * \note Any interrupt that occurs which is not selected by mask will not
 * cause the wait operation to terminate and will also not be reported in
 * \a irqs after the function call returns.
 * \note All reported interrupts are automatically cleared.
 *
 * \return ERR_NONE: No error, one of the defined interrupts occured or
 * the timeout expired.
 *****************************************************************************
 */
s8 as3911WaitForInterruptTimed(u32 mask, u16 timeout, u32 *irqs);
void  TimerStart( unsigned char TimerNo, int ms );
int TimerCheck( unsigned char TimerNo );
extern  u32 ggjiffies_count;

#define quck_printk(x) do{  \
if(++ggjiffies_count>6700)  \
	{  \
			ggjiffies_count=0; \
			printk(" %x (i)",x);  \
	}}while(0)

#define quck_printk2(x) do{  \
if(++ggjiffies_count>500)  \
	{  \
			ggjiffies_count=0; \
			printk(" %x (a)",x);  \
	}}while(0)

#define QUCK_PUB_INTERR_OFF() 	reg_gpio_disable_interrupt( BCM5892_GPB12 ) 
#define QUCK_PUB_INTERR_ON()	reg_gpio_enable_interrupt(BCM5892_GPB12)
/*! \ingroup as3911IrqHandling
 *****************************************************************************
 * \brief Get interrupt status of certain interrupts of the AS3911.
 *
 * \param[in] mask The interrupts which status has to be checked.
 * \param[out] irqs Set to the interrupts which are currently pending.
 *
 * \note A pending interrupt that is not selected for readout by \a mask
 * will not be reported.
 * \note All reported interrupts are automatically cleared.
 *
 * \return ERR_NONE: No error, interrupt status read out successful.
 *****************************************************************************
 */
s8 as3911GetInterrupts(u32 mask, u32 *irqs);
void as3911InterruptInit(void);
extern volatile u32 quck_InterruptStatus  ;
#endif /* AS3911_INTERRUPT_H */

