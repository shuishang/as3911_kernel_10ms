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
 * PROJECT: AS3911 firmware
 * $Revision: $
 * LANGUAGE: ANSI C
 */

/*! \file as3911_com.c
 *
 * \author Oliver Regenfelder
 *
 * \brief AS3911 RFID communication
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "as3911_com.h"

#include "aS3911.h"
#include "logger.h"

/*
******************************************************************************
* DEFINES
******************************************************************************
*/

/*! Sanity timeout during an AS3911 transmit operation (milliseconds).
 * If no interrupt is signaled by the AS3911 within this timeframe during
 * a tranmsit operation then a severe software or hardware failure has to be
 * asumed.
 */
#define AS3911_TX_IRQ_SANITY_TIMEOUT        100

/*! Sanity timeout during an AS3911 receive operation (milliseconds).
 * If no interrupt is signaled by the AS3911 within this timeframe during
 * a receive operation then a severe software or hardware failure has to be
 * asumed.
 */
#define AS3911_RX_IRQ_SANITY_TIMEOUT        6000

/*! Time offset between the start of a PICC response and the start of
 * receive interrupt (carrier cycles).
 */
#define AS3911_SOR_IRQ_OFFSET               640

/*! Maximum size of a continuous FIFO read operation inside the irq handler.
 * Limiting the fifo read size is necessary to enable proper handling of
 * EMV transmission error recovery. A complete FIFO read time would take
 * to long and exceed the maximum response time to a transmission error
 * as defined by the EMV standard.
 */
#define AS3911_IRQ_FIFO_READ_CHUNK_SIZE     10

/*! Bitmask for the rxon bit of the auxiliary display register of the AS3911. */
#define AS3911_AUX_DISPLAY_RXON_BIT         0x08

/*
******************************************************************************
* MACROS
******************************************************************************
*/

/*
******************************************************************************
* LOCAL DATA TYPES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/

/*! Minimum delay between a received PICC message and the next PCD message (carrier cycles). */
static u32 as3911FrameDelayTime = 0;

/*! Indicates whether a reception is currently in progress (TRUE) or not (FALSE). */
static volatile bool_t as3911ReceptionInProgress = FALSE;

/*! \ToDo: Remove this variable. It is no longer used.
 */
static volatile bool_t as3911TransmissionInProgress = FALSE;

/*!
 *****************************************************************************
 * \brief Enable exception processing according to the EMV standard.
 *
 * If \a as3911EmvExceptionProcessing is set to TRUE, then the receiver will be
 * reenabled  and the received data ignored if certain combinations of receive
 * errors occure.
 *****************************************************************************
 */
static bool_t as3911EmvExceptionProcessing = FALSE;

/*!
 *****************************************************************************
 * This variable is only considered if emv_exception_processing is enabled.
 *
 * If a frame is received with a data encoding (parity), or crc error
 * AND the length of the frame is <= \a as3911TransmissionErrorThreshold then
 * the receieved frame is asumed to be noise and the receiver is reseted to
 * receive another frame.
 *****************************************************************************
 */
static u32 as3911TransmissionErrorThreshold = 0;

/*
******************************************************************************
* LOCAL TABLES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/

/*!
 *****************************************************************************
 * \brief Check whether the frame delay timer is running.
 *
 * \return TRUE: The FDT is running.
 * \return FALSE: The FDT is not running.
 *****************************************************************************
 */
static bool_t as3911FdtIsRunning(void);

/*
******************************************************************************
* GLOBAL VARIABLE DEFINITIONS
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/

void as3911EnableEmvExceptionProcessing(bool_t enableEmvExceptionProcessing)
{
    as3911EmvExceptionProcessing = enableEmvExceptionProcessing;
}

bool_t as3911EmvExceptionProcessingIsEnabled(void)
{
    return as3911EmvExceptionProcessing;
}

void as3911SetTransmissionErrorThreshold(u32 transmissionErrorThreshold)
{
    as3911TransmissionErrorThreshold = transmissionErrorThreshold;
}

u32 as3911GetTransmissionErrorThreshold(void)
{
    return as3911TransmissionErrorThreshold;
}

void as3911SetReceiverDeadtime(u32 receiverDeadtime)
{
    u8 reg_mode;
    u8 mrt_value;

    as3911ReadRegister(AS3911_REG_MODE, &reg_mode);
    if ((reg_mode & 0x78) == 0x00)
    {
        /* NFCIP1 mode. */
        mrt_value = (u8) ((receiverDeadtime >> 9) & 0xFF);
    }
    else
    {
        /* Non NFCIP1 mode. */
        mrt_value = (u8) ((receiverDeadtime >> 6) & 0xFF);
    }

    as3911WriteRegister(AS3911_REG_MASK_RX_TIMER, mrt_value);
}

void as3911SetReceiveTimeout(u32 receiveTimeout)
{
    u16 nrtValue;

    if (receiveTimeout > U32_C(0x003FFFC0))
    {
        /* Use 1:4096 prescaler. */
        nrtValue = (u16) (receiveTimeout >> 12);

        /* Round up if necessary. */
        if ((((u16) receiveTimeout) & 0x0FFF) != 0)
            nrtValue += 1;

        as3911ModifyRegister(AS3911_REG_GPT_CONF, 0x03, 0x03);
    }
    else
    {
        /* Use 1:64 prescaler. */
        nrtValue = (u16) (receiveTimeout >> 6);

        /* Round up if necessary. */
        if ((((u16) receiveTimeout) & 0x03F) != 0)
            nrtValue += 1;

        as3911ModifyRegister(AS3911_REG_GPT_CONF, 0x03, 0x02);
    }

    as3911WriteRegister(AS3911_REG_NO_RESPONSE_TIMER1, (u8) (nrtValue >> 8));
    as3911WriteRegister(AS3911_REG_NO_RESPONSE_TIMER2, (u8) (nrtValue & 0xFF));
}

void as3911SetFrameDelayTime(u32 frameDelayTime)
{
    u16 gptValue;

    as3911FrameDelayTime = frameDelayTime;

    /* Start GPT Timer only with start command */
    as3911ModifyRegister(AS3911_REG_GPT_CONF, 0xE0, 0x20); // 0x00
    
    gptValue = (u16) (as3911FrameDelayTime >> 3);
    as3911WriteRegister(AS3911_REG_GPT1, (u8) ((gptValue >> 8) & 0xFF));
    as3911WriteRegister(AS3911_REG_GPT2, (u8) (gptValue & 0xFF));
}

u32 as3911GetFrameDelayTime(void)
{
    return as3911FrameDelayTime;
}

void as3911Transmit(const u8 *message, u32 messageLength, AS3911RequestFlags_t requestFlags)
{
    u8 numBytesForFifo = 0;
    u32 numBytesTransmitted = 0;
    u32 irqStatus = 0;
    AS3911RequestFlags_t transmissionModeFlags = requestFlags & AS3911_TX_MODE_MASK;

    /* Clear FIFO. */
    as3911ExecuteCommand(AS3911_CMD_CLEAR_FIFO);
	
	/* Reset squelch gain reduction. */
	as3911ExecuteCommand(AS3911_CMD_CLEAR_SQUELCH);

    if (transmissionModeFlags == AS3911_TRANSMIT_WUPA)
    {
        /* Enable anticollision frame handling. */
        as3911ModifyRegister(AS3911_REG_ISO14443A_NFC, 0x01, 0x01);
        as3911ExecuteCommand(AS3911_CMD_TRANSMIT_WUPA);
    }
    else if (transmissionModeFlags == AS3911_TRANSMIT_REQA)
    {
        /* Enable anticollision frame handling. */
        as3911ModifyRegister(AS3911_REG_ISO14443A_NFC, 0x01, 0x01);
        as3911ExecuteCommand(AS3911_CMD_TRANSMIT_REQA);
    }
    else
    {
        /* Setup number of bytes to send. */
		as3911WriteRegister(AS3911_REG_NUM_TX_BYTES1, (messageLength >> 5) & 0xFF);
        as3911WriteRegister(AS3911_REG_NUM_TX_BYTES2, (messageLength << 3) & 0xFF);

        if (messageLength > AS3911_FIFO_SIZE)
            numBytesForFifo = AS3911_FIFO_SIZE;
        else
            numBytesForFifo = messageLength;

        /* Load first part of the message into the FIFO. */
        as3911WriteFifo(message, numBytesForFifo);
        numBytesTransmitted = numBytesForFifo;

        /* Start transmission. */
        if (transmissionModeFlags == AS3911_TRANSMIT_WITHOUT_CRC)
            as3911ExecuteCommand(AS3911_CMD_TRANSMIT_WITHOUT_CRC);
        else
            as3911ExecuteCommand(AS3911_CMD_TRANSMIT_WITH_CRC);
    }

    do
    {
        as3911WaitForInterruptTimed(AS3911_IRQ_MASK_TXE | AS3911_IRQ_MASK_WL,
            AS3911_TX_IRQ_SANITY_TIMEOUT, &irqStatus);

        if (irqStatus & AS3911_IRQ_MASK_WL)
        {
            /* FIFO water level interrupt. */
            if(numBytesTransmitted < messageLength)
            {
                u32 numBytesForFifo = messageLength - numBytesTransmitted;

                if(numBytesForFifo > (AS3911_FIFO_SIZE - AS3911_FIFO_TRANSMIT_WL0))
                {
                    numBytesForFifo = AS3911_FIFO_SIZE - AS3911_FIFO_TRANSMIT_WL0;
                }

                as3911WriteFifo(&message[numBytesTransmitted], (u8) numBytesForFifo);
                numBytesTransmitted += numBytesForFifo;
            }
        }
        if (irqStatus & AS3911_IRQ_MASK_TXE)
        {
            return;
        }
    } while (0 != irqStatus);

    /* This code is only reached when the AS3911 interrupt sanity timeout expires. */
    return;
}

s16 as3911Receive(u8 *response, u32 maxResponseLength, u32 *responseLength)
{
    u32 irqStatus = 0;
    u32 numBytesReceived = 0;
    bool_t receiving = FALSE;
    bool_t nrtExpired = FALSE;
    bool_t overflowOccured = FALSE;

//    LOG("----------------as3911Receive\n");
    if(responseLength != NULL)
    {
	    *responseLength = 0;
    }	
	
    do
    {

        as3911WaitForInterruptTimed(
            AS3911_IRQ_MASK_RXS | AS3911_IRQ_MASK_RXE | AS3911_IRQ_MASK_WL | AS3911_IRQ_MASK_NRE,
            AS3911_RX_IRQ_SANITY_TIMEOUT,
            &irqStatus
            );

        if (irqStatus & AS3911_IRQ_MASK_WL)
        {
            /* FIFO water level interrupt. */
            u8 auxDisplay = 0;
            u8 fifoStatus[2];
            u8 numBytesInFifo = 0;

            /* Read data from the FIFO in chunks as long as FIFO data is available
             * and the chip is receiving. Reading the FIFO data in chunks asures that
             * a too long FIFO read doesn't interfere with the error handling required
             * at the end of receive interrupt.
             */
            as3911ContinuousRead(AS3911_REG_FIFO_RX_STATUS1, &fifoStatus[0], 2);
            numBytesInFifo = fifoStatus[0] & 0x7F;
            
            as3911ReadRegister(AS3911_REG_AUX_DISPLAY, &auxDisplay);
            while ((auxDisplay & AS3911_AUX_DISPLAY_RXON_BIT) && (numBytesInFifo > 0))
            {
                u8 fifoReadSize = 0;

                fifoReadSize = (numBytesInFifo > AS3911_IRQ_FIFO_READ_CHUNK_SIZE) ? AS3911_IRQ_FIFO_READ_CHUNK_SIZE : numBytesInFifo;

                if (numBytesReceived + fifoReadSize > maxResponseLength)
                {
                    fifoReadSize = maxResponseLength - numBytesReceived;
                    overflowOccured = TRUE;
                }
                
                as3911ReadFifo(&response[numBytesReceived], fifoReadSize);
                numBytesReceived += fifoReadSize;
                numBytesInFifo -= fifoReadSize;

                as3911ReadRegister(AS3911_REG_AUX_DISPLAY, &auxDisplay);
            }

            if (!(auxDisplay & AS3911_AUX_DISPLAY_RXON_BIT))
                irqStatus |= AS3911_IRQ_MASK_RXE;
        }
        if (irqStatus & AS3911_IRQ_MASK_RXS)
        {
            /* start of receive interrupt. */
            receiving = TRUE;
        }
        if (irqStatus & AS3911_IRQ_MASK_NRE)
        {
            /* no response timer expired. */
            if (!receiving)
            {
//                LOG("----------------nrt time out\n");
                return AS3911_TIMEOUT_ERROR;
            }
             else
                nrtExpired = TRUE;
//            	LOG("----------------nrt time out after rxs\n");
        }
        if (irqStatus & AS3911_IRQ_MASK_RXE)
        {
            /* End of reception. */
            u8 fifoStatus[2];
            u32 errorIrqStatus = 0;
            u8 numBytesInFifo = 0;
            bool_t reenableReceiver = FALSE;

            as3911ContinuousRead(AS3911_REG_FIFO_RX_STATUS1, &fifoStatus[0], 2);
            numBytesInFifo = fifoStatus[0] & 0x7F;

            as3911GetInterrupts(
                AS3911_IRQ_MASK_COL | AS3911_IRQ_MASK_CRC | AS3911_IRQ_MASK_PAR | AS3911_IRQ_MASK_ERR1 | AS3911_IRQ_MASK_ERR2,
                &errorIrqStatus
                );

            if (fifoStatus[1] & 0x20)
                overflowOccured = TRUE;

            /* Residual bits are handled as hard framing error. */
            if ((fifoStatus[1] & 0x11) != 0x00)
                errorIrqStatus |= AS3911_IRQ_MASK_ERR2;

            if(as3911EmvExceptionProcessing)
            {
                if (  (errorIrqStatus & AS3911_IRQ_MASK_COL)
                   || (errorIrqStatus & AS3911_IRQ_MASK_ERR1)
                   || (errorIrqStatus & AS3911_IRQ_MASK_ERR2))
                    reenableReceiver = TRUE;

                if (  (errorIrqStatus & AS3911_IRQ_MASK_PAR)
                   && ((numBytesReceived + numBytesInFifo) < as3911TransmissionErrorThreshold))
                    reenableReceiver = TRUE;

                /* Exclude messages shorter than 2 bytes from CRC checking as they can not
                 * contain a CRC. This is done specifically to fullfil test case TB306.12
                 * of the EMV_CL_PCD_TEST_BENCH_TEST_CASES_v2.0.1a, which requires a too
                 * short message (1 byte) without a CRC to be handled as protocol error.
                 */
                if (  (errorIrqStatus & AS3911_IRQ_MASK_CRC)
                   && ((numBytesReceived + numBytesInFifo) < as3911TransmissionErrorThreshold)
                   && ((numBytesReceived + numBytesInFifo) >= 2))
                    reenableReceiver = TRUE;
            }

            if (reenableReceiver)
            {
                /* Reset the receive buffer. */
                numBytesReceived = 0;

                /* If no receive timeout has occured yet then the
                 * receiver needs to be reenabled. If a receive timeout
                 * has already occured then we have a reception timeout.
                 */
                if (!nrtExpired)
                {
                    receiving = FALSE;
                    as3911ExecuteCommand(AS3911_CMD_CLEAR_FIFO);
                    as3911ExecuteCommand(AS3911_CMD_UNMASK_RECEIVE_DATA);
                }
                else
                {
                    return AS3911_TIMEOUT_ERROR;
                }
            }
            else
            {
                /* Read data from the FIFO. */
                if (numBytesReceived + numBytesInFifo > maxResponseLength)
                {
                    as3911ReadFifo(&response[numBytesReceived], maxResponseLength - numBytesReceived);
                    numBytesReceived = maxResponseLength;
                    overflowOccured = TRUE;
                }
                else
                {
                    as3911ReadFifo(&response[numBytesReceived], numBytesInFifo);
                    numBytesReceived += numBytesInFifo;
                }

                /* Stop NRT timer. */
   // modify date: 2013-12-31
                as3911ModifyRegister(AS3911_REG_GPT_CONF, 0x02, 0x00);
                as3911ExecuteCommand(AS3911_CMD_CLEAR_FIFO);

                /* Start GPT timer used for FDT_PCD. */
                as3911ExecuteCommand(AS3911_CMD_START_GP_TIMER);

                as3911ReceptionInProgress = FALSE;
		if(responseLength != NULL)
		{
		 *responseLength = numBytesReceived;
		}				
                if (errorIrqStatus & AS3911_IRQ_MASK_COL)
                    return AS3911_COLLISION_ERROR;
                else if (errorIrqStatus & AS3911_IRQ_MASK_ERR1)
                    return AS3911_HARD_FRAMING_ERROR;
                else if (errorIrqStatus & AS3911_IRQ_MASK_CRC)
                    return AS3911_CRC_ERROR;
                else if (errorIrqStatus & AS3911_IRQ_MASK_PAR)
                    return AS3911_PARITY_ERROR;
                else if (errorIrqStatus & AS3911_IRQ_MASK_ERR2)
                {
                    LOG("Soft framing error\n");

                    return AS3911_SOFT_FRAMING_ERROR;
                }
                 else if (overflowOccured)
                    return AS3911_OVERFLOW_ERROR;
                else
                    return AS3911_NO_ERROR;
            }
        }
    } while (0 != irqStatus);

    /* We only reach this code if the IRQ sanity timeout expired. */
    return AS3911_INTERNAL_ERROR;
}

s16 as3911Transceive(const u8 *request, u32 requestLength
    , u8 *response, u32 maxResponseLength, u32 *responseLength
    , u32 timeout, AS3911RequestFlags_t requestFlags)
{
    s16 returnValue = AS3911_NO_ERROR;

    /* When the CRC is ignored, then it is automatically but into the FIFO by
     * the AS3911.
     */
    if (requestFlags & AS3911_IGNORE_CRC)
    {
        as3911ModifyRegister(AS3911_REG_ISO14443A_NFC, 0x01, 0x01);
        as3911ModifyRegister(AS3911_REG_AUX, 0xC0, 0x80);
    }
    else if (requestFlags & AS3911_CRC_TO_FIFO)
    {
        as3911ModifyRegister(AS3911_REG_ISO14443A_NFC, 0x01, 0x00);
        as3911ModifyRegister(AS3911_REG_AUX, 0xC0, 0x40);
    }
    else
    {
        as3911ModifyRegister(AS3911_REG_ISO14443A_NFC, 0x01, 0x00);
        as3911ModifyRegister(AS3911_REG_AUX, 0xC0, 0x00);
    }

    /* Adjust modulation level. */
    as3911AdjustModulationLevel();
	
    /* Adjust gain. */
    as3911AdjustGain();

    /* Wait for the frame delay time to pass. */
    while (as3911FdtIsRunning())
        ;

    /* Setup timeout. */
    as3911SetReceiveTimeout(timeout);

    /* Reset irq status flags. */
    as3911ClearInterrupts(AS3911_IRQ_MASK_ALL);

    /* Enable interrupts. */
    as3911EnableInterrupts(AS3911_IRQ_MASK_ALL);
    AS3911_IRQ_ON();

    as3911Transmit(request, requestLength, requestFlags);
    returnValue = as3911Receive(response, maxResponseLength, responseLength);

    /* Restore old irq settings. */
    /* Disable AS3911 interrupts. */
    as3911DisableInterrupts(AS3911_IRQ_MASK_ALL);
    AS3911_IRQ_OFF();

    return returnValue;
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/

static bool_t as3911FdtIsRunning()
{
    u8 aux = 0;
    as3911ReadRegister(AS3911_REG_VSS_REGULATOR_RESULT, &aux);

    if (aux & 0x04)
        return TRUE;
    else
        return FALSE;
}
