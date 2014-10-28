

#include "emv_display.h"

#include <linux/stddef.h>
#include "logger.h"

#include "emv_layer4.h"
#include "emv_response_buffer.h"
#include "main.h"

#if USE_LOGGER == LOGGER_ON
#define EMV_LOG dbgLog
#else
#define EMV_LOG(...)
#endif

#undef EMV_LOG
//#define EMV_LOG(...)   
#define EMV_LOG printk



void emvDisplayString(const char *string)
{
    EMV_LOG(string);
}

void emvDisplayByteArray(const u8 *array, u32 length)
{
    u32 index = 0;
    for (index = 0; index < length; index++)
        EMV_LOG(",%02x", array[index]);
}

void emvDisplayUid(const u8 *uid, u32 length)
{
    u32 index = length - 1;

    do
    {
        EMV_LOG("%02x", uid[index]);
        index--;
    } while (index > 0);
}

void emvDisplayError(s16 errorCode)
{
    switch (errorCode)
    {
    case EMV_ERR_OK:
        EMV_LOG("EMV: no error\n");
        break;
    case EMV_ERR_COLLISION:
        EMV_LOG("EMV: collision error\n");
        break;
    case EMV_ERR_PROTOCOL:
        EMV_LOG("EMV: protocol error\n");
        break;
    case EMV_ERR_TRANSMISSION:
        EMV_LOG("EMV: transmission error\n");
        break;
    case EMV_ERR_TIMEOUT:
        EMV_LOG("EMV: timeout error\n");
        break;
    case EMV_ERR_INTERNAL:
        EMV_LOG("EMV: internal error\n");
        break;
    case EMV_ERR_STOPPED:
        EMV_LOG("EMV: stopped error\n");
        break;
    default:
        EMV_LOG("EMV: unkown error code\n");
    }
}

void emvDisplayMessage(s16 messageCode)
{
    switch (messageCode)
    {
    case EMV_M_POLLING:
        EMV_LOG("EMV: Polling ...\n");
        break;
    case EMV_M_REMOVE_CARD:
        EMV_LOG("EMV: Remove card ...\n");
        break;
    default:
        EMV_LOG("EMV: unkown message code\n");
    }
}

void emvDisplayCAPDU(const u8 *apdu, u32 length)
{
    emvDisplayString("EMV: C-APDU ");
    emvDisplayByteArray(apdu, length);
    emvDisplayString("\n");
}

void emvDisplayRAPDU(const u8 *apdu, u32 length)
{
    emvDisplayString("EMV: R-APDU ");
    emvDisplayByteArray(apdu, length);
    emvDisplayString("\n");
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
