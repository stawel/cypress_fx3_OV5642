#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3usb.h>
#include <cyu3uart.h>
#include <cyu3gpif.h>
#include <cyu3i2c.h>
#include <cyu3gpio.h>
#include <cyu3pib.h>
#include <cyu3utils.h>

#include "uvc.h"
#include "sensor.h"
#include "camera_ptzcontrol.h"
#include "i2c.h"


#define WRITE              (0xBA)
#define READ               (0xBB)

void printState()
{
    uint8_t curState_p;
    CyU3PReturnStatus_t s;
    s = CyU3PGpifGetSMState(&curState_p);
    CyU3PDebugPrint (2, "printState: state %d %d\r\n", (int)curState_p, s);
}

CyBool_t SensorDebugUSBSetup (uint32_t setupdat0, uint32_t setupdat1)
{
    CyBool_t uvcHandleReq = CyFalse;
    uint32_t status;
    uint8_t  bRequest, bmReqType;
    uint8_t  bType;//, bTarget;
    uint16_t wValue, wIndex, wLength;

    /* Obtain Request Type and Request */
    bmReqType = (uint8_t)(setupdat0 & CY_FX_USB_SETUP_REQ_TYPE_MASK);

    bRequest  = (uint8_t)((setupdat0 & CY_FX_USB_SETUP_REQ_MASK) >> 8);
    wValue    = (uint16_t)((setupdat0 & CY_FX_USB_SETUP_VALUE_MASK) >> 16);
    wIndex    = (uint16_t)(setupdat1 & CY_FX_USB_SETUP_INDEX_MASK);
    wLength   = (uint16_t)((setupdat1 & CY_FX_USB_SETUP_LENGTH_MASK) >> 16);

    bType    = (bmReqType & CY_U3P_USB_TYPE_MASK);


    /* Handle supported vendor requests. */
    if (bType == CY_U3P_USB_VENDOR_RQT)
    {
    	printState();
        uint8_t  i2cAddr, value;
    	uvcHandleReq = CyTrue;
    	i2cAddr = wValue & 0x00ff;
    	value = wLength;
        switch (bRequest)
        {
            case WRITE:
            	status = SensorWrite1B(i2cAddr, wIndex, value);
                break;

            case READ:
            	i2cAddr = wValue & 0x00ff;
            	status = SensorRead1B(i2cAddr, wIndex, &value);
            	CyU3PDebugPrint (2, "SensorDebugUSBSetup:  i2c: %d  addr: %d  val: %d\r\n", (int)i2cAddr, (int)wIndex, (int)value);
                break;

            default:
            	uvcHandleReq = CyFalse;
                break;
        }
        if (status != CY_U3P_SUCCESS)
        {
        	uvcHandleReq = CyFalse;
        }

    }
    return uvcHandleReq;
}
