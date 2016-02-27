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
//#include "cyfxgpif2config.h"


/* GPIO application initialization function. */

#define CY_FX_PWM_PERIOD                 (20 - 1)   /* PWM time period. */
#define CY_FX_PWM_50P_THRESHOLD          (10  - 1)   /* PWM threshold value for 50% duty cycle. */


CyU3PReturnStatus_t
CyFxGpioInit (void)
{
	CyU3PDebugPrint (4, "CyU3PGpioInit start4");
	CyU3PDebugPrint (0, "CyU3PGpioInit start0");
	CyU3PDebugPrint (8, "CyU3PGpioInit start8");

	while(1);
	return CY_U3P_SUCCESS;

    CyU3PGpioClock_t gpioClock;
    CyU3PGpioComplexConfig_t gpioConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Init the GPIO module. The GPIO block will be running
     * with a fast clock at SYS_CLK / 2 and slow clock is not
     * used. For the DVK, the SYS_CLK is running at 403 MHz.*/
    gpioClock.fastClkDiv = 2;
    gpioClock.slowClkDiv = 0;
    gpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    gpioClock.clkSrc = CY_U3P_SYS_CLK;
    gpioClock.halfDiv = 0;

    apiRetStatus = CyU3PGpioInit(&gpioClock, NULL);
    if (apiRetStatus != 0)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "CyU3PGpioInit failed, error code = %d\n", apiRetStatus);
    }

    /* Configure GPIO 50 as PWM output */
    gpioConfig.outValue = CyFalse;
    gpioConfig.inputEn = CyFalse;
    gpioConfig.driveLowEn = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.pinMode = CY_U3P_GPIO_MODE_PWM;
    gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
    gpioConfig.timerMode = CY_U3P_GPIO_TIMER_HIGH_FREQ;
    gpioConfig.timer = 0;
    gpioConfig.period = CY_FX_PWM_PERIOD;
    gpioConfig.threshold = CY_FX_PWM_50P_THRESHOLD;
    apiRetStatus = CyU3PGpioSetComplexConfig(50, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PGpioSetComplexConfig failed, error code = %d\n",
                apiRetStatus);
    }
    return apiRetStatus;
}


#define CY_FX_RQT_ID_CHECK                      (0xB0)
#define CY_FX_RQT_I2C_EEPROM_WRITE              (0xBA)
#define CY_FX_RQT_I2C_EEPROM_READ               (0xBB)

const uint8_t glFirmwareID[32] __attribute__ ((aligned (32))) = { 'F', 'X', '3', ' ', 'I', '2', 'C', '\0' };
static uint8_t glEp0Buffer[4096] __attribute__ ((aligned (32)));

static uint16_t glI2cPageSize = 0x40;   /* I2C Page size to be used for transfers. */

/* I2C read / write for programmer application. */
CyU3PReturnStatus_t
CyFxUsbI2cTransfer (
        uint16_t  byteAddress,
        uint8_t   devAddr,
        uint16_t  byteCount,
        uint8_t  *buffer,
        CyBool_t  isRead)
{
    CyU3PI2cPreamble_t preamble;
    uint16_t pageCount = (byteCount / glI2cPageSize);
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint16_t resCount = glI2cPageSize;

    if (byteCount == 0)
    {
        return CY_U3P_SUCCESS;
    }

    if ((byteCount % glI2cPageSize) != 0)
    {
        pageCount ++;
        resCount = byteCount % glI2cPageSize;
    }

    CyU3PDebugPrint (2, "I2C access - dev: 0x%x, address: 0x%x, size: 0x%x, pages: 0x%x.\r\n",
            devAddr, byteAddress, byteCount, pageCount);

    while (pageCount != 0)
    {
        if (isRead)
        {
            /* Update the preamble information. */
            preamble.length    = 4;
            preamble.buffer[0] = devAddr;
            preamble.buffer[1] = (uint8_t)(byteAddress >> 8);
            preamble.buffer[2] = (uint8_t)(byteAddress & 0xFF);
            preamble.buffer[3] = (devAddr | 0x01);
            preamble.ctrlMask  = 0x0004;

            status = CyU3PI2cReceiveBytes (&preamble, buffer, (pageCount == 1) ? resCount : glI2cPageSize, 0);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PDebugPrint (2, "read error - dev: 0x%x, address: 0x%x, size: 0x%x, pages: 0x%x status : %d.\r\n",
                        devAddr, byteAddress, byteCount, pageCount, status);
                return status;
            }
        }
        else /* Write */
        {
            /* Update the preamble information. */
            preamble.length    = 3;
            preamble.buffer[0] = devAddr;
            preamble.buffer[1] = (uint8_t)(byteAddress >> 8);
            preamble.buffer[2] = (uint8_t)(byteAddress & 0xFF);
            preamble.ctrlMask  = 0x0000;

            status = CyU3PI2cTransmitBytes (&preamble, buffer, (pageCount == 1) ? resCount : glI2cPageSize, 0);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PDebugPrint (2, "write error - dev: 0x%x, address: 0x%x, size: 0x%x, pages: 0x%x status : %d.\r\n",
                        devAddr, byteAddress, byteCount, pageCount, status);
                return status;
            }


            /* Wait for the write to complete. */
            preamble.length = 1;
            status = CyU3PI2cWaitForAck(&preamble, 200);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
        }

        /* An additional delay seems to be required after receiving an ACK. */
        CyU3PThreadSleep (1);

        /* Update the parameters */
        byteAddress  += glI2cPageSize;
        buffer += glI2cPageSize;
        pageCount --;
    }

    return CY_U3P_SUCCESS;
}


/* Callback to handle the USB Setup Requests and UVC Class events */
CyBool_t
CyFxUVCApplnUSBSetupCB_old (
        uint32_t setupdat0, /* SETUP Data 0 */
        uint32_t setupdat1  /* SETUP Data 1 */
        )
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
    //bTarget  = (bmReqType & CY_U3P_USB_TARGET_MASK);

    /* Handle supported vendor requests. */
    if (bType == CY_U3P_USB_VENDOR_RQT)
    {
        uint8_t  i2cAddr;

    	uvcHandleReq = CyTrue;

        switch (bRequest)
        {
            case CY_FX_RQT_ID_CHECK:
                CyU3PUsbSendEP0Data (8, (uint8_t *)glFirmwareID);
                break;

            case CY_FX_RQT_I2C_EEPROM_WRITE:
                //i2cAddr = 0xA0 | ((wValue & 0x0007) << 1);
            	i2cAddr = wValue & 0x00fe;
                status  = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
                if (status == CY_U3P_SUCCESS)
                {
                    CyFxUsbI2cTransfer (wIndex, i2cAddr, wLength,
                            glEp0Buffer, CyFalse);
                }
                break;

            case CY_FX_RQT_I2C_EEPROM_READ:
//                i2cAddr = 0xA0 | ((wValue & 0x0007) << 1);
            	i2cAddr = wValue & 0x00fe;
                CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
                status = CyFxUsbI2cTransfer (wIndex, i2cAddr, wLength,
                        glEp0Buffer, CyTrue);
                if (status == CY_U3P_SUCCESS)
                {
                    status = CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
                }
                break;

            default:
                // This is unknown request.
            	uvcHandleReq = CyFalse;
                break;
        }
        if (status != CY_U3P_SUCCESS)
        {
        	uvcHandleReq = CyFalse;
        }

    }
    /* Return status of request handling to the USB driver */
    return uvcHandleReq;
}
