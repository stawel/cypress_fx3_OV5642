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


/* GPIO application initialization function. */

#define CY_FX_PWM_PERIOD                 (8 - 1)   /* PWM time period. */
#define CY_FX_PWM_50P_THRESHOLD          (4 - 1)   /* PWM threshold value for 50% duty cycle. */


void CyFxAppErrorHandler (CyU3PReturnStatus_t apiRetStatus);

CyU3PReturnStatus_t SensorGpioInit (void)
{
	CyU3PDebugPrint (4, "SensorGpioInit start\n");

    CyU3PGpioComplexConfig_t gpioConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;


    apiRetStatus = CyU3PDeviceGpioOverride (SENSOR_XCLK_GPIO, CyFalse);
    if (apiRetStatus != 0)
    {
        CyU3PDebugPrint (4, "GPIO Override failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
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
    apiRetStatus = CyU3PGpioSetComplexConfig(SENSOR_XCLK_GPIO, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PGpioSetComplexConfig failed, error code = %d\n",
                apiRetStatus);
    }
    return apiRetStatus;
}
