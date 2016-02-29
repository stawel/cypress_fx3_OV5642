/*
 ## Cypress FX3 Camera Kit source file (sensor.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2012,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
 */

/* This file implements the I2C based driver for an image sensor that uses I2C
 for control in the FX3 HD 720p camera kit.
 */

#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3uart.h>
#include <cyu3i2c.h>
#include <cyu3types.h>
#include <cyu3gpio.h>
#include <cyu3utils.h>
#include "sensor.h"


static void SensorI2CAccessDelay(CyU3PReturnStatus_t status) {
	/* Add a 10us delay if the I2C operation that preceded this call was successful. */
	if (status == CY_U3P_SUCCESS)
		CyU3PBusyWait(10);
}


CyU3PReturnStatus_t SensorWrite1B(uint8_t slaveAddr, uint16_t addr, uint8_t data)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PI2cPreamble_t preamble;
	uint8_t buf[2];

	preamble.buffer[0] = slaveAddr;
	preamble.buffer[1] = addr>>8;
	preamble.buffer[2] = addr&0xff;
	preamble.length = 3; /*  Three byte preamble. */
	preamble.ctrlMask = 0x0000; /*  No additional start and stop bits. */

	buf[0] = data;

	apiRetStatus = CyU3PI2cTransmitBytes(&preamble, buf, 1, 0);
	SensorI2CAccessDelay(apiRetStatus);

	return apiRetStatus;
}

struct addrval_list {
 uint16_t addr;
 uint8_t value;
 };


CyU3PReturnStatus_t SensorConfig(struct addrval_list * config, int size)
{
	for(int i=0;i<size;i++) {
		if(SensorWrite1B(SENSOR_ADDR_WR, config[i].addr, config[i].value) != CY_U3P_SUCCESS) {
			CyU3PDebugPrint(4, "Error: SensorConfig: [%d] %x := %x !\r\n", i, config[i].addr, config[i].value);
			return 1;
		}
	}
	return CY_U3P_SUCCESS;
}


CyU3PReturnStatus_t SensorRead1B(uint8_t slaveAddr, uint16_t addr, uint8_t *buf)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PI2cPreamble_t preamble;

	preamble.buffer[0] = slaveAddr & I2C_SLAVEADDR_MASK; /*  Mask out the transfer type bit. */
	preamble.buffer[1] = addr >> 8;
	preamble.buffer[2] = addr & 0xff;
	preamble.buffer[3] = slaveAddr;
	preamble.length = 4;
	preamble.ctrlMask = 0x0004; /*  Send start bit after third byte of preamble. */

	apiRetStatus = CyU3PI2cReceiveBytes(&preamble, buf, 1, 0);
	SensorI2CAccessDelay(apiRetStatus);

	return apiRetStatus;
}


/*
 * Reset the image sensor using GPIO.
 */
void SensorReset(void) {
	CyU3PReturnStatus_t apiRetStatus;

	/* Drive the GPIO low to reset the sensor. */
	apiRetStatus = CyU3PGpioSetValue(SENSOR_RESET_GPIO, CyFalse);
	if (apiRetStatus != CY_U3P_SUCCESS) {
		CyU3PDebugPrint(4, "GPIO Set Value Error, Error Code = %d\n",
				apiRetStatus);
		return;
	}

	/* Wait for some time to allow proper reset. */
	CyU3PThreadSleep(10);

	/* Drive the GPIO high to bring the sensor out of reset. */
	apiRetStatus = CyU3PGpioSetValue(SENSOR_RESET_GPIO, CyTrue);
	if (apiRetStatus != CY_U3P_SUCCESS) {
		CyU3PDebugPrint(4, "GPIO Set Value Error, Error Code = %d\n",
				apiRetStatus);
		return;
	}

	/* Delay the allow the sensor to power up. */
	CyU3PThreadSleep(10);
	return;
}


//OV5642_RGB_QVGA
static struct addrval_list ov5642_init2[] = {
#include "config.h"
};


#define sizeofArray(x) (sizeof(x)/sizeof(x[0]))

/* Image sensor initialization sequence. */
void SensorInit(void) {
	if (SensorI2cBusTest() != CY_U3P_SUCCESS) /* Verify that the sensor is connected. */
	{
		CyU3PDebugPrint(4, "Error: Reading Sensor ID failed!\r\n");
		return;
	}

	SensorConfig(ov5642_init2, sizeofArray(ov5642_init2));

	/* Update sensor configuration based on desired video stream parameters. Using 720p 30fps as default setting.*/
	//SensorScaling_HD720p_30fps();

//	SensorConfig(ov5642_final, sizeofArray(ov5642_final));
}



/*
   Verify that the sensor can be accessed over the I2C bus from FX3.
 */

#define REG_CHIP_ID_HIGH                0x300a
#define REG_CHIP_ID_LOW                 0x300b

uint8_t SensorI2cBusTest(void) {
	uint16_t id;
	uint8_t * buf =(uint8_t *) &id;

	/* Reading sensor ID */
	if (SensorRead1B(SENSOR_ADDR_RD, REG_CHIP_ID_LOW, &buf[0]) == CY_U3P_SUCCESS
	&&  SensorRead1B(SENSOR_ADDR_RD, REG_CHIP_ID_HIGH, &buf[1]) == CY_U3P_SUCCESS) {

		CyU3PDebugPrint(4, "SensorI2cBusTest ID = %x \r\n", (int)id);
		return CY_U3P_SUCCESS;
	}
	return 1;
}


/* Function to set the image sensor in VGA streaming mode. */
void SensorScaling_VGA(void) {
/*	Populate particular sensor control commands that will setup the image sensor to stream
	640 * 480 at 15 FPS in this function.
 */
    return;
}

#define REG_WINDOW_START_X_HIGH         0x3800
#define REG_WINDOW_START_X_LOW          0x3801
#define REG_WINDOW_START_Y_HIGH         0x3802
#define REG_WINDOW_START_Y_LOW          0x3803
#define REG_WINDOW_WIDTH_HIGH           0x3804
#define REG_WINDOW_WIDTH_LOW            0x3805
#define REG_WINDOW_HEIGHT_HIGH          0x3806
#define REG_WINDOW_HEIGHT_LOW           0x3807
#define REG_OUT_WIDTH_HIGH              0x3808
#define REG_OUT_WIDTH_LOW               0x3809
#define REG_OUT_HEIGHT_HIGH             0x380a
#define REG_OUT_HEIGHT_LOW              0x380b
#define REG_OUT_TOTAL_WIDTH_HIGH        0x380c
#define REG_OUT_TOTAL_WIDTH_LOW         0x380d
#define REG_OUT_TOTAL_HEIGHT_HIGH       0x380e
#define REG_OUT_TOTAL_HEIGHT_LOW        0x380f
#define OV5642_WIDTH            1280
#define OV5642_HEIGHT           720
#define OV5642_TOTAL_WIDTH      3200
#define OV5642_TOTAL_HEIGHT     2000
#define OV5642_SENSOR_SIZE_X    2592
#define OV5642_SENSOR_SIZE_Y    1944

void setResolution(int width, int height)
{
	CyU3PDebugPrint(4, "setResolution: %dx%d !\r\n", width, height);
	SensorWrite1B(SENSOR_ADDR_WR, REG_WINDOW_WIDTH_HIGH, width >> 8);
	SensorWrite1B(SENSOR_ADDR_WR, REG_WINDOW_WIDTH_LOW, width & 0xff);
	SensorWrite1B(SENSOR_ADDR_WR, REG_WINDOW_HEIGHT_HIGH, height >> 8);
	SensorWrite1B(SENSOR_ADDR_WR, REG_WINDOW_HEIGHT_LOW, height & 0xff);

	SensorWrite1B(SENSOR_ADDR_WR, REG_OUT_WIDTH_HIGH, width >> 8);
	SensorWrite1B(SENSOR_ADDR_WR, REG_OUT_WIDTH_LOW, width & 0xff);
	SensorWrite1B(SENSOR_ADDR_WR, REG_OUT_HEIGHT_HIGH, height >> 8);
	SensorWrite1B(SENSOR_ADDR_WR, REG_OUT_HEIGHT_LOW, height & 0xff);

}

/* Function to set the image sensor in HD720 streaming mode. */
void SensorScaling_HD720p_30fps(void) {
/*	Populate particular sensor control commands that will setup the image sensor to stream
	1280 * 720 at 30 FPS in this function.
 */

	//setResolution(1280, 720);
    return;
}

/*
 Get the current brightness setting from the image sensor.
 */
uint8_t SensorGetBrightness(void) {
	uint8_t buf[2];
/*
	SensorRead2B(SENSOR_ADDR_RD, 0x00, 0x02, buf);*/
	return (uint8_t) buf[1];
}

/*
 Update the brightness setting for the image sensor.
 */
void SensorSetBrightness(uint8_t brightness) {
	//SensorWrite2B(SENSOR_ADDR_WR, 0x00, 0x02, 0x00, brightness);
}
