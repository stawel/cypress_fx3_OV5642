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

/* This function inserts a delay between successful I2C transfers to prevent
 false errors due to the slave being busy.
 */
static void SensorI2CAccessDelay(CyU3PReturnStatus_t status) {
	/* Add a 10us delay if the I2C operation that preceded this call was successful. */
	if (status == CY_U3P_SUCCESS)
		CyU3PBusyWait(10);
}

/* Write to an I2C slave with two bytes of data. */
CyU3PReturnStatus_t SensorWrite2B(uint8_t slaveAddr, uint8_t highAddr,
		uint8_t lowAddr, uint8_t highData, uint8_t lowData) {
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PI2cPreamble_t preamble;
	uint8_t buf[2];

	/* Validate the I2C slave address. */
	if ((slaveAddr != SENSOR_ADDR_WR) && (slaveAddr != I2C_MEMORY_ADDR_WR)) {
		CyU3PDebugPrint(4, "I2C Slave address is not valid!\n");
		return 1;
	}

	/* Set the parameters for the I2C API access and then call the write API. */
	preamble.buffer[0] = slaveAddr;
	preamble.buffer[1] = highAddr;
	preamble.buffer[2] = lowAddr;
	preamble.length = 3; /*  Three byte preamble. */
	preamble.ctrlMask = 0x0000; /*  No additional start and stop bits. */

	buf[0] = highData;
	buf[1] = lowData;

	apiRetStatus = CyU3PI2cTransmitBytes(&preamble, buf, 2, 0);
	SensorI2CAccessDelay(apiRetStatus);

	return apiRetStatus;
}

CyU3PReturnStatus_t SensorWrite(uint8_t slaveAddr, uint8_t highAddr,
		uint8_t lowAddr, uint8_t count, uint8_t *buf) {
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PI2cPreamble_t preamble;

	/* Validate the I2C slave address. */
	if ((slaveAddr != SENSOR_ADDR_WR) && (slaveAddr != I2C_MEMORY_ADDR_WR)) {
		CyU3PDebugPrint(4, "I2C Slave address is not valid!\n");
		return 1;
	}

	if (count > 64) {
		CyU3PDebugPrint(4, "ERROR: SensorWrite count > 64\n");
		return 1;
	}

	/* Set up the I2C control parameters and invoke the write API. */
	preamble.buffer[0] = slaveAddr;
	preamble.buffer[1] = highAddr;
	preamble.buffer[2] = lowAddr;
	preamble.length = 3;
	preamble.ctrlMask = 0x0000;

	apiRetStatus = CyU3PI2cTransmitBytes(&preamble, buf, count, 0);
	SensorI2CAccessDelay(apiRetStatus);

	return apiRetStatus;
}

CyU3PReturnStatus_t SensorRead2B(uint8_t slaveAddr, uint8_t highAddr,
		uint8_t lowAddr, uint8_t *buf) {
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PI2cPreamble_t preamble;

	if ((slaveAddr != SENSOR_ADDR_RD) && (slaveAddr != I2C_MEMORY_ADDR_RD)) {
		CyU3PDebugPrint(4, "I2C Slave address is not valid!\n");
		return 1;
	}

	preamble.buffer[0] = slaveAddr & I2C_SLAVEADDR_MASK; /*  Mask out the transfer type bit. */
	preamble.buffer[1] = highAddr;
	preamble.buffer[2] = lowAddr;
	preamble.buffer[3] = slaveAddr;
	preamble.length = 4;
	preamble.ctrlMask = 0x0004; /*  Send start bit after third byte of preamble. */

	apiRetStatus = CyU3PI2cReceiveBytes(&preamble, buf, 2, 0);
	SensorI2CAccessDelay(apiRetStatus);

	return apiRetStatus;
}

CyU3PReturnStatus_t SensorRead(uint8_t slaveAddr, uint8_t highAddr,
		uint8_t lowAddr, uint8_t count, uint8_t *buf) {
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PI2cPreamble_t preamble;

	/* Validate the parameters. */
	if ((slaveAddr != SENSOR_ADDR_RD) && (slaveAddr != I2C_MEMORY_ADDR_RD)) {
		CyU3PDebugPrint(4, "I2C Slave address is not valid!\n");
		return 1;
	}
	if (count > 64) {
		CyU3PDebugPrint(4, "ERROR: SensorWrite count > 64\n");
		return 1;
	}

	preamble.buffer[0] = slaveAddr & I2C_SLAVEADDR_MASK; /*  Mask out the transfer type bit. */
	preamble.buffer[1] = highAddr;
	preamble.buffer[2] = lowAddr;
	preamble.buffer[3] = slaveAddr;
	preamble.length = 4;
	preamble.ctrlMask = 0x0004; /*  Send start bit after third byte of preamble. */

	apiRetStatus = CyU3PI2cReceiveBytes(&preamble, buf, count, 0);
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

/* Image sensor initialization sequence. */
void SensorInit(void) {
	if (SensorI2cBusTest() != CY_U3P_SUCCESS) /* Verify that the sensor is connected. */
	{
		CyU3PDebugPrint(4, "Error: Reading Sensor ID failed!\r\n");
		return;
	}

	/* Generic settings (which are common for all resolutions) for bringing up the image sensor to stream
	 video data should be populated here.
	 */

	/* Update sensor configuration based on desired video stream parameters. Using 720p 30fps as default setting.*/
	SensorScaling_HD720p_30fps();
}

/*
   Verify that the sensor can be accessed over the I2C bus from FX3.
 */
uint8_t SensorI2cBusTest(void) {
	/* The sensor ID register can be read here to verify sensor connectivity. */
	uint8_t buf[2];

	/* Reading sensor ID */
	if (SensorRead2B(SENSOR_ADDR_RD, 0x00, 0x00, buf) == CY_U3P_SUCCESS) {
		if ((buf[0] == 0x01) && (buf[1] == 0x02)) {
			return CY_U3P_SUCCESS;
		}
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

/* Function to set the image sensor in HD720 streaming mode. */
void SensorScaling_HD720p_30fps(void) {
/*	Populate particular sensor control commands that will setup the image sensor to stream
	1280 * 720 at 30 FPS in this function.
 */
    return;
}

/*
 Get the current brightness setting from the image sensor.
 */
uint8_t SensorGetBrightness(void) {
	uint8_t buf[2];

	SensorRead2B(SENSOR_ADDR_RD, 0x00, 0x02, buf);
	return (uint8_t) buf[1];
}

/*
 Update the brightness setting for the image sensor.
 */
void SensorSetBrightness(uint8_t brightness) {
	SensorWrite2B(SENSOR_ADDR_WR, 0x00, 0x02, 0x00, brightness);
}

