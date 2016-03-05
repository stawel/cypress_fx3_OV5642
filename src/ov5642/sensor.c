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
#include "../i2c.h"
#include "ov5642_conifg.h"

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

void SensorReset(void) {
	CyU3PThreadSleep(10);
	CyU3PThreadSleep(10);
	return;
}

#define sizeofArray(x) (sizeof(x)/sizeof(x[0]))

/* Image sensor initialization sequence. */
void SensorInit(void) {
	if (SensorI2cBusTest() != CY_U3P_SUCCESS) {
		CyU3PDebugPrint(4, "Error: Reading Sensor ID failed!\r\n");
		return;
	}

	SensorConfig(SENSOR_ADDR_WR, ov5642_init);
	SensorScaling_HD720p_30fps();
}

void SensorScaling_VGA(void) {
	//TODO:??
	SensorConfig(SENSOR_ADDR_WR, ov5642_vga_preview);
    return;
}

void SensorScaling_HD720p_30fps(void) {

	SensorConfig(SENSOR_ADDR_WR, ov5642_720p_preview);
}

uint8_t SensorGetBrightness(void) {
	uint8_t buf[2];
/*
	SensorRead2B(SENSOR_ADDR_RD, 0x00, 0x02, buf);*/
	return (uint8_t) buf[1];
}

void SensorSetBrightness(uint8_t brightness) {
	//SensorWrite2B(SENSOR_ADDR_WR, 0x00, 0x02, 0x00, brightness);
}
