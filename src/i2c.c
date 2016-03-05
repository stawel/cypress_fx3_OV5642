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

#include "i2c.h"

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


CyU3PReturnStatus_t SensorConfig(uint8_t slaveAddr, struct addrval_list * config)
{
	int i = 0;
	while(config[i].addr != 0xffff) {
		if(SensorWrite1B(slaveAddr, config[i].addr, config[i].value) != CY_U3P_SUCCESS) {
			CyU3PDebugPrint(4, "Error: SensorConfig: [%d] %x := %x !\r\n", i, config[i].addr, config[i].value);
			return 1;
		}
		i++;
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
