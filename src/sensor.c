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



static struct addrval_list ov5642_init2[] = {
	{0x3103 ,0x93},
	{0x3008 ,0x82},
	{0x3017 ,0x7f},
	{0x3018 ,0xfc},
	{0x3810 ,0xc2},
	{0x3615 ,0xf0},
	{0x3000 ,0x00},
	{0x3001 ,0x00},
	{0x3002 ,0x5c},
	{0x3003 ,0x00},
	{0x3004 ,0xff},
	{0x3005 ,0xff},
	{0x3006 ,0x43},
	{0x3007 ,0x37},
	{0x3011 ,0x08},
	{0x3010 ,0x10},
	{0x460c ,0x22},
	{0x3815 ,0x04},
	{0x370c ,0xa0},
	{0x3602 ,0xfc},
	{0x3612 ,0xff},
	{0x3634 ,0xc0},
	{0x3613 ,0x00},
	{0x3605 ,0x7c},
	{0x3621 ,0x09},
	{0x3622 ,0x60},
	{0x3604 ,0x40},
	{0x3603 ,0xa7},
	{0x3603 ,0x27},
	{0x4000 ,0x21},
	{0x401d ,0x22},
	{0x3600 ,0x54},
	{0x3605 ,0x04},
	{0x3606 ,0x3f},
	{0x3c01 ,0x80},
	{0x5000 ,0x4f},
	{0x5020 ,0x04},
	{0x5181 ,0x79},
	{0x5182 ,0x00},
	{0x5185 ,0x22},
	{0x5197 ,0x01},
	{0x5001 ,0xff},
	{0x5500 ,0x0a},
	{0x5504 ,0x00},
	{0x5505 ,0x7f},
	{0x5080 ,0x08},
	{0x300e ,0x18},
	{0x4610 ,0x00},

//	{0x471d ,0x05}, VSYNC stawel
	{0x471d ,0x04},
	{0x4708 ,0x06},
	{0x3808 ,0x02},
	{0x3809 ,0x80},
	{0x380a ,0x01},
	{0x380b ,0xe0},
	{0x380e ,0x07},
	{0x380f ,0xd0},
	{0x501f ,0x00},
	{0x5000 ,0x4f},
	{0x4300 ,0x30},
	{0x3503 ,0x07},
	{0x3501 ,0x73},
	{0x3502 ,0x80},
	{0x350b ,0x00},
	{0x3503 ,0x07},
	{0x3824 ,0x11},
	{0x3501 ,0x1e},
	{0x3502 ,0x80},
	{0x350b ,0x7f},
	{0x380c ,0x0c},
	{0x380d ,0x80},
	{0x380e ,0x03},
	{0x380f ,0xe8},
	{0x3a0d ,0x04},
	{0x3a0e ,0x03},

	{0x3818 ,0xc1},// stawel Compression
//	{0x3818 ,0x80},

//	{0x471b, 0x3}, // HSYNC
//	{0x4711, 0x10},
//	{0x4712, 0x10},
	{0x4741, 0x4}, //test pattern

	{0x4713, 0x03},

	{0x3705 ,0xdb},
	{0x370a ,0x81},
	{0x3801 ,0x80},
	{0x3621 ,0x87},
	{0x3801 ,0x50},
	{0x3803 ,0x08},
	{0x3827 ,0x08},
	{0x3810 ,0x40},
	{0x3804 ,0x05},
	{0x3805 ,0x00},
	{0x5682 ,0x05},
	{0x5683 ,0x00},
	{0x3806 ,0x03},
	{0x3807 ,0xc0},
	{0x5686 ,0x03},
	{0x5687 ,0xbc},
	{0x3a00 ,0x78},
	{0x3a1a ,0x05},
	{0x3a13 ,0x30},
	{0x3a18 ,0x00},
	{0x3a19 ,0x7c},
	{0x3a08 ,0x12},
	{0x3a09 ,0xc0},
	{0x3a0a ,0x0f},
	{0x3a0b ,0xa0},
	{0x350c ,0x07},
	{0x350d ,0xd0},
	{0x3500 ,0x00},
	{0x3501 ,0x00},
	{0x3502 ,0x00},
	{0x350a ,0x00},
	{0x350b ,0x00},
	{0x3503 ,0x00},
	{0x528a ,0x02},
	{0x528b ,0x04},
	{0x528c ,0x08},
	{0x528d ,0x08},
	{0x528e ,0x08},
	{0x528f ,0x10},
	{0x5290 ,0x10},
	{0x5292 ,0x00},
	{0x5293 ,0x02},
	{0x5294 ,0x00},
	{0x5295 ,0x02},
	{0x5296 ,0x00},
	{0x5297 ,0x02},
	{0x5298 ,0x00},
	{0x5299 ,0x02},
	{0x529a ,0x00},
	{0x529b ,0x02},
	{0x529c ,0x00},
	{0x529d ,0x02},
	{0x529e ,0x00},
	{0x529f ,0x02},
	{0x3030 ,0x2b},
	{0x3a02 ,0x00},
	{0x3a03 ,0x7d},
	{0x3a04 ,0x00},
	{0x3a14 ,0x00},
	{0x3a15 ,0x7d},
	{0x3a16 ,0x00},
	{0x3a00 ,0x78},
	{0x3a08 ,0x09},
	{0x3a09 ,0x60},
	{0x3a0a ,0x07},
	{0x3a0b ,0xd0},
	{0x3a0d ,0x08},
	{0x3a0e ,0x06},
	{0x5193 ,0x70},
	{0x589b ,0x04},
	{0x589a ,0xc5},
	{0x401e ,0x20},
	{0x4001 ,0x42},
	{0x401c ,0x04},
	{0x528a ,0x01},
	{0x528b ,0x04},
	{0x528c ,0x08},
	{0x528d ,0x10},
	{0x528e ,0x20},
	{0x528f ,0x28},
	{0x5290 ,0x30},
	{0x5292 ,0x00},
	{0x5293 ,0x01},
	{0x5294 ,0x00},
	{0x5295 ,0x04},
	{0x5296 ,0x00},
	{0x5297 ,0x08},
	{0x5298 ,0x00},
	{0x5299 ,0x10},
	{0x529a ,0x00},
	{0x529b ,0x20},
	{0x529c ,0x00},
	{0x529d ,0x28},
	{0x529e ,0x00},
	{0x529f ,0x30},
	{0x5282 ,0x00},
	{0x5300 ,0x00},
	{0x5301 ,0x20},
	{0x5302 ,0x00},
	{0x5303 ,0x7c},
	{0x530c ,0x00},
	{0x530d ,0x0c},
	{0x530e ,0x20},
	{0x530f ,0x80},
	{0x5310 ,0x20},
	{0x5311 ,0x80},
	{0x5308 ,0x20},
	{0x5309 ,0x40},
	{0x5304 ,0x00},
	{0x5305 ,0x30},
	{0x5306 ,0x00},
	{0x5307 ,0x80},
	{0x5314 ,0x08},
	{0x5315 ,0x20},
	{0x5319 ,0x30},
	{0x5316 ,0x10},
	{0x5317 ,0x00},
	{0x5318 ,0x02},
	{0x5402 ,0x3f},
	{0x5403 ,0x00},
	{0x3406 ,0x00},
	{0x5180 ,0xff},
	{0x5181 ,0x52},
	{0x5182 ,0x11},
	{0x5183 ,0x14},
	{0x5184 ,0x25},
	{0x5185 ,0x24},
	{0x5186 ,0x06},
	{0x5187 ,0x08},
	{0x5188 ,0x08},
	{0x5189 ,0x7c},
	{0x518a ,0x60},
	{0x518b ,0xb2},
	{0x518c ,0xb2},
	{0x518d ,0x44},
	{0x518e ,0x3d},
	{0x518f ,0x58},
	{0x5190 ,0x46},
	{0x5191 ,0xf8},
	{0x5192 ,0x04},
	{0x5193 ,0x70},
	{0x5194 ,0xf0},
	{0x5195 ,0xf0},
	{0x5196 ,0x03},
	{0x5197 ,0x01},
	{0x5198 ,0x04},
	{0x5199 ,0x12},
	{0x519a ,0x04},
	{0x519b ,0x00},
	{0x519c ,0x06},
	{0x519d ,0x82},
	{0x519e ,0x00},
	{0x5025 ,0x80},
	{0x5583 ,0x40},
	{0x5584 ,0x40},
	{0x5580 ,0x02},
	{0x5000 ,0xcf},
	{0x3710 ,0x10},
	{0x3632 ,0x51},
	{0x3702 ,0x10},
	{0x3703 ,0xb2},
	{0x3704 ,0x18},
	{0x370b ,0x40},
	{0x370d ,0x03},
	{0x3631 ,0x01},
	{0x3632 ,0x52},
	{0x3606 ,0x24},
	{0x3620 ,0x96},
	{0x5785 ,0x07},
	{0x3a13 ,0x30},
	{0x3600 ,0x52},
	{0x3604 ,0x48},
	{0x3606 ,0x1b},
	{0x370d ,0x0b},
	{0x370f ,0xc0},
	{0x3709 ,0x01},
	{0x3823 ,0x00},
	{0x5007 ,0x00},
	{0x5009 ,0x00},
	{0x5011 ,0x00},
	{0x5013 ,0x00},
	{0x519e ,0x00},
	{0x5086 ,0x00},
	{0x5087 ,0x00},
	{0x5088 ,0x00},
	{0x5089 ,0x00},
	{0x302b ,0x00},
	{0x3808 ,0x01},
	{0x3809 ,0x40},
	{0x380a ,0x00},
	{0x380b ,0xf0},
	{0x3a00 ,0x78},
	{0x5001 ,0xFF},
	{0x5583 ,0x50},
	{0x5584 ,0x50},
	{0x5580 ,0x02},
	{0x3c01 ,0x80},
	{0x3c00 ,0x04},
	//;LENS
	{0x5800 ,0x48},
	{0x5801 ,0x31},
	{0x5802 ,0x21},
	{0x5803 ,0x1b},
	{0x5804 ,0x1a},
	{0x5805 ,0x1e},
	{0x5806 ,0x29},
	{0x5807 ,0x38},
	{0x5808 ,0x26},
	{0x5809 ,0x17},
	{0x580a ,0x11},
	{0x580b ,0xe },
	{0x580c ,0xd },
	{0x580d ,0xe },
	{0x580e ,0x13},
	{0x580f ,0x1a},
	{0x5810 ,0x15},
	{0x5811 ,0xd },
	{0x5812 ,0x8 },
	{0x5813 ,0x5 },
	{0x5814 ,0x4 },
	{0x5815 ,0x5 },
	{0x5816 ,0x9 },
	{0x5817 ,0xd },
	{0x5818 ,0x11},
	{0x5819 ,0xa },
	{0x581a ,0x4 },
	{0x581b ,0x0 },
	{0x581c ,0x0 },
	{0x581d ,0x1 },
	{0x581e ,0x6 },
	{0x581f ,0x9 },
	{0x5820 ,0x12},
	{0x5821 ,0xb },
	{0x5822 ,0x4 },
	{0x5823 ,0x0 },
	{0x5824 ,0x0 },
	{0x5825 ,0x1 },
	{0x5826 ,0x6 },
	{0x5827 ,0xa },
	{0x5828 ,0x17},
	{0x5829 ,0xf },
	{0x582a ,0x9 },
	{0x582b ,0x6 },
	{0x582c ,0x5 },
	{0x582d ,0x6 },
	{0x582e ,0xa },
	{0x582f ,0xe },
	{0x5830 ,0x28},
	{0x5831 ,0x1a},
	{0x5832 ,0x11},
	{0x5833 ,0xe },
	{0x5834 ,0xe },
	{0x5835 ,0xf },
	{0x5836 ,0x15},
	{0x5837 ,0x1d},
	{0x5838 ,0x6e},
	{0x5839 ,0x39},
	{0x583a ,0x27},
	{0x583b ,0x1f},
	{0x583c ,0x1e},
	{0x583d ,0x23},
	{0x583e ,0x2f},
	{0x583f ,0x41},
	{0x5840 ,0xe },
	{0x5841 ,0xc },
	{0x5842 ,0xd },
	{0x5843 ,0xc },
	{0x5844 ,0xc },
	{0x5845 ,0xc },
	{0x5846 ,0xc },
	{0x5847 ,0xc },
	{0x5848 ,0xd },
	{0x5849 ,0xe },
	{0x584a ,0xe },
	{0x584b ,0xa },
	{0x584c ,0xe },
	{0x584d ,0xe },
	{0x584e ,0x10},
	{0x584f ,0x10},
	{0x5850 ,0x11},
	{0x5851 ,0xa },
	{0x5852 ,0xf },
	{0x5853 ,0xe },
	{0x5854 ,0x10},
	{0x5855 ,0x10},
	{0x5856 ,0x10},
	{0x5857 ,0xa },
	{0x5858 ,0xe },
	{0x5859 ,0xe },
	{0x585a ,0xf },
	{0x585b ,0xf },
	{0x585c ,0xf },
	{0x585d ,0xa },
	{0x585e ,0x9 },
	{0x585f ,0xd },
	{0x5860 ,0xc },
	{0x5861 ,0xb },
	{0x5862 ,0xd },
	{0x5863 ,0x7 },
	{0x5864 ,0x17},
	{0x5865 ,0x14},
	{0x5866 ,0x18},
	{0x5867 ,0x18},
	{0x5868 ,0x16},
	{0x5869 ,0x12},
	{0x586a ,0x1b},
	{0x586b ,0x1a},
	{0x586c ,0x16},
	{0x586d ,0x16},
	{0x586e ,0x18},
	{0x586f ,0x1f},
	{0x5870 ,0x1c},
	{0x5871 ,0x16},
	{0x5872 ,0x10},
	{0x5873 ,0xf },
	{0x5874 ,0x13},
	{0x5875 ,0x1c},
	{0x5876 ,0x1e},
	{0x5877 ,0x17},
	{0x5878 ,0x11},
	{0x5879 ,0x11},
	{0x587a ,0x14},
	{0x587b ,0x1e},
	{0x587c ,0x1c},
	{0x587d ,0x1c},
	{0x587e ,0x1a},
	{0x587f ,0x1a},
	{0x5880 ,0x1b},
	{0x5881 ,0x1f},
	{0x5882 ,0x14},
	{0x5883 ,0x1a},
	{0x5884 ,0x1d},
	{0x5885 ,0x1e},
	{0x5886 ,0x1a},
	{0x5887 ,0x1a},
	//;AWB
	{0x5180 ,0xff},
	{0x5181 ,0x52},
	{0x5182 ,0x11},
	{0x5183 ,0x14},
	{0x5184 ,0x25},
	{0x5185 ,0x24},
	{0x5186 ,0x14},
	{0x5187 ,0x14},
	{0x5188 ,0x14},
	{0x5189 ,0x69},
	{0x518a ,0x60},
	{0x518b ,0xa2},
	{0x518c ,0x9c},
	{0x518d ,0x36},
	{0x518e ,0x34},
	{0x518f ,0x54},
	{0x5190 ,0x4c},
	{0x5191 ,0xf8},
	{0x5192 ,0x04},
	{0x5193 ,0x70},
	{0x5194 ,0xf0},
	{0x5195 ,0xf0},
	{0x5196 ,0x03},
	{0x5197 ,0x01},
	{0x5198 ,0x05},
	{0x5199 ,0x2f},
	{0x519a ,0x04},
	{0x519b ,0x00},
	{0x519c ,0x06},
	{0x519d ,0xa0},
	{0x519e ,0xa0},
	//;D/S
	{0x528a ,0x00},
	{0x528b ,0x01},
	{0x528c ,0x04},
	{0x528d ,0x08},
	{0x528e ,0x10},
	{0x528f ,0x20},
	{0x5290 ,0x30},
	{0x5292 ,0x00},
	{0x5293 ,0x00},
	{0x5294 ,0x00},
	{0x5295 ,0x01},
	{0x5296 ,0x00},
	{0x5297 ,0x04},
	{0x5298 ,0x00},
	{0x5299 ,0x08},
	{0x529a ,0x00},
	{0x529b ,0x10},
	{0x529c ,0x00},
	{0x529d ,0x20},
	{0x529e ,0x00},
	{0x529f ,0x30},
	{0x5282 ,0x00},
	{0x5300 ,0x00},
	{0x5301 ,0x20},
	{0x5302 ,0x00},
	{0x5303 ,0x7c},
	{0x530c ,0x00},
	{0x530d ,0x10},
	{0x530e ,0x20},
	{0x530f ,0x80},
	{0x5310 ,0x20},
	{0x5311 ,0x80},
	{0x5308 ,0x20},
	{0x5309 ,0x40},
	{0x5304 ,0x00},
	{0x5305 ,0x30},
	{0x5306 ,0x00},
	{0x5307 ,0x80},
	{0x5314 ,0x08},
	{0x5315 ,0x20},
	{0x5319 ,0x30},
	{0x5316 ,0x10},
	{0x5317 ,0x00},
	{0x5318 ,0x02},
	//;CMX
	{0x5380 ,0x01},
	{0x5381 ,0x00},
	{0x5382 ,0x00},
	{0x5383 ,0x1f},
	{0x5384 ,0x00},
	{0x5385 ,0x06},
	{0x5386 ,0x00},
	{0x5387 ,0x00},
	{0x5388 ,0x00},
	{0x5389 ,0xE1},
	{0x538A ,0x00},
	{0x538B ,0x2B},
	{0x538C ,0x00},
	{0x538D ,0x00},
	{0x538E ,0x00},
	{0x538F ,0x10},
	{0x5390 ,0x00},
	{0x5391 ,0xB3},
	{0x5392 ,0x00},
	{0x5393 ,0xA6},
	{0x5394 ,0x08},
	//;GAMMA
	{0x5480 ,0x0c},
	{0x5481 ,0x18},
	{0x5482 ,0x2f},
	{0x5483 ,0x55},
	{0x5484 ,0x64},
	{0x5485 ,0x71},
	{0x5486 ,0x7d},
	{0x5487 ,0x87},
	{0x5488 ,0x91},
	{0x5489 ,0x9a},
	{0x548A ,0xaa},
	{0x548B ,0xb8},
	{0x548C ,0xcd},
	{0x548D ,0xdd},
	{0x548E ,0xea},
	{0x548F ,0x1d},
	{0x5490 ,0x05},
	{0x5491 ,0x00},
	{0x5492 ,0x04},
	{0x5493 ,0x20},
	{0x5494 ,0x03},
	{0x5495 ,0x60},
	{0x5496 ,0x02},
	{0x5497 ,0xB8},
	{0x5498 ,0x02},
	{0x5499 ,0x86},
	{0x549A ,0x02},
	{0x549B ,0x5B},
	{0x549C ,0x02},
	{0x549D ,0x3B},
	{0x549E ,0x02},
	{0x549F ,0x1C},
	{0x54A0 ,0x02},
	{0x54A1 ,0x04},
	{0x54A2 ,0x01},
	{0x54A3 ,0xED},
	{0x54A4 ,0x01},
	{0x54A5 ,0xC5},
	{0x54A6 ,0x01},
	{0x54A7 ,0xA5},
	{0x54A8 ,0x01},
	{0x54A9 ,0x6C},
	{0x54AA ,0x01},
	{0x54AB ,0x41},
	{0x54AC ,0x01},
	{0x54AD ,0x20},
	{0x54AE ,0x00},
	{0x54AF ,0x16},
	{0x54B0 ,0x01},
	{0x54B1 ,0x20},
	{0x54B2 ,0x00},
	{0x54B3 ,0x10},
	{0x54B4 ,0x00},
	{0x54B5 ,0xf0},
	{0x54B6 ,0x00},
	{0x54B7 ,0xDF},
	{0x5402 ,0x3f},
	{0x5403 ,0x00},
	//;UV ADJUST
	{0x5500 ,0x10},
	{0x5502 ,0x00},
	{0x5503 ,0x06},
	{0x5504 ,0x00},
	{0x5505 ,0x7f},
	//;AE
	{0x5025 ,0x80},
	{0x3a0f ,0x30},
	{0x3a10 ,0x28},
	{0x3a1b ,0x30},
	{0x3a1e ,0x28},
	{0x3a11 ,0x61},
	{0x3a1f ,0x10},
	{0x5688 ,0xfd},
	{0x5689 ,0xdf},
	{0x568a ,0xfe},
	{0x568b ,0xef},
	{0x568c ,0xfe},
	{0x568d ,0xef},
	{0x568e ,0xaa},
	{0x568f ,0xaa},

	{0x4740,0x21},
	{0x501e,0x2a},
	{0x5002,0xf8},
	{0x501f,0x01},
	{0x4300,0x61},

//	{0xffff,0xff},
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
