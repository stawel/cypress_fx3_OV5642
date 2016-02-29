/*
 ## Cypress FX3 Camera Kit header file (sensor.h)
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

/* This file defines the parameters and the interface for the image sensor driver. */

#ifndef _INCLUDED_SENSOR_H_
#define _INCLUDED_SENSOR_H_

#include <cyu3types.h>

/* I2C Slave address for the image sensor. */
#define SENSOR_ADDR_WR 0x78             /* Slave address used to write sensor registers. Default set to EEPROM. */
#define SENSOR_ADDR_RD 0x79             /* Slave address used to read from sensor registers. Default set to EEPROM */

#define I2C_SLAVEADDR_MASK 0xFE         /* Mask to get actual I2C slave address value without direction bit. */

//#define I2C_MEMORY_ADDR_WR 0xA0         /* I2C slave address used to write to an EEPROM. */
//#define I2C_MEMORY_ADDR_RD 0xA1         /* I2C slave address used to read from an EEPROM. */

/* GPIO 22 on FX3 is used to reset the Image sensor. */
#define SENSOR_RESET_GPIO 22
#define SENSOR_XCLK_GPIO 50

/* Function    : SensorWrite2B
   Description : Write two bytes of data to image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address for the sensor.
                 highAddr  - High byte of memory address being written to.
                 lowAddr   - Low byte of memory address being written to.
                 highData  - High byte of data to be written.
                 lowData   - Low byte of data to be written.
 */


extern CyU3PReturnStatus_t SensorWrite1B(uint8_t slaveAddr, uint16_t addr, uint8_t data);
extern CyU3PReturnStatus_t SensorRead1B(uint8_t slaveAddr, uint16_t addr, uint8_t *buf);


extern CyU3PReturnStatus_t
SensorWrite2B (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t highData,
        uint8_t lowData);

/* Function    : SensorWrite
   Description : Write arbitrary amount of data to image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address for the sensor.
                 highAddr  - High byte of memory address being written to.
                 lowAddr   - Low byte of memory address being written to.
                 count     - Size of write data in bytes. Limited to a maximum of 64 bytes.
                 buf       - Pointer to buffer containing data.
 */
extern CyU3PReturnStatus_t
SensorWrite (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t count,
        uint8_t *buf);

/* Function    : SensorRead2B
   Description : Read 2 bytes of data from image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address for the sensor.
                 highAddr  - High byte of memory address being read from.
                 lowAddr   - Low byte of memory address being read from.
                 buf       - Buffer to be filled with data. MSB goes in byte 0.
 */
extern CyU3PReturnStatus_t
SensorRead2B (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t *buf);

/* Function    : SensorRead
   Description : Read arbitrary amount of data from image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address for the sensor.
                 highAddr  - High byte of memory address being read from.
                 lowAddr   - Low byte of memory address being read from.
                 count     = Size of data to be read in bytes. Limited to a max of 64.
                 buf       - Buffer to be filled with data.
 */
extern CyU3PReturnStatus_t
SensorRead (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t count,
        uint8_t *buf);

/* Function    : SensorInit
   Description : Initialize the image sensor.
   Parameters  : None
 */
extern void
SensorInit (
        void);

/* Function    : SensorReset
   Description : Reset the image sensor using FX3 GPIO.
   Parameters  : None
 */
extern void
SensorReset (
        void);

/* Function     : SensorScaling_HD720p_30fps
   Description  : Configure the image sensor for 720p 30 fps video stream.
   Parameters   : None
 */
extern void
SensorScaling_HD720p_30fps (
        void);

/* Function     : SensorScaling_VGA
   Description  : Configure the image sensor for VGA video stream.
   Parameters   : None
 */
extern void
SensorScaling_VGA (
        void);

/* Function    : SensorI2cBusTest
   Description : Test whether the image sensor is connected on the I2C bus.
   Parameters  : None
 */
extern uint8_t
SensorI2cBusTest (
        void);

/* Function    : SensorGetBrightness
   Description : Get the current brightness setting from the image sensor.
   Parameters  : None
 */
extern uint8_t
SensorGetBrightness (
        void);

/* Function    : SensorSetBrightness
   Description : Set the desired brightness setting on the image sensor.
   Parameters  :
                 brightness - Desired brightness level.
 */
extern void
SensorSetBrightness (
        uint8_t brightness);

#endif /* _INCLUDED_SENSOR_H_ */

/*[]*/

