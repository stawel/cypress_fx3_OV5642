#ifndef _INCLUDED_OV5642_SENSOR_H_
#define _INCLUDED_OV5642_SENSOR_H_

#include <cyu3types.h>

#define SENSOR_ADDR_WR 0x78
#define SENSOR_ADDR_RD 0x79

#define SENSOR_RESET_GPIO 22
#define SENSOR_XCLK_GPIO 23

/* Function    : SensorInit
 Description : Initialize the image sensor.
 Parameters  : None
 */
extern void SensorInit(void);

/* Function    : SensorReset
 Description : Reset the image sensor using FX3 GPIO.
 Parameters  : None
 */
extern void SensorReset(void);

/* Function     : SensorScaling_HD720p_30fps
 Description  : Configure the image sensor for 720p 30 fps video stream.
 Parameters   : None
 */
extern void SensorScaling_HD720p_30fps(void);

/* Function     : SensorScaling_VGA
 Description  : Configure the image sensor for VGA video stream.
 Parameters   : None
 */
extern void SensorScaling_VGA(void);

/* Function    : SensorGetBrightness
 Description : Get the current brightness setting from the image sensor.
 Parameters  : None
 */
extern uint8_t SensorGetBrightness(void);

/* Function    : SensorSetBrightness
 Description : Set the desired brightness setting on the image sensor.
 Parameters  :
 brightness - Desired brightness level.
 */
extern void SensorSetBrightness(uint8_t brightness);

#endif /* _INCLUDED_OV5642_SENSOR_H_ */

/*[]*/

