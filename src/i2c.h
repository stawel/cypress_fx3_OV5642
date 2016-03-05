#ifndef _I2C_H_
#define _I2C_H_

#include <cyu3types.h>

//should end with {0xffff,0xff}
struct addrval_list {
 uint16_t addr;
 uint8_t value;
 };


#define I2C_SLAVEADDR_MASK 0xFE         /* Mask to get actual I2C slave address value without direction bit. */

extern CyU3PReturnStatus_t SensorWrite1B(uint8_t slaveAddr, uint16_t addr, uint8_t data);
extern CyU3PReturnStatus_t SensorRead1B(uint8_t slaveAddr, uint16_t addr, uint8_t *buf);

extern CyU3PReturnStatus_t SensorConfig(uint8_t slaveAddr, struct addrval_list * config);

#endif /* _I2C_H_ */


