#ifndef __I2CCMDS_H
#define __I2CCMDS_H

#include <stdint.h> // uint8_t

struct i2cdev_s *i2cdev_oid_lookup(uint8_t oid);
void i2cdev_write(struct i2cdev_s *i2c, uint8_t data_len, uint8_t* data);
void i2cdev_read(struct i2cdev_s *i2c, uint8_t reg_len, uint8_t* reg,
                 uint8_t data_len, uint8_t* data);

#endif // i2ccmds.h