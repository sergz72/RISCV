#ifndef I2C_FUNC_H
#define I2C_FUNC_H

#include <ch32v30x_i2c.h>

int i2c_write(I2C_TypeDef *instance, unsigned char address, const unsigned char *data, unsigned int length, unsigned int timeout);

#endif
