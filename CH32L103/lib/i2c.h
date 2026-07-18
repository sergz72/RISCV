#ifndef _I2C_H
#define _I2C_H

void I2CInit(void);
int i2c_read(unsigned char address, unsigned char *data, unsigned int l, unsigned int timeout);
int i2c_write(unsigned char address, unsigned char *data, unsigned int l, unsigned int timeout, bool stop);

#endif
