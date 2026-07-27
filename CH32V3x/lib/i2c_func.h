#ifndef I2C_FUNC_H
#define I2C_FUNC_H

#include <ch32v30x_i2c.h>

typedef union
{
  unsigned int uint;
  unsigned char bytes[4];
} uint_to_bytes;

int i2c_write(I2C_TypeDef *instance, unsigned char address, const unsigned char *data, unsigned int length, unsigned int timeout);
int i2c_check(I2C_TypeDef *instance, unsigned char address, unsigned int timeout);
int i2c_memory_write_page(I2C_TypeDef *instance, unsigned char i2c_address, unsigned int memory_address,
                      unsigned int memory_address_length, const unsigned char *data, unsigned int length,
                      unsigned int timeout);
int i2c_memory_write_pages(I2C_TypeDef *instance, unsigned char i2c_address, unsigned int memory_address,
                      unsigned int memory_address_length, unsigned int page_size,
                      const unsigned char *data, unsigned int length, unsigned int timeout);
int i2c_memory_read(I2C_TypeDef *instance, unsigned char i2c_address, unsigned int memory_address,
                      unsigned int memory_address_length, unsigned char *data, unsigned int length,
                      unsigned int timeout);
void build_address(unsigned char *bytes, unsigned int address_length);

#endif
