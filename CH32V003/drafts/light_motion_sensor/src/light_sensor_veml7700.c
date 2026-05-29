#include "board.h"
#include <veml7700.h>

int light_sensor_init(void)
{
  enable_i2c();
  int rc =  veml7700_ex_init(VEML7700_GAIN_2|VEML7700_IT_100ms, VEML7700_PSM_MODE4|VEML7700_PSM_ENABLE);
  disable_i2c();
  return rc;
}

int light_sensor_read(unsigned short *result)
{
  enable_i2c();
  int rc = veml7700_read(VEML7700_REG_ALS, result);
  disable_i2c();
  return rc;
}

int veml7700_read(unsigned char reg, unsigned short *data)
{
  int rc = i2c_write(VEML7700_I2C_ADDRESS << 1, &reg, 1, I2C_TIMEOUT, false);
  if (rc)
    return rc;
  return i2c_read(VEML7700_I2C_ADDRESS << 1, (unsigned char*)data, 2, I2C_TIMEOUT);
}

int veml7700_write(unsigned char reg, unsigned short value)
{
  unsigned char data[3] = {reg, (unsigned char)value, (unsigned char)(value >> 8)};
  return i2c_write(VEML7700_I2C_ADDRESS << 1, data, 3, I2C_TIMEOUT, true);
}
