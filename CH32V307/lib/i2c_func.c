#include <i2c_func.h>

int i2c_write(I2C_TypeDef *instance, unsigned char address, const unsigned char *data, unsigned int length, unsigned int timeout)
{
  unsigned int t = timeout;
  while(I2C_GetFlagStatus(instance, I2C_FLAG_BUSY ) != RESET && t)
    t--;
  if (!t)
    return 1;

  I2C_GenerateSTART(instance, ENABLE);
  t = timeout;
  while(!I2C_CheckEvent(instance, I2C_EVENT_MASTER_MODE_SELECT) && t)
    t--;
  if (!t)
    return 2;

  I2C_Send7bitAddress(instance, address, I2C_Direction_Transmitter);
  t = timeout;
  while(!I2C_CheckEvent(instance, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && t)
    t--;
  if (!t)
  {
    I2C_GenerateSTOP(instance, ENABLE);
    return 3;
  }

  while (length--)
  {
    t = timeout;
    while (!I2C_GetFlagStatus(instance, I2C_FLAG_TXE) && t)
      t--;
    if (!t)
    {
      I2C_GenerateSTOP(instance, ENABLE);
      return 4;
    }
    I2C_SendData(instance, *data++);

    t = timeout;
    while(!I2C_CheckEvent(instance, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && t)
      t--;
    if (!t)
    {
      I2C_GenerateSTOP(instance, ENABLE);
      return 5;
    }
  }

  I2C_GenerateSTOP(instance, ENABLE);

  return timeout ? 0 : 6;
}