#include <i2c_func.h>

#include "delay.h"

static int i2c_send_address(I2C_TypeDef *instance, unsigned char address, unsigned int timeout, int transmitter_mode)
{
  unsigned int t;
  if (transmitter_mode)
  {
    t = timeout;
    while(I2C_GetFlagStatus(instance, I2C_FLAG_BUSY ) != RESET && t)
      t--;
    if (!t)
      return 1;
  }

  I2C_GenerateSTART(instance, ENABLE);
  t = timeout;
  while(!I2C_CheckEvent(instance, I2C_EVENT_MASTER_MODE_SELECT) && t)
    t--;
  if (!t)
    return 2;

  I2C_Send7bitAddress(instance, address, transmitter_mode ? I2C_Direction_Transmitter : I2C_Direction_Receiver);
  t = timeout;
  uint32_t event = transmitter_mode ? I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED : I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED;
  while(!I2C_CheckEvent(instance, event) && t)
    t--;
  if (!t)
  {
    I2C_GenerateSTOP(instance, ENABLE);
    return 3;
  }

  return 0;
}

static int i2c_send_data(I2C_TypeDef *instance, const unsigned char *data, unsigned int length, unsigned int timeout)
{
  while (length--)
  {
    unsigned int t = timeout;
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

  return 0;
}

static int i2c_receive_data(I2C_TypeDef *instance, unsigned char *data, unsigned int length, unsigned int timeout)
{
  while (length--)
  {
    if (!length)
      I2C_AcknowledgeConfig( instance, DISABLE );
    unsigned int t = timeout;
    while (!I2C_GetFlagStatus( instance, I2C_FLAG_RXNE))
    {
      t--;
      if (!t)
        return 4;
    }
    *data++ = I2C_ReceiveData(instance);
  }

  return 0;
}

int i2c_write(I2C_TypeDef *instance, unsigned char address, const unsigned char *data, unsigned int length, unsigned int timeout)
{
  int rc = i2c_send_address(instance, address, timeout, 1);
  if (rc)
    return rc;

  rc = i2c_send_data(instance, data, length, timeout);
  if (rc)
    return rc;

  I2C_GenerateSTOP(instance, ENABLE);

  return timeout ? 0 : 6;
}

int i2c_check(I2C_TypeDef *instance, unsigned char address, unsigned int timeout)
{
  int rc = i2c_send_address(instance, address, timeout, 1);
  if (rc)
    return rc;
  I2C_GenerateSTOP(instance, ENABLE);
  return 0;
}

void build_address(unsigned char *bytes, unsigned int address_length)
{
  unsigned char temp;
  switch (address_length)
  {
    case 2:
      temp = bytes[0];
      bytes[0] = bytes[1];
      bytes[1] = temp;
      break;
    case 3:
      temp = bytes[0];
      bytes[0] = bytes[2];
      bytes[2] = temp;
      break;
    case 4:
      temp = bytes[0];
      bytes[0] = bytes[3];
      bytes[3] = temp;
      temp = bytes[1];
      bytes[1] = bytes[2];
      bytes[2] = temp;
      break;
    default:
      break;
  }
}

int i2c_memory_write_page(I2C_TypeDef *instance, unsigned char i2c_address, unsigned int memory_address,
                      unsigned int memory_address_length, const unsigned char *data, unsigned int length,
                      unsigned int timeout)
{
  uint_to_bytes ub;

  ub.uint = memory_address;
  build_address(ub.bytes, memory_address_length);

  int rc = i2c_send_address(instance, i2c_address, timeout, 1);
  if (rc)
    return rc;

  rc = i2c_send_data(instance, ub.bytes, memory_address_length, timeout);
  if (rc)
    return rc;

  rc = i2c_send_data(instance, data, length, timeout);
  if (rc)
    return rc;

  I2C_GenerateSTOP(instance, ENABLE);

  return timeout ? 0 : 6;
}

int i2c_memory_write_pages(I2C_TypeDef *instance, unsigned char i2c_address, unsigned int memory_address,
                      unsigned int memory_address_length, unsigned int page_size,
                      const unsigned char *data, unsigned int length, unsigned int timeout)
{
  int first = 1;
  while (length)
  {
    unsigned int l = length > page_size ? page_size : length;
    int rc;
    for (int i = 0; i < 10; i++)
    {
      rc = i2c_memory_write_page(instance, i2c_address, memory_address, memory_address_length, data, l, timeout);
      if (rc && first)
        return rc;
      first = 0;
      if (!rc)
        break;
      Delay_Ms(1);
    }
    if (rc)
      return rc;
    length -= l;
    data += l;
    memory_address += l;
  }
  return 0;
}

int i2c_memory_read(I2C_TypeDef *instance, unsigned char i2c_address, unsigned int memory_address,
                      unsigned int memory_address_length, unsigned char *data, unsigned int length,
                      unsigned int timeout)
{
  uint_to_bytes ub;

  ub.uint = memory_address;
  build_address(ub.bytes, memory_address_length);

  I2C_AcknowledgeConfig( instance, ENABLE );

  int rc = i2c_send_address(instance, i2c_address, timeout, 1);
  if (rc)
    return rc;

  rc = i2c_send_data(instance, ub.bytes, memory_address_length, timeout);
  if (rc)
    return rc;

  rc = i2c_send_address(instance, i2c_address, timeout, 0);
  if (rc)
    return rc + 10;

  rc = i2c_receive_data(instance, data, length, timeout);
  if (rc)
    return rc + 10;

  I2C_GenerateSTOP(instance, ENABLE);

  return timeout ? 0 : 6;
}
