#include "board.h"
#include <veml7700.h>
#include <tsl2591.h>
#include <delay.h>
#include <string.h>

static void ports_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

  RCC->APB2PCENR |= RCC_IOPCEN;

  POWER_ON;
  GPIO_InitStructure.GPIO_Pin = POWER_ON_PIN;
  GPIO_Init(POWER_ON_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = BUTTON_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(BUTTON_PORT, &GPIO_InitStructure);
}

/*
 * I2C1 SCL = PC2(6)
 * I2C1 SDA = PC1(5)
 */
static void i2c_master_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitTSturcture;

  RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( GPIOC, &GPIO_InitStructure );

  I2C_InitTSturcture.I2C_ClockSpeed = I2C_SPEED;
  I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
  I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
  I2C_InitTSturcture.I2C_OwnAddress1 = 0;
  I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
  I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init( I2C_INST, &I2C_InitTSturcture );

  I2C_Cmd( I2C_INST, ENABLE );
}

void SysInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  RCC->CFGR0 = (RCC->CFGR0 & 0xFFFFFF0F) | CLOCK_DIVIDER;
  SystemCoreClock = 4000000;
  RCC->APB2PCENR |= RCC_AFIOEN;
  Delay_Init();
  ports_init();
  i2c_master_init();
}

int i2c_read(unsigned char address, unsigned char *data, unsigned int l, unsigned int timeout)
{
  unsigned int t;

  I2C_INST->CTLR1 |= I2C_CTLR1_START;

  t = timeout;
  while( !I2C_CheckEvent( I2C_INST, I2C_EVENT_MASTER_MODE_SELECT ) )
  {
    t--;
    if (!t)
    {
      I2C_GenerateSTOP( I2C_INST, ENABLE );
      return 1;
    }
  }
  I2C_Send7bitAddress( I2C_INST, address, I2C_Direction_Receiver );
  t = timeout;
  while( !I2C_CheckEvent( I2C_INST, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) )
  {
    t--;
    if (!t)
    {
      I2C_GenerateSTOP( I2C_INST, ENABLE );
      return 2;
    }
  }
  while (l--)
  {
    t = timeout;
    while ( !I2C_GetFlagStatus( I2C_INST, I2C_FLAG_RXNE ) )
    {
      //todo I2C_AcknowledgeConfig( I2C_INST, DISABLE );
      t--;
      if (!t)
      {
        I2C_GenerateSTOP( I2C_INST, ENABLE );
        return 3;
      }
    }
    *data++ = I2C_ReceiveData(I2C_INST);
  }
  I2C_GenerateSTOP( I2C_INST, ENABLE );
  return 0;
}

int i2c_write(unsigned char address, unsigned char *data, unsigned int l, unsigned int timeout, bool stop)
{
  unsigned int t;

  I2C_INST->CTLR1 |= I2C_CTLR1_START;

  t = timeout;
  while( !I2C_CheckEvent( I2C_INST, I2C_EVENT_MASTER_MODE_SELECT ) )
  {
    t--;
    if (!t)
    {
      I2C_GenerateSTOP( I2C_INST, ENABLE );
      return 4;
    }
  }
  I2C_Send7bitAddress( I2C_INST, address, I2C_Direction_Transmitter );
  t = timeout;
  while( !I2C_CheckEvent( I2C_INST, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) )
  {
    t--;
    if (!t)
    {
      I2C_GenerateSTOP( I2C_INST, ENABLE );
      return 5;
    }
  }
  while (l--)
  {
    t = timeout;
    while ( !I2C_GetFlagStatus( I2C_INST, I2C_FLAG_TXE ) )
    {
      t--;
      if (!t)
      {
        I2C_GenerateSTOP( I2C_INST, ENABLE );
        return 6;
      }
    }
    I2C_SendData( I2C_INST, *data++);
  }
  t = timeout;
  while( !I2C_CheckEvent( I2C_INST, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) )
  {
    t--;
    if (!t)
    {
      I2C_GenerateSTOP( I2C_INST, ENABLE );
      return 7;
    }
  }
  if (stop)
    I2C_GenerateSTOP( I2C_INST, ENABLE );
  return 0;
}


int SSD1306_I2C_Write(int num_bytes, unsigned char control_byte, unsigned char *buffer)
{
  static unsigned char i2c_buffer[256];

  i2c_buffer[0] = control_byte;
  memcpy(i2c_buffer + 1, buffer, num_bytes);
  return i2c_write(SSD1306_I2C_ADDRESS, i2c_buffer, num_bytes + 1, I2C_TIMEOUT, true);
}

#ifdef SENSOR_VEML7700
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
#endif

#ifdef SENSOR_TSL2591
int tsl2591_read8(unsigned char reg, unsigned char *data)
{
  int rc = i2c_write(TSL2591_I2C_ADDRESS << 1, &reg, 1, I2C_TIMEOUT, false);
  if (rc)
    return rc;
  return i2c_read(TSL2591_I2C_ADDRESS << 1, data, 1, I2C_TIMEOUT);
}

int tsl2591_read16(unsigned char reg, unsigned short *data)
{
  int rc = i2c_write(TSL2591_I2C_ADDRESS << 1, &reg, 1, I2C_TIMEOUT, false);
  if (rc)
    return rc;
  return i2c_read(TSL2591_I2C_ADDRESS << 1, (unsigned char*)data, 2, I2C_TIMEOUT);
}

int tsl2591_write(unsigned char reg, unsigned char value)
{
  unsigned char data[2] = {reg, value};
  return i2c_write(TSL2591_I2C_ADDRESS << 1, data, 2, I2C_TIMEOUT, true);
}
#endif

void delayms(int ms)
{
  Delay_Ms(ms);
}
