#include "board.h"
#include <i2c.h>

void I2CInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitTSturcture;

  RCC_PB1PeriphClockCmd( I2C_CLOCK, ENABLE );

  I2C_REMAP;

  GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init( I2C_PORT, &GPIO_InitStructure );

  I2C_InitTSturcture.I2C_ClockSpeed = I2C_SPEED;
  I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
  I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
  I2C_InitTSturcture.I2C_OwnAddress1 = 0;
  I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
  I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init( I2C_INST, &I2C_InitTSturcture );

  I2C_Cmd( I2C_INST, ENABLE );
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
  I2C_Send7bitAddress( I2C_INST, address << 1, I2C_Direction_Receiver );
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
  I2C_Send7bitAddress( I2C_INST, address << 1, I2C_Direction_Transmitter );
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
