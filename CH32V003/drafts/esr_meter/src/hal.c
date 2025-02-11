#include "board.h"
#include <mcp3426.h>
#include <delay.h>
#include <string.h>

static void ports_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

  RCC->APB2PCENR |= RCC_IOPAEN | RCC_IOPCEN | RCC_IOPDEN;

  GPIO_InitStructure.GPIO_Pin = LED_TIMER_PIN;
  GPIO_Init(LED_TIMER_PORT, &GPIO_InitStructure);
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

  I2C_InitTSturcture.I2C_ClockSpeed = 100000;
  I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
  I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
  I2C_InitTSturcture.I2C_OwnAddress1 = 0;
  I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
  I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init( I2C1, &I2C_InitTSturcture );

  I2C_Cmd( I2C1, ENABLE );
}

static void pwm_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  GPIO_InitStructure.GPIO_Pin = PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(PWM_PORT, &GPIO_InitStructure);

  TIM_TimeBaseInitStructure.TIM_Period = PWM_PERIOD - 1; // 100 khz
  TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = PWM_PULSE;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
  PWM_OCINIT( TIM1, &TIM_OCInitStructure );

  TIM_CtrlPWMOutputs(TIM1, ENABLE );
  PWM_OCPRELOADCONFIG( TIM1, TIM_OCPreload_Disable );
  TIM_ARRPreloadConfig( TIM1, ENABLE );
  TIM_Cmd( TIM1, ENABLE );
}

void SysInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  RCC->APB2PCENR |= RCC_AFIOEN;
  Delay_Init();
  ports_init();
  i2c_master_init();
  pwm_init();
}

static int i2c_read(unsigned char address, unsigned char *data, unsigned int l, unsigned int timeout)
{
  unsigned int t;

  I2C1->CTLR1 |= I2C_CTLR1_START;

  t = timeout;
  while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) )
  {
    t--;
    if (!t)
      return 1;
  }
  I2C_Send7bitAddress( I2C1, address, I2C_Direction_Receiver );
  t = timeout;
  while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) )
  {
    t--;
    if (!t)
      return 2;
  }
  while (l--)
  {
    t = timeout;
    while ( !I2C_GetFlagStatus( I2C1, I2C_FLAG_RXNE ) )
    {
      //todo I2C_AcknowledgeConfig( I2C1, DISABLE );
      t--;
      if (!t)
        return 3;
    }
    *data++ = I2C_ReceiveData(I2C1);
  }
  I2C_GenerateSTOP( I2C1, ENABLE );
  return 0;
}

static int i2c_write(unsigned char address, unsigned char *data, unsigned int l, unsigned int timeout)
{
  unsigned int t;

  I2C1->CTLR1 |= I2C_CTLR1_START;

  t = timeout;
  while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) )
  {
    t--;
    if (!t)
      return 1;
  }
  I2C_Send7bitAddress( I2C1, address, I2C_Direction_Transmitter );
  t = timeout;
  while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) )
  {
    t--;
    if (!t)
      return 2;
  }
  while (l--)
  {
    t = timeout;
    while ( !I2C_GetFlagStatus( I2C1, I2C_FLAG_TXE ) )
    {
      t--;
      if (!t)
        return 3;
    }
    I2C_SendData( I2C1, *data++);
  }
  t = timeout;
  while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) )
  {
    t--;
    if (!t)
      return 4;
  }
  I2C_GenerateSTOP( I2C1, ENABLE );
  return 0;
}

int mcp3426Read(int channel, unsigned char address, unsigned char *data, unsigned int l)
{
  return i2c_read(address, data, l, I2C_TIMEOUT);
}

int mcp3426Write(int channel, unsigned char address, unsigned char data)
{
  return i2c_write(address, &data, 1, I2C_TIMEOUT);
}

int SSD1306_I2C_Write(int num_bytes, unsigned char control_byte, unsigned char *buffer)
{
  static unsigned char i2c_buffer[256];

  i2c_buffer[0] = control_byte;
  memcpy(i2c_buffer + 1, buffer, num_bytes);
  return i2c_write(SSD1306_I2C_ADDRESS, i2c_buffer, num_bytes + 1, I2C_TIMEOUT);
}
