#include "board.h"
#include <mcp3426.h>
#include <delay.h>
#include <string.h>

static void ports_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

  RCC->APB2PCENR |= RCC_IOPAEN | RCC_IOPBEN;

  GPIO_InitStructure.GPIO_Pin = LED_TIMER_PIN;
  GPIO_Init(LED_TIMER_PORT, &GPIO_InitStructure);
}

/*
 * I2C1 SCL = PB6
 * I2C1 SDA = PB7
 */

static void i2c_master1_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitTSturcture;

  RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( GPIOB, &GPIO_InitStructure );

  I2C_InitTSturcture.I2C_ClockSpeed = 100000;
  I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
  I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
  I2C_InitTSturcture.I2C_OwnAddress1 = 0;
  I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
  I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init( I2C1, &I2C_InitTSturcture );

  I2C_Cmd( I2C1, ENABLE );
}

/*
 * I2C1 SCL = PB10
 * I2C1 SDA = PB11
 */

static void i2c_master2_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitTSturcture;

  RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C2, ENABLE );

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( GPIOB, &GPIO_InitStructure );

  I2C_InitTSturcture.I2C_ClockSpeed = 100000;
  I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
  I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
  I2C_InitTSturcture.I2C_OwnAddress1 = 0;
  I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
  I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init( I2C2, &I2C_InitTSturcture );

  I2C_Cmd( I2C2, ENABLE );
}

static void pwm_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  GPIO_InitStructure.GPIO_Pin = PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(PWM_PORT, &GPIO_InitStructure);

  TIM_TimeBaseInitStructure.TIM_Period = PWM_PERIOD - 1; // 100 khz
  TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = PWM_PULSE;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
  PWM_OCINIT( TIM1, &TIM_OCInitStructure );

  TIM_CtrlPWMOutputs(TIM1, ENABLE );
  PWM_OCPRELOADCONFIG( TIM1, TIM_OCPreload_Disable );
  TIM_ARRPreloadConfig( TIM1, ENABLE );
  TIM_Cmd( TIM1, ENABLE );
}

static void opa1_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  OPA_InitTypeDef  OPA_InitStructure;

  GPIO_InitStructure.GPIO_Pin = OPA1_N_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init( OPA1_N_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = OPA1_P_PIN;
  GPIO_Init( OPA1_P_PORT, &GPIO_InitStructure);

  OPA_InitStructure.OPA_NUM = OPA1;
  OPA_InitStructure.PSEL = CHP1;
  OPA_InitStructure.NSEL = CHN1;
  OPA_InitStructure.Mode = OUT_IO_OUT0;
  OPA_Init( &OPA_InitStructure );
  OPA_Cmd( OPA1, ENABLE );
}

static void opa2_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  OPA_InitTypeDef  OPA_InitStructure;

  GPIO_InitStructure.GPIO_Pin = OPA2_N_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init( OPA2_N_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = OPA2_P_PIN;
  GPIO_Init( OPA2_P_PORT, &GPIO_InitStructure);

  OPA_InitStructure.OPA_NUM = OPA2;
  OPA_InitStructure.PSEL = CHP1;
  OPA_InitStructure.NSEL = CHN1;
  OPA_InitStructure.Mode = OUT_IO_OUT0;
  OPA_Init( &OPA_InitStructure );
  OPA_Cmd( OPA2, ENABLE );
}

void SysInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  RCC->APB2PCENR |= RCC_AFIOEN;
  Delay_Init();
  ports_init();
  i2c_master1_init();
  i2c_master2_init();
  opa1_init();
  opa2_init();
  pwm_init();
}

static int i2c_read(I2C_TypeDef *instance, unsigned char address, unsigned char *data, unsigned int l, unsigned int timeout)
{
  unsigned int t;

  instance->CTLR1 |= I2C_CTLR1_START;

  t = timeout;
  while( !I2C_CheckEvent( instance, I2C_EVENT_MASTER_MODE_SELECT ) )
  {
    t--;
    if (!t)
      return 1;
  }
  I2C_Send7bitAddress( instance, address, I2C_Direction_Receiver );
  t = timeout;
  while( !I2C_CheckEvent( instance, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) )
  {
    t--;
    if (!t)
      return 2;
  }
  while (l--)
  {
    t = timeout;
    while ( !I2C_GetFlagStatus( instance, I2C_FLAG_RXNE ) )
    {
      //todo I2C_AcknowledgeConfig( instance, DISABLE );
      t--;
      if (!t)
        return 3;
    }
    *data++ = I2C_ReceiveData(instance);
  }
  I2C_GenerateSTOP( instance, ENABLE );
  return 0;
}

static int i2c_write(I2C_TypeDef *instance, unsigned char address, unsigned char *data, unsigned int l, unsigned int timeout)
{
  unsigned int t;

  instance->CTLR1 |= I2C_CTLR1_START;

  t = timeout;
  while( !I2C_CheckEvent( instance, I2C_EVENT_MASTER_MODE_SELECT ) )
  {
    t--;
    if (!t)
      return 1;
  }
  I2C_Send7bitAddress( instance, address, I2C_Direction_Transmitter );
  t = timeout;
  while( !I2C_CheckEvent( instance, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) )
  {
    t--;
    if (!t)
      return 2;
  }
  while (l--)
  {
    t = timeout;
    while ( !I2C_GetFlagStatus( instance, I2C_FLAG_TXE ) )
    {
      t--;
      if (!t)
        return 3;
    }
    I2C_SendData( I2C1, *data++);
  }
  t = timeout;
  while( !I2C_CheckEvent( instance, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) )
  {
    t--;
    if (!t)
      return 3;
  }
  I2C_GenerateSTOP( instance, ENABLE );
  return 0;
}

int mcp3426Read(int channel, unsigned char address, unsigned char *data, unsigned int l)
{
  return i2c_read(MCP3426_I2C_INSTANCE, address, data, l, I2C_TIMEOUT);
}

int mcp3426Write(int channel, unsigned char address, unsigned char data)
{
  return i2c_write(MCP3426_I2C_INSTANCE, address, &data, 1, I2C_TIMEOUT);
}

int SSD1306_I2C_Write(int num_bytes, unsigned char control_byte, unsigned char *buffer)
{
  static unsigned char i2c_buffer[256];

  i2c_buffer[0] = control_byte;
  memcpy(i2c_buffer + 1, buffer, num_bytes);

  return i2c_write(SSD1306_I2C_INSTANCE, SSD1306_I2C_ADDRESS, i2c_buffer, num_bytes + 1, I2C_TIMEOUT);
}
