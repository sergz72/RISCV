#include "board.h"
#include <delay.h>
#include <ch32v00X_tim.h>
#include <common_printf.h>
#include <string.h>
#include <ina228.h>
#include <spi_memory.h>

#include "spi_soft.h"

volatile unsigned int timer_interrupt;
volatile char command_line[COMMMAND_LINE_SIZE];
volatile char *command_line_p, *command_line_echo_p;
volatile bool command_ready;

#ifdef XPACK
void __attribute__((naked)) TIM2_IRQHandler(void)
#else
void __attribute__((interrupt("WCH-Interrupt-fast"))) TIM2_IRQHandler(void)
#endif
{
  timer_interrupt = 1;
  TIM2->INTFR = 0;
}

const USART_InitTypeDef USART_InitStructure = {
  .USART_BaudRate = USART_BAUDRATE,
  .USART_WordLength = USART_WordLength_8b,
  .USART_StopBits = USART_StopBits_1,
  .USART_Parity = USART_Parity_No,
  .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
  .USART_Mode = USART_Mode_Tx | USART_Mode_Rx
};

#ifdef XPACK
void __attribute__((naked)) USART_IRQHandler(void)
#else
void __attribute__((interrupt("WCH-Interrupt-fast"))) USART_IRQHandler(void)
#endif
{
  if(USART_GetITStatus(USART_INST, USART_IT_RXNE) != RESET)
  {
    char c = (char)USART_ReceiveData(USART_INST);
    if (command_ready)
      return;
    if (c == '\r')
      command_ready = true;
    else
      *command_line_p++ = c;
  }
}

static void GPIOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_InitStructure.GPIO_Pin = LED_TIMER_PIN;
  GPIO_Init(LED_TIMER_PORT, &GPIO_InitStructure);
}

static void TIM2Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM2, ENABLE );

  // 100ms interrupt
  TIM_TimeBaseInitStructure.TIM_Period = 10000-1;
  TIM_TimeBaseInitStructure.TIM_Prescaler = SystemCoreClock/100000-1;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMER_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

static void USARTInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure = {0};
  NVIC_InitTypeDef  NVIC_InitStructure = {0};

  USART_CLOCK_ENABLE;

  GPIO_InitStructure.GPIO_Pin = USART_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(USART_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = USART_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USART_PORT, &GPIO_InitStructure);

  USART_Init(USART_INST, (USART_InitTypeDef*)&USART_InitStructure);
  USART_ITConfig(USART_INST, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = USART_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_Cmd(USART_INST, ENABLE);
}

/*
 * I2C_INST SCL = PA10(6)
 * I2C_INST SDA = PA11(5)
 */
static void I2CInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitTSturcture;

  RCC_PB1PeriphClockCmd( I2C_CLOCK, ENABLE );

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init( GPIOC, &GPIO_InitStructure );

  I2C_InitTSturcture.I2C_ClockSpeed = 400000;
  I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
  I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
  I2C_InitTSturcture.I2C_OwnAddress1 = 0;
  I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
  I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init( I2C_INST, &I2C_InitTSturcture );

  I2C_Cmd( I2C_INST, ENABLE );
}

static void SPIInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = SPI_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(SPI_MISO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SPI_MOSI_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
  GPIO_Init(SPI_SCK_PORT, &GPIO_InitStructure);
  SPI_CS_SET(0);
  GPIO_InitStructure.GPIO_Pin = SPI_CS_PIN;
  GPIO_Init(SPI_CS_PORT, &GPIO_InitStructure);
}

void SysInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  Delay_Init();
  GPIOInit();
  TIM2Init();
  I2CInit();
  SPIInit();
  USARTInit();
  timer_interrupt = 0;
}

void delayms(unsigned int ms)
{
  Delay_Ms(ms);
}

void TimerEnable(void)
{
  TIM_Cmd(TIM2, ENABLE);
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

int ina228ReadRegister16(int channel, unsigned char address, unsigned char reg, unsigned short *data)
{
  unsigned char d[2];
  int rc = i2c_write(address, &reg, 1, I2C_TIMEOUT, false);
  if (rc)
    return rc;
  rc = i2c_read(address, d, 2, I2C_TIMEOUT);
  if (!rc)
    *data = (d[0] << 8) | d[1];
  return rc;
}

int ina228WriteRegister16(int channel, unsigned char address, const unsigned char reg, const unsigned short data)
{
  unsigned char d[3];
  d[0] = reg;
  d[1] = data >> 8;
  d[2] = data;
  return i2c_write(address, d, 3, I2C_TIMEOUT, true);
}

int ina228ReadRegister24(int channel, unsigned char address, unsigned char reg, unsigned int *data)
{
  unsigned char d[3];
  int rc = i2c_write(address, &reg, 1, I2C_TIMEOUT, false);
  if (rc)
    return rc;
  rc = i2c_read(address, d, 3, I2C_TIMEOUT);
  if (!rc)
    *data = (d[0] << 16) | (d[1] << 8) | d[2];
  return rc;
}

int ina228WriteRegister24(int channel, unsigned char address, unsigned char reg, unsigned int data)
{
  unsigned char d[4];
  d[0] = reg;
  d[1] = data >> 16;
  d[2] = data >> 8;
  d[3] = data;
  return i2c_write(address, d, 4, I2C_TIMEOUT, true);
}

int SSD1306_I2C_Write(int num_bytes, unsigned char control_byte, unsigned char *buffer)
{
  static unsigned char i2c_buffer[256];

  i2c_buffer[0] = control_byte;
  memcpy(i2c_buffer + 1, buffer, num_bytes);
  return i2c_write(SSD1306_I2C_ADDRESS >> 1, i2c_buffer, num_bytes + 1, I2C_TIMEOUT, true);
}

void spi_finish(int channel)
{
  SPI_CS_SET(0);
}

void spi_trfr(int channel, int nwrite, const unsigned char *wdata, int nop_cycles, int nread, unsigned char *rdata, int set_cs)
{
  SPI_CS_CLR(0);
  while (nwrite--)
    spi_byte(0, *wdata++);
  nop_cycles >>= 3;
  while (nop_cycles--)
    spi_byte(0, 0);
  if (nread)
  {
    while (nread--)
      *rdata++ = spi_byte(0, 0);
  }

  if (set_cs)
    spi_finish(channel);
}

void usart_transmit(char c)
{
  while(USART_GetFlagStatus(USART_INST, USART_FLAG_TXE) == RESET) /* waiting for sending finish */
    ;
  USART_SendData(USART_INST, c);
}

void puts_(const char *s)
{
  while (*s)
    usart_transmit(*s++);
}

void GPIO_IPD_Unused(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOB | RCC_PB2Periph_GPIOC | RCC_PB2Periph_GPIOD, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_3|GPIO_Pin_4
                               |GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2
                               |GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5
                               |GPIO_Pin_6;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_3;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}
