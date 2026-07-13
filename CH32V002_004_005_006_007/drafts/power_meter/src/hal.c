#include "board.h"
#include <delay.h>
#include <ch32v00X_tim.h>
#include <common_printf.h>
#include <string.h>
#include <ina228.h>
#include <spi_memory.h>
#include <spi_soft.h>
#include <i2c.h>
#include <usart.h>

volatile bool timer_interrupt;
volatile char command;
volatile unsigned int time_since_boot_ms;
unsigned int prev_keyboard_status;

void __attribute__((interrupt("WCH-Interrupt-fast"))) TIM2_IRQHandler(void)
{
  timer_interrupt = 1;
  time_since_boot_ms += 100;
  TIM2->INTFR = 0;
}

void __attribute__((interrupt("WCH-Interrupt-fast"))) USART_IRQHandler(void)
{
  if(USART_GetITStatus(USART_INST, USART_IT_RXNE) != RESET)
    command = (char)USART_ReceiveData(USART_INST);
}

/*
 * LED_TIMER = PD4(8)
 */
static void GPIOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_InitStructure.GPIO_Pin = LED_TIMER_PIN;
  GPIO_Init(LED_TIMER_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = BUTTON1_PIN;
  GPIO_Init(BUTTON1_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = BUTTON2_PIN;
  GPIO_Init(BUTTON2_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = BUTTON3_PIN;
  GPIO_Init(BUTTON3_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Pin = ALERT_PIN;
  GPIO_Init(ALERT_PORT, &GPIO_InitStructure);
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

/*
 * CS   = PA2(13)
 * MISO = PC0(16)
 * MOSI = PC3(3)
 * CLK  = PC4(4)
 */
static void SPIInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  spi_channel_init(0, 0);

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
  command = 0;
  timer_interrupt = false;
  prev_keyboard_status = 0;
  time_since_boot_ms = 0;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  Delay_Init();
  GPIOInit();
  TIM2Init();
  I2CInit();
  SPIInit();
  USARTInit();
}

void delayms(unsigned int ms)
{
  Delay_Ms(ms);
}

void TimerEnable(void)
{
  TIM_Cmd(TIM2, ENABLE);
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

unsigned int get_keyboard_status(void)
{
  unsigned int status = BUTTON1_PRESSED ? 1 : 0;
  if (BUTTON2_PRESSED)
    status |= 2;
  if (BUTTON3_PRESSED)
    status |= 4;
  if ((prev_keyboard_status && !status) || (!prev_keyboard_status && status))
  {
    prev_keyboard_status = status;
    return status;
  }
  return 0;
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
