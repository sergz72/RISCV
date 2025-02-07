#include "hal.h"
#include <ch32x035.h>
#include "handler.h"
#include <spi_soft.h>

#define ADC_VREF 33000 //x0.1mv

static unsigned char *rxbuf_p, *txbuf_p;

static inline void pointers_reset(void *rxaddress, const void *txaddress)
{
  rxbuf_p = (unsigned char*)rxaddress;
  txbuf_p = (unsigned char*)txaddress;
}

#if __GNUC__ > 13
void __attribute__((naked)) TIM2_UP_IRQHandler(void)
#else
void __attribute__((interrupt("WCH-Interrupt-fast"))) TIM2_UP_IRQHandler(void)
#endif
{
  timer_interrupt = 1;
  TIM2->INTFR = 0;
#if __GNUC__ > 13
  asm volatile ("mret");
#endif
}

#if __GNUC__ > 13
void __attribute__((naked)) SPI1_IRQHandler(void)
#else
void __attribute__((interrupt("WCH-Interrupt-fast"))) SPI1_IRQHandler(void)
#endif
{
  unsigned int status = SPI1->STATR;
  if (status & SPI_STATR_TXE)
    SPI1->DATAR = *txbuf_p++;
  if (status & SPI_STATR_RXNE)
    *rxbuf_p++ = SPI1->DATAR;
#if __GNUC__ > 13
  asm volatile ("mret");
#endif
}

#if __GNUC__ > 13
void __attribute__((naked)) EXTI7_0_IRQHandler(void)
#else
void __attribute__((interrupt("WCH-Interrupt-fast"))) EXTI7_0_IRQHandler(void)
#endif
{
  pointers_reset(rxbuf, txbufs[rxbuf[0]]);
  SPI1->DATAR = *txbuf_p++;
  command_ready = 1;
  EXTI->INTFR = 0x1FFFFF;
#if __GNUC__ > 13
  asm volatile ("mret");
#endif
}

/*
 * SPI1_NSS  = PA4
 * SPI1_SCK  = PA5
 * SPI1_MISO = PA6
 * SPI1_MOSI = PA7
 */

static void spi_slave_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( GPIOA, &GPIO_InitStructure );

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init( GPIOA, &GPIO_InitStructure );

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init( GPIOA, &GPIO_InitStructure );

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init( SPI1, &SPI_InitStructure );

  NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  SPI1->CTLR2 |= SPI_CTLR2_TXEIE | SPI_CTLR2_RXNEIE;

  SPI_Cmd( SPI1, ENABLE );
}

void exti_init(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //high priority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*
 * SPI_NSS  = PB11
 * SPI_SCK  = PC15
 * SPI_MOSI = PC14
 */

static void spi_master_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  SPI_CS_SET(0);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( GPIOB, &GPIO_InitStructure );
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14;
  GPIO_Init( GPIOC, &GPIO_InitStructure );
}

static void timer_init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE );

  // 5ms interrupt
  TIM_TimeBaseInitStructure.TIM_Period = 5000-1;
  TIM_TimeBaseInitStructure.TIM_Prescaler = SystemCoreClock/1000000-1;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //low priority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

/*
 * ADC_IN9 = PB1
 */

static void adc_init(void)
{
  ADC_InitTypeDef  ADC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  ADC_CLKConfig(ADC1, ADC_CLK_Div6);

  GPIO_InitStructure.GPIO_Pin = ADC_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(ADC_PORT, &GPIO_InitStructure);

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigInjecConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_CHANNEL, 1, ADC_SampleTime_11Cycles);

  ADC_Cmd(ADC1, ENABLE);
}

/*
 *LED_TIMER   = PB2
 *LED_COMMAND = PA3
 *INTERRUPT   = PB7
 */

static void ports_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
#ifdef LED_TIMER_ON
  GPIO_InitStructure.GPIO_Pin = LED_TIMER_PIN;
  GPIO_Init(LED_TIMER_PORT, &GPIO_InitStructure);
#endif
#ifdef LED_COMMAND_ON
  GPIO_InitStructure.GPIO_Pin = LED_COMMAND_PIN;
  GPIO_Init(LED_COMMAND_PORT, &GPIO_InitStructure);
#endif
}

void SysInit(void *rxaddress, const void *txaddress)
{
  pointers_reset(rxaddress, txaddress);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE );
  ports_init();
  spi_slave_init();
  exti_init();
  spi_master_init();
  timer_init();
  adc_init();
}

void ad9833_write(int channel, unsigned short data)
{
  unsigned char cmd, d;
  cmd = data >> 8;
  d = data & 0xFF;
  spi_command(0, cmd, &d, NULL, 1, 1);
}

/* ADC Software start mask */
#define CTLR2_EXTTRIG_SWSTART_Set        ((uint32_t)0x00500000)
#define CTLR2_SWSTART_Set                ((uint32_t)0x00400000)

unsigned short adc_get(void)
{
  ADC1->CTLR2 |= CTLR2_EXTTRIG_SWSTART_Set;
  while (ADC1->CTLR2 & CTLR2_SWSTART_Set)
    ;
  unsigned int result = ADC1->RDATAR;
  unsigned short mV = (unsigned short)((result * ADC_VREF) >> 12);
  return mV;
}

void timer_enable(void)
{
  TIM_Cmd(TIM2, ENABLE);
}

void status_updated(void)
{
}
