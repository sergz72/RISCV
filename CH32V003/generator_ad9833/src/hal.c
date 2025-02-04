#include "hal.h"
#include <ch32v00x.h>
#include <spi_soft.h>
#include <ch32v00x_misc.h>

inline static void dma_enable(void)
{
  //Enable DMA Channels
  DMA1_Channel3->CFGR |= DMA_CFGR3_EN;
  DMA1_Channel2->CFGR |= DMA_CFGR2_EN;
}

static void dma_disable(void *rxaddress, const void *txaddress)
{
  DMA1_Channel3->CFGR &= ~DMA_CFGR3_EN;
  DMA1_Channel2->CFGR &= ~DMA_CFGR3_EN;
  //Buffer address
  DMA1_Channel3->MADDR = (uint32_t)txaddress;
  //Number of data transfer
  DMA1_Channel3->CNTR = MAX_SPI_TRANSFER_SIZE;
  //Buffer address
  DMA1_Channel2->MADDR = (uint32_t)rxaddress;
  //Number of data transfer
  DMA1_Channel2->CNTR = MAX_SPI_TRANSFER_SIZE;
}

inline static void spi_enable(void)
{
  //Enable SPI1
  SPI1->CTLR1 |= SPI_CTLR1_SPE;
  dma_enable();
}

inline static void spi_disable(void *rxaddress, const void *txaddress)
{
  //Disable SPI1
  dma_disable(rxaddress, txaddress);
  SPI1->CTLR1 &= ~SPI_CTLR1_SPE;
}

#if __GNUC__ > 13
void __attribute__((naked)) EXTI7_0_IRQHandler(void)
#else
void __attribute__((interrupt("WCH-Interrupt-fast"))) EXTI7_0_IRQHandler(void)
#endif
{
  if (EXTI->INTFR & SPI_NCS_EXTI_LINE)
  {
    if (GPIOC->INDR & SPI_NCS_PIN) // Rising edge
    {
      spi_disable(spi_rxbuf, spi_txbufs[spi_rxbuf[0]]);
      command_ready = 1;
    }
    else
      spi_enable();
    EXTI->INTFR = SPI_NCS_EXTI_LINE;
  }
#if __GNUC__ > 13
  asm volatile ("mret");
#endif
}

#if __GNUC__ > 13
void __attribute__((naked)) TIM1_UP_IRQHandler(void)
#else
void __attribute__((interrupt("WCH-Interrupt-fast"))) TIM1_UP_IRQHandler(void)
#endif
{
  timer_interrupt = 1;
  TIM1->INTFR = 0;
#if __GNUC__ > 13
  asm volatile ("mret");
#endif
}

/*
  CH32V003F4P6
  PC1 - NSS
  PC5 - SCK
  PC6 - MOSI
  PC7 - MISO
 */

static void spi_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  //Enable clock for PORTC, SPI1 and DMA1
  RCC->APB2PCENR |=  RCC_IOPCEN | RCC_SPI1EN | RCC_AFIOEN;
  RCC->AHBPCENR |= RCC_DMA1EN;

  //PC6(MOSI) and PC5(SCK) - Floating inputs
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  //PC7(MISO) - Multiplexed push pull output
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //Set SPI1, max clock 48Mhz/2 = 24Mhz, slave mode, full-duplex mode,8bit data length
  //SPI1->CTLR1 = SPI_CTLR1_SSI | SPI_CTLR1_SSM;
  //Enable DMA for SPI TX and SPI RX
  SPI1->CTLR2 = SPI_CTLR2_TXDMAEN | SPI_CTLR2_RXDMAEN;

  //Set DMA1 - Channel 3, Very high priority, 8bit size, mem -> spi direction
  // memory increment and no perpheral increment
  DMA1_Channel3->CFGR |= DMA_CFGR1_PL_0 | DMA_CFGR1_PL_1 | DMA_CFGR1_DIR | DMA_CFGR1_MINC;
  //Take SPI1 Data register address
  DMA1_Channel3->PADDR = (uint32_t)&SPI1->DATAR;

  //Set DMA1 - Channel 2, Very high priority, 8bit size, spi -> mem direction
  // memory increment and no perpheral increment
  DMA1_Channel2->CFGR = DMA_CFGR1_PL_0 | DMA_CFGR1_PL_1 | DMA_CFGR1_MINC;
  //Take SPI1 Data register address
  DMA1_Channel2->PADDR = (uint32_t)&SPI1->DATAR;
}

static void exti_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  GPIO_InitStructure.GPIO_Pin = SPI_NCS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, SPI_NCS_EXTI_SOURCE);
  EXTI_InitStructure.EXTI_Line = SPI_NCS_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //high priority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

static void timer_init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE );

  // 5ms interrupt
  TIM_TimeBaseInitStructure.TIM_Period = 1000-1;
  TIM_TimeBaseInitStructure.TIM_Prescaler = 48-1;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 5;
  TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

  TIM_ClearITPendingBit( TIM1, TIM_IT_Update );

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //low priority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
}

static void adc_init(void)
{
  ADC_InitTypeDef  ADC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);

  GPIO_InitStructure.GPIO_Pin = ADC_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigInjecConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_CHANNEL, 1, ADC_SampleTime_241Cycles);

  ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);

  ADC_Cmd(ADC1, ENABLE);

  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));
}

//Sets System clock frequency to 48MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
static void SetSysClock(void)
{
  FLASH->ACTLR = 1; // 1 wait (recommended 24=<SYSCLK=<48MHz)
  /* HCLK = SYSCLK = APB1 */
  RCC->CFGR0 = RCC_HPRE_DIV1 | RCC_PLLSRC_HSI_Mul2;
  /* Enable PLL */
  RCC->CTLR |= RCC_PLLON;
  /* Wait till PLL is ready */
  while((RCC->CTLR & RCC_PLLRDY) == 0)
  {
  }
  RCC->CFGR0 |= RCC_SW_PLL;
  /* Wait till PLL is used as system clock source */
  while ((RCC->CFGR0 & RCC_SWS) != RCC_SWS_PLL)
  {
  }
}

static void ports_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
#ifdef LED_TIMER_ON
  RCC->APB2PCENR |= LED_TIMER_IOPC;
  GPIO_InitStructure.GPIO_Pin = LED_TIMER_PIN;
  GPIO_Init(LED_TIMER_PORT, &GPIO_InitStructure);
#endif
#ifdef LED_COMMAND_ON
  RCC->APB2PCENR |= LED_COMMAND_IOPC;
  GPIO_InitStructure.GPIO_Pin = LED_COMMAND_PIN;
  GPIO_Init(LED_COMMAND_PORT, &GPIO_InitStructure);
#endif
}

void SysInit(void *rxaddress, const void *txaddress)
{
  SetSysClock();
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  SystemCoreClockUpdate();
  ports_init();
  spi_init();
  exti_init();
  dma_disable(rxaddress, txaddress);
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

unsigned short adc_get(void)
{
  //todo
  return 0;
}

void timer_enable(void)
{
    TIM_Cmd( TIM1, ENABLE );
}