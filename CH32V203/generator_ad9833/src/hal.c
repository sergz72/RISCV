#include "hal.h"
#include <ch32v20x.h>
#include "handler.h"

#define ADC_VREF 33000 //x0.1mv

static short calibrattion_val;
static unsigned char *rxbuf_p, *txbuf_p;

static inline void pointers_reset(void *rxaddress, const void *txaddress)
{
  rxbuf_p = (unsigned char*)rxaddress;
  txbuf_p = (unsigned char*)txaddress;
}

#if __GNUC__ > 13
void __attribute__((naked)) TIM2_IRQHandler(void)
#else
void __attribute__((interrupt("WCH-Interrupt-fast"))) TIM2_IRQHandler(void)
#endif
{
  timer_interrupt = 1;
  TIM2->INTFR = 0;
#if __GNUC__ > 13
  asm volatile ("mret");
#endif
}

#ifdef INTERFACE_I2C
#if __GNUC__ > 13
void __attribute__((naked)) I2C1_EV_IRQHandler(void)
#else
void __attribute__((interrupt("WCH-Interrupt-fast"))) I2C1_EV_IRQHandler(void)
#endif
{
  unsigned int status = I2C1->STAR1;
  unsigned int dummy;

  if (status & I2C_IT_ADDR & (status & (I2C_FLAG_TRA | I2C_FLAG_TXE) != I2C_FLAG_TRA | I2C_FLAG_TXE)) // read mode
  {
    dummy = I2C1->STAR2;
    (void)dummy;
  }
  if (status & I2C_IT_RXNE) // read
    *rxbuf_p++ = I2C1->DATAR;
  else if (status & I2C_IT_TXE) // write
    I2C1->DATAR = *txbuf_p++;
  else if (status & I2C_IT_STOPF)
  {
    I2C1->CTLR1 &= I2C1->CTLR1;
    ((void)(I2C1->STAR1));
    pointers_reset(rxbuf, txbufs[rxbuf[0]]);
    command_ready = 1;
  }
  else if (status & (I2C_IT_BTF | I2C_IT_SB))
  {
    ((void)(I2C1->STAR1));
    dummy = I2C1->DATAR;
    (void)dummy;
  }
#if __GNUC__ > 13
  asm volatile ("mret");
#endif
}

#if __GNUC__ > 13
void __attribute__((naked)) I2C1_ER_IRQHandler(void)
#else
void __attribute__((interrupt("WCH-Interrupt-fast"))) I2C1_ER_IRQHandler(void)
#endif
{
  unsigned int status = I2C1->STAR1;
  if (status & I2C_IT_AF)
    I2C1->STAR1 = ~(unsigned short)(I2C_IT_AF & 0xFFFF);
#if __GNUC__ > 13
  asm volatile ("mret");
#endif
}


/*
 * I2C1 SCL = PB8
 * I2C1 SDA = PB9
 */

static void i2c_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitTSturcture;
  NVIC_InitTypeDef NVIC_InitStructure;

	GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( GPIOB, &GPIO_InitStructure );

  I2C_InitTSturcture.I2C_ClockSpeed = 100000;
  I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
  I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
  I2C_InitTSturcture.I2C_OwnAddress1 = I2C_ADDRESS;
  I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
  I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init( I2C1, &I2C_InitTSturcture );

  I2C_Cmd( I2C1, ENABLE );

  NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure );

  NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure );

  I2C_ITConfig( I2C1, I2C_IT_BUF, ENABLE );
  I2C_ITConfig( I2C1, I2C_IT_EVT, ENABLE );
  I2C_ITConfig( I2C1, I2C_IT_ERR, ENABLE );
}
#else
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
void __attribute__((naked)) EXTI4_IRQHandler(void)
#else
void __attribute__((interrupt("WCH-Interrupt-fast"))) EXTI4_IRQHandler(void)
#endif
{
  pointers_reset(rxbuf, txbufs[rxbuf[0]]);
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

  SPIx->CTLR2 |= SPI_CTLR2_TXEIE | SPI_CTLR2_RXNEIE;

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

  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //high priority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

#endif

/*
 * SPI2_NSS  = PB12
 * SPI2_SCK  = PB13
 * SPI2_MISO = PB14
 * SPI2_MOSI = PB15
 */

static void spi_master_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( GPIOB, &GPIO_InitStructure );

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init( GPIOB, &GPIO_InitStructure );

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init( SPI2, &SPI_InitStructure );

  SPI_Cmd( SPI2, ENABLE );
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

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
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
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);

  GPIO_InitStructure.GPIO_Pin = ADC_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(ADC_PORT, &GPIO_InitStructure);

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigInjecConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);

  ADC_Cmd(ADC1, ENABLE);

  ADC_BufferCmd(ADC1, DISABLE); //disable buffer
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1))
    ;
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1))
    ;
  calibrattion_val = Get_CalibrationValue(ADC1);
  //ADC_BufferCmd(ADC1, ENABLE); //enable buffer
}

/*
 *LED_TIMER   = PA2
 *LED_COMMAND = PA3
 *INTERRUPT   = PB7
 */

static void ports_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
#ifdef INTERFACE_I2C
  GPIO_InitStructure.GPIO_Pin = INTERRUPT_PIN;
  GPIO_Init(INTERRUPT_PIN_PORT, &GPIO_InitStructure);
#endif
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
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE );
  ports_init();
#ifdef INTERFACE_I2C
  i2c_init();
#else
  spi_slave_init();
  exti_init();
#endif
  spi_master_init();
  timer_init();
  adc_init();
  pointers_reset(rxaddress, txaddress);
}

void ad9833_write(int channel, unsigned short data)
{
  SPI_I2S_SendData(SPI2, data);
}

static unsigned short Get_ConversionVal(unsigned short val)
{
  short v = (short)val + calibrattion_val;
  if (v <= 0)
    return 0;
  if (v > 4095)
    return 4095;
  return v;
}

/* ADC Software start mask */
#define CTLR2_EXTTRIG_SWSTART_Set        ((uint32_t)0x00500000)
#define CTLR2_SWSTART_Set                ((uint32_t)0x00400000)

unsigned short adc_get(void)
{
  ADC1->CTLR2 |= CTLR2_EXTTRIG_SWSTART_Set;
  while (ADC1->CTLR2 & CTLR2_SWSTART_Set)
    ;
  unsigned int result = Get_ConversionVal(ADC1->RDATAR);
  unsigned short mV = (unsigned short)((result * ADC_VREF) >> 12);
  return mV;
}

void timer_enable(void)
{
    TIM_Cmd(TIM2, ENABLE);
}

void status_updated(void)
{
#ifdef INTERFACE_I2C
  if (status)
    INTERRUPT_PIN_PORT->BSHR = INTERRUPT_PIN;
  else
    INTERRUPT_PIN_PORT->BCR = INTERRUPT_PIN;
#endif
}
