#include "board.h"
#include <string.h>

USART_InitTypeDef USART_InitStructure = {
  .USART_BaudRate = USART_BAUDRATE,
  .USART_WordLength = USART_WordLength_8b,
  .USART_StopBits = USART_StopBits_1,
  .USART_Parity = USART_Parity_No,
  .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
  .USART_Mode = USART_Mode_Tx | USART_Mode_Rx
};

volatile bool timer_interrupt;
#ifdef USART_ENABLED
volatile char command_line[COMMMAND_LINE_SIZE];
volatile char *command_line_p, *command_line_echo_p;
volatile bool command_ready;
#endif

void __attribute__((interrupt("WCH-Interrupt-fast"))) TIM_TIMER_HANDLER(void)
{
  if(TIM_GetITStatus(TIM_TIMER, TIM_IT_Update)==SET)
    timer_interrupt = true;
  TIM_ClearITPendingBit( TIM_TIMER, TIM_IT_Update );
}

#ifdef USART_ENABLED
void __attribute__((interrupt("WCH-Interrupt-fast"))) USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    char c = (char)USART_ReceiveData(USART1);
    if (command_ready)
      return;
    if (c == '\r')
      command_ready = true;
    else
      *command_line_p++ = c;
  }
}
#endif

static void ports_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

  RCC->APB2PCENR |= RCC_IOPAEN | RCC_IOPCEN | RCC_IOPDEN;

  GPIO_InitStructure.GPIO_Pin = LED_BATTERY_PIN;
  GPIO_Init(LED_BATTERY_PORT, &GPIO_InitStructure);
#ifdef USART_ENABLED
  GPIO_InitStructure.GPIO_Pin = LED_TIMER_PIN | LED_BATTERY_PIN;
  GPIO_Init(LED_TIMER_PORT, &GPIO_InitStructure);
#endif
}

/*
 * I2C_INST SCL = PC2(6)
 * I2C_INST SDA = PC1(5)
 */
static void i2c_master_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitTSturcture;

  RCC_APB1PeriphClockCmd( I2C_CLOCK, ENABLE );

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
  I2C_Init( I2C_INST, &I2C_InitTSturcture );

  RCC_APB1PeriphClockCmd( I2C_CLOCK, DISABLE );
}

void disable_i2c(void)
{
  I2C_Cmd( I2C_INST, DISABLE );
  RCC_APB1PeriphClockCmd( I2C_CLOCK, DISABLE );
}

void enable_i2c(void)
{
  RCC_APB1PeriphClockCmd( I2C_CLOCK, ENABLE );
  I2C_Cmd( I2C_INST, ENABLE );
}

static void pwm_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  RCC_APB2PeriphClockCmd(PWM_TIMER_CLOCK, ENABLE);

  AFIO->PCFR1 |= AFIO_PCFR1_TIM1_REMAP_0;

  GPIO_InitStructure.GPIO_Pin = PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(PWM_PORT, &GPIO_InitStructure);

  TIM_TimeBaseInitStructure.TIM_Period = PWM_PERIOD - 1; // 100 khz
  TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit( PWM_TIM, &TIM_TimeBaseInitStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
  PWM_OCINIT( PWM_TIM, &TIM_OCInitStructure );

  PWM_OCPRELOADCONFIG( PWM_TIM, TIM_OCPreload_Disable );
  TIM_ARRPreloadConfig( PWM_TIM, ENABLE );

  RCC_APB2PeriphClockCmd(PWM_TIMER_CLOCK, DISABLE);
}

void enable_pwm(uint16_t pulse)
{
  RCC_APB2PeriphClockCmd(PWM_TIMER_CLOCK, ENABLE);
  TIM_SetCompare2(PWM_TIM, pulse);
  TIM_CtrlPWMOutputs(PWM_TIM, ENABLE );
  TIM_Cmd( PWM_TIM, ENABLE );
}

void disable_pwm(void)
{
  TIM_CtrlPWMOutputs(PWM_TIM, DISABLE );
  TIM_Cmd( PWM_TIM, DISABLE );
  RCC_APB2PeriphClockCmd(PWM_TIMER_CLOCK, DISABLE);
}

void timer_init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure={0};
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

  RCC_APB1PeriphClockCmd(TIM_TIMER_CLOCK, ENABLE );

  TIM_TimeBaseInitStructure.TIM_Period = TIM_TIMER_PERIOD_LOW;
  TIM_TimeBaseInitStructure.TIM_Prescaler = TIM_TIMER_PRESCALER;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit( TIM_TIMER, &TIM_TimeBaseInitStructure);

  TIM_ClearITPendingBit( TIM_TIMER, TIM_IT_Update );

  NVIC_InitStructure.NVIC_IRQChannel = TIM_TIMER_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMER_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ITConfig(TIM_TIMER, TIM_IT_Update, ENABLE);
  TIM_ARRPreloadConfig( TIM_TIMER, ENABLE );
}

static void opa_init(void)
{
  OPA_InitTypeDef OPA_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = OPA_P_PIN | OPA_N_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(OPA_IN_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = OPA_OUT_PIN;
  GPIO_Init(OPA_OUT_PORT, &GPIO_InitStructure);

  OPA_InitStructure.NSEL = CHN0;
  OPA_InitStructure.PSEL = CHP0;
  OPA_Init(&OPA_InitStructure);
  OPA_Cmd(ENABLE);
}

static void adc_init(void)
{
  ADC_InitTypeDef  ADC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_ADCCLKConfig(RCC_PCLK2_Div2);

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigInjecConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  //OPA output is on PD4 (ADC channel 7)
  ADC_InjectedSequencerLengthConfig(ADC1, 1);
  ADC_InjectedChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_15Cycles);
  ADC_ExternalTrigInjectedConvCmd(ADC1, DISABLE);

  ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);

  ADC_Cmd(ADC1, ENABLE);

  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1))
    ;
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1))
    ;

  disable_adc();
}

unsigned short adc_get(void)
{
  ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
  ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);
  while (!(ADC1->STATR & ADC_JEOC))
    ;
  return ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
}

unsigned short get_vbat(void)
{
  //VREF is on ADC channel 8
  ADC_InjectedChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_15Cycles);
  unsigned int value = adc_get();
  //OPA output is on PD4 (ADC channel 7)
  ADC_InjectedChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_15Cycles);
  return value;
}

void enable_adc(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
}

void disable_adc(void)
{
  ADC_Cmd(ADC1, DISABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
}

#ifdef USART_ENABLED
static void usart_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure = {0};
  NVIC_InitTypeDef  NVIC_InitStructure = {0};

  command_line_p = command_line;
  command_line_echo_p = command_line;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);

  GPIO_InitStructure.GPIO_Pin = USART_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(USART_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = USART_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USART_PORT, &GPIO_InitStructure);

  USART_Init(USART1, &USART_InitStructure);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);;

  USART_Cmd(USART1, ENABLE);
}
#endif

// 3 MHz system clock
void set_low_system_clock(void)
{
  TIM_SetAutoreload(TIM_TIMER, TIM_TIMER_PERIOD_LOW);
  RCC->CFGR0 = (RCC->CFGR0 & 0xFFFFFF0F) | RCC_HPRE_DIV8;
  SystemCoreClock = 3000000;
#ifdef USART_ENABLED
  USART_Init(USART1, &USART_InitStructure);
#endif
}

// 24 MHz system clock
void set_high_system_clock(void)
{
  TIM_SetAutoreload(TIM_TIMER, TIM_TIMER_PERIOD_HIGH);
  RCC->CFGR0 &= 0xFFFFFF0F;
  SystemCoreClock = 24000000;
#ifdef USART_ENABLED
  USART_Init(USART1, &USART_InitStructure);
#endif
}

/*
 * I2C1 -> APB1
 * Periodic timer -> APB1
 * PWM Timer -> APB2
 * ADC -> APB2
 */
void SysInit(void)
{
  timer_interrupt = false;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  timer_init();

  set_low_system_clock();

  RCC->APB2PCENR |= RCC_AFIOEN;
  ports_init();
  i2c_master_init();
  pwm_init();
  opa_init();
  adc_init();
#ifdef USART_ENABLED
  usart_init();
#endif
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

#ifdef USART_ENABLED
void usart_transmit(char c)
{
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) /* waiting for sending finish */
    ;
  USART_SendData(USART1, c);
}

void puts_(const char *s)
{
  while (*s)
    usart_transmit(*s++);
}

void pwm_set_duty(unsigned int duty)
{
  TIM_SetCompare2(PWM_TIM, duty);
}
#endif

void delayms(int ms)
{
}

void pwm_on(unsigned short duty)
{
  RCC_ADCCLKConfig(RCC_PCLK2_Div16);
  set_high_system_clock();
  enable_pwm(duty);
}

void pwm_off(void)
{
  disable_pwm();
  set_low_system_clock();
  RCC_ADCCLKConfig(RCC_PCLK2_Div2);
}
