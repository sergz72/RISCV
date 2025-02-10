#include "board.h"
#include <i2c_soft.h>

static void ports_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

  RCC->APB2PCENR |= RCC_IOPAEN | RCC_IOPCEN | RCC_IOPDEN;

  GPIO_InitStructure.GPIO_Pin = LED_TIMER_PIN;
  GPIO_Init(LED_TIMER_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = INTERRUPT_FLAG_PIN;
  GPIO_Init(INTERRUPT_FLAG_PORT, &GPIO_InitStructure);
}

static void i2c_slave_init(void)
{
  //todo
}

static void i2c_master_init(void)
{
  //todo
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

void SysInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  RCC->APB2PCENR |= RCC_AFIOEN;
  Delay_Init();
  ports_init();
  i2c_slave_init();
  i2c_master_init();
  pwm_init();
}
