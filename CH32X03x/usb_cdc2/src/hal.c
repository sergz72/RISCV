#include "board.h"
#include "delay.h"
#include "ch32x035_tim.h"

volatile unsigned int timer_interrupt;

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

static void GPIOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = LED_PIN;
  GPIO_Init(LED_PORT, &GPIO_InitStructure);
}

static void TIM2Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE );

  // 10ms interrupt
  TIM_TimeBaseInitStructure.TIM_Period = 10000-1;
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

void SysInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  Delay_Init();
  GPIOInit();
  TIM2Init();

  timer_interrupt = 0;
}

void TimerEnable(void)
{
  TIM_Cmd(TIM2, ENABLE);
}
