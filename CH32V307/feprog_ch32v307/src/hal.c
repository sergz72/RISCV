#include "board.h"
#include <ch32v30x_gpio.h>
#include "debug.h"
#include "delay.h"

static void GPIOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // led green
  GPIO_InitStructure.GPIO_Pin = LED_BLUE_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED_BLUE_PORT, &GPIO_InitStructure);

  // led red
  GPIO_InitStructure.GPIO_Pin = LED_RED_PIN;
  GPIO_Init(LED_RED_PORT, &GPIO_InitStructure);
}

void HalInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  Delay_Init();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  GPIOInit();
}