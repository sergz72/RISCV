#include "board.h"
#include "delay.h"

static void GPIOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  //Led
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = LED_PIN;
  GPIO_Init(LED_PORT, &GPIO_InitStructure);

}

static void USBInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_16;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_17;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBFS, ENABLE );
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );
}

void SysInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  Delay_Init();

  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIOInit();
  USBInit();
}
