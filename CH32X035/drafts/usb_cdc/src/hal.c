#include "board.h"
#include <usb_device_x035.h>
#include <usb_device.h>
#include "delay.h"

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

void SysInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  Delay_Init();
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBFS, ENABLE );
  GPIOInit();
}

#if __GNUC__ > 13
void __attribute__((naked)) void USBFS_IRQHandler(void)
#else
void __attribute__((interrupt("WCH-Interrupt-fast"))) USBFS_IRQHandler(void)
#endif
{
  USBDeviceInterruptHandler();
#if __GNUC__ > 13
  asm volatile ("mret");
#endif
}
