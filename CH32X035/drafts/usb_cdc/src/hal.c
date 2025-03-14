#include "board.h"
#include <usb_device_x035.h>
#include <usb_device.h>
#include "delay.h"

void SysInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  Delay_Init();
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBFS, ENABLE );
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
