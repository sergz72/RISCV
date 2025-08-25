#include "board.h"
#include "debug.h"
#include "ch32v30x_usbhs_device.h"
#include <usb_cdc.h>

static int led_state;

static void LEDSToggle(void)
{
  led_state = !led_state;
  if (led_state)
  {
    LED_BLUE_ON;
    LED_RED_OFF;
  }
  else
  {
    LED_BLUE_OFF;
    LED_RED_ON;
  }
}

int main(void)
{
  unsigned char *buffer;

  led_state = 0;

  HalInit();

  USBHS_RCC_Init( );
  USBHS_Device_Init( ENABLE );

  LEDSToggle();

  while (1)
  {
    CDC_ReceiveEnable();
    unsigned int length = CDC_Receive(&buffer);
    if (length)
    {
      LEDSToggle();
      CDC_Transmit(buffer, length);
    }
  }
}
