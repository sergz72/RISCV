#include "board.h"
#include "debug.h"
#include "ch32v30x_usbhs_device.h"
#include <usb_cdc.h>

static int led_state;

static unsigned char usb_cdc_buffer[USB_CDC_RX_BUFFER_SIZE];

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
  led_state = 0;

  HalInit();

  USBHS_RCC_Init( );
  USBHS_Device_Init( ENABLE );

  LEDSToggle();

  while (1)
  {
    unsigned int length = CDC_Receive(usb_cdc_buffer, sizeof(usb_cdc_buffer));
    if (length)
    {
      LEDSToggle();
      CDC_Transmit(usb_cdc_buffer, length);
    }
  }
}
