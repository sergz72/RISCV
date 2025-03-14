#include "board.h"
#include "delay.h"
#include "usb_device_x035.h"
#include "class/cdc/usb_cdc.h"

int led_state;

static void led_toggle(void)
{
  led_state = !led_state;
  if (led_state)
    LED_ON;
  else
    LED_OFF;
}

void cdc_rx_callback(unsigned int port_id, unsigned char *buffer, unsigned int buffer_length)
{

}

int main(void)
{
  SysInit();

  led_state = 0;

  if (USB_CDC_Init(x035_endpoints, 1, 0, 0, 100, cdc_rx_callback))
  {
    while (1)
      ;
  }
  USBInit();

  while (1)
  {
    Delay_Ms(1000);
    led_toggle();
  }
}
