#include "board.h"
#include "delay.h"
#include "ch32x035_usbfs_device.h"
#include <ch32x035_pwr.h>
#include <usb_cdc.h>

static int led_state;

static void led_toggle(void)
{
  led_state = !led_state;
  if (led_state)
    LED_ON;
  else
    LED_OFF;
}

static unsigned char cdc_rx_buffer[CDC_RX_BUF_LEN];

int main(void)
{
  int counter = 0;

  SysInit();

  led_state = 0;

  USBFS_RCC_Init( );
  USBFS_Device_Init( ENABLE , PWR_VDD_SupplyVoltage());

  TimerEnable();

  while (1)
  {
    __WFI();
    if (timer_interrupt)
    {
      if (counter == 99)
      {
        counter = 0;
        led_toggle();
      }
      else
        counter++;
      unsigned int length = CDC_Receive(cdc_rx_buffer, sizeof(cdc_rx_buffer));
      if (length)
        CDC_Transmit(cdc_rx_buffer, length);

      timer_interrupt = 0;
    }
  }
}
