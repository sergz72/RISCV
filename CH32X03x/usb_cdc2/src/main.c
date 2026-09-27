#include "board.h"
#include "ch32x035_usbfs_device.h"
#include <ch32x035_pwr.h>
#include <usb_cdc.h>
#include <shell.h>
#include <getstring.h>

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
static char puts_buffer[PRINTF_BUFFER_LENGTH*2];

void puts_(const char *s)
{
  int l = 0;
  char *p = puts_buffer;
  while (*s)
  {
    char c = *s++;
    if (c == '\n')
    {
      *p++ = '\r';
      l++;
    }
    *p++ = c;
    l++;
  }
  CDC_Transmit((const unsigned char*)puts_buffer, l);
}

int main(void)
{
  int counter = 0;

  SysInit();

  led_state = 0;

  shell_init(common_printf);

  getstring_init(command_line, COMMAND_LINE_LENGTH, getch_, puts_);

  USBFS_RCC_Init( );
  USBFS_Device_Init( ENABLE , PWR_VDD_SupplyVoltage());

  TimerEnable();

  while (1)
  {
    asm __volatile__("wfi");
    if (timer_interrupt)
    {
      timer_interrupt = 0;
      if (counter == 99)
      {
        counter = 0;
        led_toggle();
      }
      else
        counter++;
      unsigned int length = CDC_Receive(cdc_rx_buffer, sizeof(cdc_rx_buffer));
      const unsigned char *p = cdc_rx_buffer;
      while (length--)
        shell_process_char(*p++);
      shell_handler();
    }
  }
}
