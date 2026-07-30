#include "board.h"
#include "debug.h"
#include "ch32v30x_usbfs_device.h"
#include "delay.h"
#include <usb_cdc.h>
#include <stdarg.h>
#include <shell.h>
#include <getstring.h>
#include <eth_driver.h>
#include "eth_queue.h"
#include <queue.h>

static unsigned char usb_cdc_buffer[USB_CDC_RX_BUFFER_SIZE];
static char command_line[200];
static volatile unsigned int cdc_length;
static unsigned char *cdc_buffer_p;
static int led_state;

void puts_(const char *s)
{
  CDC_Transmit((unsigned char*)s, strlen(s));
}

int usb_printf(const char *format, ...)
{
  static char buffer[PRINTF_BUFFER_LENGTH], buffer2[PRINTF_BUFFER_LENGTH];
  char *p, *p2;
  va_list vArgs;
  int rc;

  va_start(vArgs, format);
  rc = vsnprintf(buffer, sizeof(buffer), format, vArgs);
  va_end(vArgs);
  p = buffer;
  p2 = buffer2;
  while (*p)
  {
    if (*p == '\n')
      *p2++ = '\r';
    *p2++ = *p++;
  }
  *p2 = 0;
  puts_(buffer2);
  return rc;
}

static int getch_(void)
{
  if (!cdc_length)
    return EOF;
  cdc_length--;
  return *cdc_buffer_p++;
}

static void LEDTimerToggle(void)
{
  led_state = !led_state;
  if (led_state)
    LED_BLUE_ON;
  else
    LED_BLUE_OFF;
}

int main(void)
{
  int rc;

  led_state = 0;

  HalInit();

  USBFS_RCC_Init( );
  USBFS_Device_Init( ENABLE );

  shell_init(usb_printf, NULL);

  getstring_init(command_line, sizeof(command_line), getch_, puts_);

  while (1)
  {
    /*Ethernet library main task function,
     * which needs to be called cyclically*/
    WCHNET_MainTask();

    unsigned int message_size;
    unsigned char *data = ETH_IRQ_QueuePoll(&message_size);
    if (data)
    {
      usb_printf("ethernet packet with length %d has been received.\n$ ", message_size);
    }

    if (timer_interrupt)
    {
      timer_interrupt = 0;
      if ((timeCnt & 32) == 0)
        LEDTimerToggle();
      cdc_length = CDC_Receive(usb_cdc_buffer, sizeof(usb_cdc_buffer));
      cdc_buffer_p = usb_cdc_buffer;
      while (cdc_length)
      {
        if (!getstring_next())
        {
          switch (command_line[0])
          {
          case SHELL_UP_KEY:
            puts_("\r\33[2K$ ");
            getstring_buffer_init(shell_get_prev_from_history());
            break;
          case SHELL_DOWN_KEY:
            puts_("\r\33[2K$ ");
            getstring_buffer_init(shell_get_next_from_history());
            break;
          default:
            rc = shell_execute(command_line);
            if (rc == 0)
              puts_("OK\r\n$ ");
            else if (rc < 0)
              puts_("Invalid command line\r\n$ ");
            else
              usb_printf("shell_execute returned %d\n$ ", rc);
            break;
          }
        }
      }
    }
  }
}
