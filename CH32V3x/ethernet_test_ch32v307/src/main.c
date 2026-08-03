#include "board.h"
#include "debug.h"
#include <shell.h>
#include <getstring.h>
#include <eth_driver.h>
#include <eth_ntp.h>
#include <eth.h>
#include <eth_queue.h>
#include <common_printf.h>
#include "ethernet_commands.h"

static const unsigned char ntp_server_address[16] = {
  0x2a, 0x00, 0x8a, 0x60, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x23
};

unsigned char rx_buffer[RX_BUF_LEN];
unsigned char *rx_buffer_write_p, *rx_buffer_read_p;
static char command_line[200];
static int led_state;


static int getch_(void)
{
  if (rx_buffer_write_p != rx_buffer_read_p)
  {
    char c = (char)*rx_buffer_read_p++;
    if (rx_buffer_read_p == rx_buffer + RX_BUF_LEN)
      rx_buffer_read_p = rx_buffer;
    return c;
  }
  return EOF;
}

int putss(const char *s)
{
  puts_(s);
  puts_("\r\n");
  return 0;
}

/*static void LEDTimerToggle(void)
{
  led_state = !led_state;
  if (led_state)
    LED_BLUE_ON;
  else
    LED_BLUE_OFF;
}*/

void eth_set_prefix_callback(void)
{
  if (!ntp_time_is_set)
    ETH_NTP_Send_Timestamp_Request(eth_instance.ntp_server_address, &eth_irq_queue);
  LED_BLUE_ON;
}

void ntp_time_received_callback(unsigned int unix_time)
{
  if (eth_instance.log_level >= ETH_LOGLEVEL_INFO)
    ETH_Printf("Unix time from a NTP server is %u\n", unix_time);
  LED_RED_ON;
}

int main(void)
{
  int rc;

  led_state = 0;

  rx_buffer_write_p = rx_buffer_read_p = rx_buffer;

  HalInit(ntp_server_address);

  shell_init(common_printf, nullptr);
  register_ethernet_commands();

  getstring_init(command_line, sizeof(command_line), getch_, puts_);

  eth_instance.puts_func = putss;
  eth_instance.log_level = ETH_LOGLEVEL_INFO;

  while (1)
  {
    /*Ethernet library main task function,
     * which needs to be called cyclically*/
    WCHNET_MainTask();
    ETH_Handler();

    if (timer_interrupt)
    {
      timer_interrupt = 0;
      //if ((timeCnt & 32) == 0)
      //  LEDTimerToggle();
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
            common_printf("shell_execute returned %d\n$ ", rc);
          break;
        }
      }
    }
  }
}
