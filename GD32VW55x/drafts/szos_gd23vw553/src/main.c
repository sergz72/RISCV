#include "board.h"
#include <shell.h>
#include <getstring.h>
//#include <stdio.h>
#include <common_printf.h>
#include "system_commands.h"
#include "fs_commands.h"
#include "sys_timer.h"
#include "fs.h"
#include <trng_commands.h>
#include <crc_commands.h>

unsigned char rx_buffer[RX_BUF_LEN];
unsigned char *rx_buffer_write_p, *rx_buffer_read_p;
static char command_line[200];

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

void puts_(const char *s);

int main(void)
{
  int rc;
  bool led_state = false;

  rx_buffer_write_p = rx_buffer_read_p = rx_buffer;

  HalInit();

  if (fs_init())
  {
    LED_TIMER_ON;
    while (1)
      __WFI();
  }

  shell_init(PRINTF, nullptr);
  register_system_commands();
  register_fs_commands();
  //register_trng_commands();
  register_crc_commands();

  getstring_init(command_line, sizeof(command_line), getch_, puts_);

  while(1)
  {
    delayms(100);
    led_state = !led_state;
    if (led_state)
      LED_TIMER_ON;
    else
      LED_TIMER_OFF;
    if (!getstring_next())
    {
      switch (command_line[0])
      {
      case SHELL_UP_KEY:
        PUTS("\r\33[2K$ ");
        getstring_buffer_init(shell_get_prev_from_history());
        break;
      case SHELL_DOWN_KEY:
        PUTS("\r\33[2K$ ");
        getstring_buffer_init(shell_get_next_from_history());
        break;
      default:
        rc = shell_execute(command_line);
        if (rc == 0)
          PUTS("OK\n");
        else if (rc < 0)
          PUTS("Invalid command line\n");
        else if (rc != INT32_MAX)
          PRINTF("shell_execute returned %d\n", rc);
        if (getstring_get_echo())
          PUTS("$ ");
        break;
      }
    }
  }
}
