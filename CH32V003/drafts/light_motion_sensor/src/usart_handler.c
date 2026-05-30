#include "board.h"
#include "usart_handler.h"
#include <string.h>
#include <common_printf.h>
#include <stdlib.h>

#include "light_sensor_veml7700.h"

#ifdef USART_ENABLED
static bool led_timer_state;
static int led_counter;

void usart_handler_init(void)
{
  led_timer_state = 0;
  led_counter = 0;
}

static void led_toggle(void)
{
  led_timer_state = !led_timer_state;
  if (led_timer_state)
    LED_TIMER_ON;
  else
    LED_TIMER_OFF;
}

static void run_command(void)
{
  size_t l = strlen((const char*)command_line);
  switch (l)
  {
  case 3:
    if (!strcmp((const char*)command_line, "adc"))
    {
      enable_adc();
      unsigned int opa_value = adc_get();
      unsigned int vbat_value = get_vbat();
      disable_adc();
      common_printf("adc result=%d vbat=%d\r\n", opa_value, vbat_value);
      return;
    }
    break;
  case 6:
    if (!strcmp((const char*)command_line, "pwm on"))
    {
      pwm_on(0);
      return;
    }
    break;
  case 7:
    if (!strcmp((const char*)command_line, "pwm off"))
    {
      pwm_off();
      return;
    }
    break;
  case 8:
    if (!strcmp((const char*)command_line, "pwm auto"))
    {
      enable_adc();
      pwm_auto(get_vbat());
      disable_adc();
      return;
    }
    if (!strcmp((const char*)command_line, "veml_get"))
    {
      unsigned short raw;
      int rc = light_sensor_read(&raw);
      unsigned int lux = LIGHT_SENSOR_LUX_X100(raw);
      common_printf("light sensor read rc=%d lux=%d.%02d\r\n", rc, lux / 100, lux % 100);
      return;
    }
    break;
  case 9:
    if (!strcmp((const char*)command_line, "veml_init"))
    {
      common_printf("light sensor init rc=%d\r\n", light_sensor_init());
      return;
    }
    break;
  default:
    break;
  }
  if (l > 4 && !strncmp((const char*)command_line, "pwm ", 4))
  {
    unsigned int duty = atoi((const char*)command_line + 4);
    duty = pwm_set_duty(duty);
    common_printf("pwm duty has been set to %d\r\n", duty);
    return;
  }
  puts_("unknown command\r\n");
}

static void usart_echo(void)
{
  while (command_line_echo_p != command_line_p)
    usart_transmit(*command_line_echo_p++);
}

void usart_handler(void)
{
  if (led_counter == TIMER_EVENT_FREQUENCY - 1)
  {
    led_counter = 0;
    led_toggle();
  }
  else
    led_counter++;
  usart_echo();
  if (command_ready)
  {
    puts_("\r\n");
    *command_line_p = 0;
    run_command();
    command_line_p = command_line;
    command_line_echo_p = command_line;
    command_ready = false;
  }
}
#endif
