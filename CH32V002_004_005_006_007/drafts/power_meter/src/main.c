#include "board.h"
#include <stdio.h>
#include "usart_handler.h"
#include "ina_func.h"
#include "ui.h"
#include <spi_memory.h>

static bool led_timer_state;

static void led_toggle(void)
{
  led_timer_state = !led_timer_state;
  if (led_timer_state)
    LED_TIMER_ON;
  else
    LED_TIMER_OFF;
}

int main(void)
{
  SysInit();

  if (ina_init())
  {
    LED_TIMER_ON;
    while (1)
      __WFI();
  }

  spi_memory_init();

  TimerEnable();

  while (1)
  {
    __WFI();
    if (timer_interrupt)
    {
      if (CHECK_ALERT)
      {
        if (!ina_read())
        {
          unsigned int keyboard_status = get_keyboard_status();
          Process_Timer_Event(keyboard_status);
          led_toggle();
        }
      }
      usart_handler();
      timer_interrupt = false;
    }
  }
}
