#include "board.h"
#include "usart_handler.h"
#include "sensor_handler.h"

int main(void)
{
#ifdef USART_ENABLED
  usart_handler_init();
#endif

  SysInit();

  int rc = light_sensor_init();
  if (rc)
  {
    LED_BATTERY_ON;
    while (1)
      ;
  }

#ifdef SENSOR_ENABLED
  sensor_handler_init();
#endif

  TIM_Cmd( TIM_TIMER, ENABLE );
#ifndef USART_ENABLED
  iwdg_init();
#endif

  while (1)
  {
    asm("wfi");
    if (timer_interrupt)
    {
      timer_interrupt = false;
#ifdef USART_ENABLED
      usart_handler();
#else
      IWDG_ReloadCounter();
#endif
#ifdef SENSOR_ENABLED
      sensor_handler();
#endif
    }
  }
}
