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

  sensor_handler_init();

  TIM_Cmd( TIM_TIMER, ENABLE );

  while (1)
  {
    asm("wfi");
    if (timer_interrupt)
    {
      timer_interrupt = false;
      //pir_sensor_adc_handler(adc_get());
#ifdef USART_ENABLED
      usart_handler();
#endif
      if (sensor_handler())
        TIM_Cmd( TIM_TIMER, DISABLE );
    }
  }
}
