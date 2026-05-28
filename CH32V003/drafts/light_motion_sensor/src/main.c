#include "board.h"
#include <pir_sensor.h>
#include "usart_handler.h"

static volatile int motion_timer;
static volatile bool motion_detected;
static bool motion_sensor_active;
static unsigned int light_sensor_disable_time;
static unsigned int motion_sensor_disable_time;

void pir_motion_detected(void)
{
  motion_detected = true;
}

int main(void)
{
#ifdef USART_ENABLED
  usart_handler_init();
#endif

  SysInit();

  //set_high_system_clock();

  enable_opa();
  enable_adc();
  enable_i2c();
  //enable_pwm(10);

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
    }
  }
}
