#include "board.h"
#include <systick.h>

void eclic_mtip_handler(void)
{
  ECLIC_ClearPendingIRQ(CLIC_INT_TMR);
  delay_decrement();
}

int main(void)
{
  systick_config();

  /* enable the LED clock */
  rcu_periph_clock_enable(LED_TIMER_CLOCK);
  /* configure LED GPIO port */
  gpio_mode_set(LED_TIMER_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_TIMER_PIN);

  while(1)
  {
    LED_TIMER_ON;
    delay_1ms(1000);
    LED_TIMER_OFF;
    delay_1ms(1000);
  }
}
