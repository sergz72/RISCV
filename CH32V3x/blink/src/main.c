#include "board.h"
#include "delay.h"

int main(void)
{
  bool led_state = false;

  HalInit();

  while (1)
  {
    led_state = !led_state;
    if (led_state)
    {
      LED_BLUE_ON;
      LED_RED_OFF;
    }
    else
    {
      LED_BLUE_OFF;
      LED_RED_ON;
    }
    Delay_Ms(500);
  }
}
