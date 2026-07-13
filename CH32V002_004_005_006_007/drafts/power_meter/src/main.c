#include "board.h"
#include <stdio.h>
#include <fonts/font12.h>
#include "usart_handler.h"

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
  int counter = 0;

  SysInit();

  LcdInit();
  LcdScreenFill(BLACK_COLOR);
  LcdDrawText(0, 0, "testtest10", &courierNew12ptFontInfo, WHITE_COLOR, BLACK_COLOR, nullptr);
  LcdDrawText(0, 16, "testtest10", &courierNew12ptFontInfo, WHITE_COLOR, BLACK_COLOR, nullptr);
  LcdDrawText(0, 32, "testtest10", &courierNew12ptFontInfo, WHITE_COLOR, BLACK_COLOR, nullptr);
  LcdDrawText(0, 48, "testtest10", &courierNew12ptFontInfo, WHITE_COLOR, BLACK_COLOR, nullptr);
  LcdUpdate();

  TimerEnable();

  while (1)
  {
    __WFI();
    if (timer_interrupt)
    {
      if (counter == 9)
      {
        led_toggle();
        counter = 0;
      }
      else
        counter++;
      usart_handler();
      timer_interrupt = 0;
    }
  }
}
