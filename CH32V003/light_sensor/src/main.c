#include "board.h"
#include <delay.h>
#include <fonts/font18.h>
#include <lcd_ks0108_buffered.h>
#include <veml7700.h>

int main(void)
{
  SysInit();

#ifdef LED_DEBUG
  LED2_ON;
#endif

  if (veml7700_init())
  {
    POWER_OFF;
    while (1)
      __WFI();
  }

#ifdef LED_DEBUG
  LED3_ON;
#endif

  LcdInit();
  LcdScreenFill(BLACK_COLOR);

#ifdef LED_DEBUG
  LED4_ON;
#endif

  while (1)
  {
    veml7700_result result;

    int rc = veml7700_measure(&result);
#ifdef LED_DEBUG
    LED5_ON;
#endif
    if (rc)
      LcdDrawText(0, 4, "Error", &courierNew18ptFontInfo, WHITE_COLOR, BLACK_COLOR, NULL);
    else
    {
      int luminocity = (int)(result.lux * 1000);
      LcdPrintf("%3d.%03d", 0, 4, &courierNew18ptFontInfo, 1, luminocity / 1000, luminocity % 1000);
    }
    LcdUpdate();
    LcdOn();
#ifdef LED_DEBUG
    LED6_ON;
#endif
    Delay_Ms(2000);
    if (!BUTTON_PRESSED)
    {
      POWER_OFF;
      while (1)
        __WFI();
    }
    LcdOff();
    Delay_Ms(2000);
#ifdef LED_DEBUG
    LED5_OFF;
    LED6_OFF;
#endif
  }
}
