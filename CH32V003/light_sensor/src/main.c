#include "board.h"
#include <delay.h>
#include <fonts/font18.h>
#include <veml7700.h>

//                         CH32V003
//                     -------------------
//                     |                 |
//      VEML7700->[SDA]|1              16|[VSS]
//                     |                 |
//      VEML7700->[SCL]|2              15|[VDD]
//                     |                 |
//      POWER_ON->[PC3]|3              14|[VSS]
//                     |                 |
//        BUTTON->[PC4]|4              13|[PA2]
//                     |                 |
//                [PC6]|5              12|[PA1]
//                     |                 |
//                [PC7]|6              11|[PD7]
//                     |                 |
//               [SWIO]|7              10|[PD6]
//                     |                 |
//                [PD4]|8               9|[PD5]
//                     |                 |
//                     -------------------

int main(void)
{
  SysInit();

  if (veml7700_init())
  {
    POWER_OFF;
    while (1)
      __WFI();
  }

  LcdInit();
  LcdScreenFill(BLACK_COLOR);

  while (1)
  {
    veml7700_result result;

    int rc = veml7700_measure(&result, 1);
    if (rc)
      LcdDrawText(0, 4, "Error", &courierNew18ptFontInfo, WHITE_COLOR, BLACK_COLOR, NULL);
    else
    {
      if (result.lux >= 1000)
      {
        int luminocity = (int)(result.lux * 100);
        LcdPrintf("%d.%02d", 0, 4, &courierNew18ptFontInfo, 1, luminocity / 100, luminocity % 100);
      }
      else if (result.lux >= 100)
      {
        int luminocity = (int)(result.lux * 1000);
        LcdPrintf("%d.%03d", 0, 4, &courierNew18ptFontInfo, 1, luminocity / 1000, luminocity % 1000);
      }
      else
      {
        int luminocity = (int)(result.lux * 10000);
        LcdPrintf("%d.%04d", 0, 4, &courierNew18ptFontInfo, 1, luminocity / 10000, luminocity % 10000);
      }
    }
    LcdUpdate();
    LcdOn();
    Delay_Ms(2000);
    if (!BUTTON_PRESSED)
    {
      POWER_OFF;
      while (1)
        __WFI();
    }
    LcdOff();
  }
}
