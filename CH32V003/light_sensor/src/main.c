#include "board.h"
#include <delay.h>
#include <fonts/font12.h>
#include <veml7700.h>
#include <tsl2591.h>

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

#ifdef SENSOR_TSL2591
static const tsl2591_config config = {
  .integration_time_ms = 600,
  .als_interrupt_enable = 0,
  .als_thresholds = {0,0},
  .no_persist_interrupt_enable = 1,
  .no_persist_als_thresholds = {1, 65535},
  .persistence_filter = AnyOutOfRange,
  .sleep_after_interrupt = 0
};
#endif

int main(void)
{
  SysInit();

#ifdef SENSOR_VEML7700
  if (veml7700_init())
  {
    POWER_OFF;
    while (1)
      __WFI();
  }
#endif

#ifdef SENSOR_TSL2591
  if (tsl2591_init(&config))
  {
    POWER_OFF;
    while (1)
      __WFI();
  }
#endif

  LcdInit();

  while (1)
  {
    float lux;
    unsigned short raw;
    unsigned int gain;
    int rc;

#ifdef SENSOR_VEML7700
    veml7700_result veml_result;
    rc = veml7700_measure(&veml_result, 1);
    lux = veml_result.lux;
    raw = veml_result.raw;
    gain = veml_result.gainx8;
#endif
#ifdef SENSOR_TSL2591
    tsl2591_result tsl_result;
    delayms(2000);
    rc = tsl2591_measure(&tsl_result);
    lux = tsl_result.lux;
    raw = tsl_result.rawch0;
    gain = tsl_result.gain;
#endif
    LcdScreenFill(BLACK_COLOR);
    if (rc)
      LcdDrawText(0, 0, "Error", &courierNew12ptFontInfo, WHITE_COLOR, BLACK_COLOR, NULL);
    else
    {
      if (lux >= 1000)
      {
        int luminocity = (int)(lux * 100);
        LcdPrintf("%d.%02d Lx", 0, 0, &courierNew12ptFontInfo, 1, luminocity / 100, luminocity % 100);
      }
      else if (lux >= 100)
      {
        int luminocity = (int)(lux * 1000);
        LcdPrintf("%d.%03d Lx", 0, 0, &courierNew12ptFontInfo, 1, luminocity / 1000, luminocity % 1000);
      }
      else if (lux >= 10)
      {
        int luminocity = (int)(lux * 10000);
        LcdPrintf("%d.%04d Lx", 0, 0, &courierNew12ptFontInfo, 1, luminocity / 10000, luminocity % 10000);
      }
      else
      {
        int luminocity = (int)(lux * 100000);
        LcdPrintf("%d.%05d Lx", 0, 0, &courierNew12ptFontInfo, 1, luminocity / 100000, luminocity % 100000);
      }
    }
    LcdPrintf("G%d R%d", 0, 16, &courierNew12ptFontInfo, 1, gain, raw);
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
