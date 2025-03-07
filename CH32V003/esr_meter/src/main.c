#include "board.h"
#include <mcp3426.h>
#include <delay.h>
#include <fonts/font18.h>

#include "lcd_ks0108_buffered.h"

#define MCP3426_DEVICE_ID 0xD6
#define VALUE_MAX 999999
#define VREF 256000 // uV
#define R 470000 //uOhm

static int led_timer_state;
static int r0;

static const MCP3426Config dcfg1 = {
  .channel = MCP3426_CHANNEL_1,
  .start_conversion = 1,
  .continuous_conversion = 0,
  .sample_rate = MCP3426_RATE_15,
  .gain = MCP3426_GAIN_4
};

static const MCP3426Config dcfg2 = {
  .channel = MCP3426_CHANNEL_2,
  .start_conversion = 1,
  .continuous_conversion = 0,
  .sample_rate = MCP3426_RATE_15,
  .gain = MCP3426_GAIN_4
};

static void led_toggle(void)
{
  led_timer_state = !led_timer_state;
  if (led_timer_state)
    LED_TIMER_ON;
  else
    LED_TIMER_OFF;
}

static int adc_get(const MCP3426Config *dcfg)
{
  int v;

  if (mcp3426SetConfig(0, MCP3426_DEVICE_ID, dcfg))
    return 0;
  Delay_Ms(125);
  if (mcp3426GetVoltage(0, MCP3426_DEVICE_ID, &v) || v <= 0)
    return 0;
  int voltage = (int)((long long int)v * VREF / 0xFFFF);
  return voltage;
}

//vt is on transformer out
//vr is on RLoad
static int calculate_value(int vt, int vr)
{
  int vcx = vt - vr;
  int r = (int)((long long int)vcx * R / vr);
  return r > VALUE_MAX ? VALUE_MAX : r;
}

static void show_result(int value)
{
  LcdPrintf("%3d.%03d", 0, 4, &courierNew18ptFontInfo, 1, value / 1000, value % 1000);
  LcdUpdate();
}

int main(void)
{
  led_timer_state = 0;
  r0 = 0;

  SysInit();

  LcdInit();
  LcdScreenFill(BLACK_COLOR);

  while (1)
  {
    led_toggle();
    Delay_Ms(250);
    int vt = adc_get(&dcfg1); // transformer out
    int vr = adc_get(&dcfg2); // transformer out + rc
    int value;
    if (vr >= vt)
      value = 0;
    else
      value = calculate_value(vt, vr);
    if (BUTTON_PRESSED && vr > 0)
      r0 = value;
    show_result(value - r0);
  }
}
