#include "board.h"
#include "ui.h"
#include <display2.h>
#include <fonts/font12.h>
#include "ina_func.h"

static unsigned int time_minutes;
static unsigned int time_hours;
static unsigned int time_days;
static unsigned int prev_seconds;
static unsigned int total_mah;
static bool high_precision;
static bool recording_started;
static bool log_started;

void DrawChar(unsigned int x, unsigned int y, unsigned int c, unsigned int text_color, unsigned int bk_color)
{
  LcdDrawText(x, y, (const char*)&c, &courierNew12ptFontInfo, text_color, bk_color, nullptr);
}

static void init_row(unsigned int row)
{
  Character c;

  c.x = 0;
  c.y = row * 16;
  c.textColor = WHITE_COLOR;
  c.bkColor = BLACK_COLOR;
  for (unsigned int i = 0; i < DISPLAY_MAX_COLUMNS; i++)
  {
    DisplayInitChar(i, row, &c);
    c.x += courierNew12ptFontInfo.character_max_width + courierNew12ptFontInfo.character_spacing;
  }
}

static void InitCurrentRow(void)
{
  const unsigned int dotX = 3 + (high_precision ? 0 : 1);
  DisplaySetChar(dotX, 0, '.');
  DisplaySetChar(dotX + 4, 0, '.');
  DisplaySetChar(dotX + 8, 0, '.');
  if (!high_precision)
    DisplaySetChar(0, 0, 'L');
}

static void InitVoltageRow(void)
{
  DisplaySetChar(5, 1, '.');
  DisplaySetChar(9, 1, 'V');
}

static void InitTimeRow(void)
{
  DisplaySetChar(1, 1, ':');
  DisplaySetChar(4, 1, ':');
  DisplaySetChar(7, 1, ':');
}

static void InitMahRow(void)
{
  DisplaySetChar(7, 3, 'm');
  DisplaySetChar(8, 3, 'A');
  DisplaySetChar(9, 3, 'h');
}

static void ShowSeconds(unsigned int value)
{
  DisplaySetChar(9, 2, '0' + value % 10);
  DisplaySetChar(8, 2, '0' + value / 10);
}

static void ShowMinutes(unsigned int value)
{
  DisplaySetChar(6, 2, '0' + value % 10);
  DisplaySetChar(5, 2, '0' + value / 10);
}

static void ShowHours(unsigned int value)
{
  DisplaySetChar(3, 2, '0' + value % 10);
  DisplaySetChar(2, 2, '0' + value / 10);
}

static void ShowDays(void)
{
  DisplaySetChar(0, 2, '0' + time_days);
}

static void ShowCurrent(void)
{

}

static void ShowVoltage(void)
{
  unsigned int v = voltage_uv / 1000;
  for (int i = 8; i > 3; i--)
  {
    if (i == 5)
      continue;
    if (v)
    {
      DisplaySetChar(i, 1, '0' + (v % 10));
      v /= 10;
    }
    else
      DisplaySetChar(i, 1, '0');
  }
  if (v)
    DisplaySetChar(3, 1, '0' + v);
  else
    DisplaySetChar(3, 1, ' ');
}

static void ShowMah(void)
{
  unsigned int v = total_mah;
  for (int i = 5; i >= 0; i--)
  {
    if (v)
    {
      DisplaySetChar(i, 3, '0' + (v % 10));
      v /= 10;
    }
    else
      DisplaySetChar(i, 3, ' ');
  }
}

void UI_Init(void)
{
  time_minutes = time_hours = time_days = prev_seconds = total_mah = 0;
  high_precision = false;
  recording_started = false;
  log_started = false;

  DisplayInit();

  LcdInit();
  LcdScreenFill(BLACK_COLOR);

  init_row(0);
  init_row(1);
  init_row(2);
  init_row(3);

  InitCurrentRow();
  InitVoltageRow();
  InitTimeRow();
  InitMahRow();
  ShowSeconds(0);
  ShowMinutes(0);
  ShowHours(0);
  ShowDays();
  ShowCurrent();
  ShowVoltage();
  ShowMah();
}

void Process_Timer_Event(unsigned int keyboard_status)
{
  unsigned int total_seconds = time_since_boot_ms / 1000;
  if (prev_seconds != total_seconds)
  {
    prev_seconds = total_seconds;
    unsigned int prev_minutes = time_minutes;
    unsigned int prev_hours = time_hours;
    unsigned int prev_days = time_days;
    time_minutes = total_seconds / 60;
    time_hours = time_minutes / 60;
    time_days = time_hours / 24;
    ShowSeconds(total_seconds % 60);
    if (time_minutes != prev_minutes)
      ShowMinutes(time_minutes % 60);
    if (time_hours != prev_hours)
      ShowHours(time_hours % 24);
    if (time_days != prev_days)
      ShowDays();
  }

  switch (keyboard_status)
  {
    case 1: // low/high sensitivity change
      high_precision = !high_precision;
      InitCurrentRow();
      break;
    case 2: // start/stop recording
      break;
    case 4: // reset
      time_since_boot_ms = time_minutes = time_hours = time_days = prev_seconds = total_mah = 0;
      ShowSeconds(0);
      ShowMinutes(0);
      ShowHours(0);
      ShowDays();
      ShowMah();
      break;
  }

  ShowCurrent();
  ShowVoltage();

  LcdUpdate();
}
