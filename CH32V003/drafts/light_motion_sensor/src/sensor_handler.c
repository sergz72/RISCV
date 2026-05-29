#include "board.h"
#include "sensor_handler.h"
#include <pir_sensor.h>

static volatile int motion_timer;
static volatile bool motion_detected;
static bool motion_sensor_active;
static unsigned int light_sensor_disable_time;
static unsigned int motion_sensor_disable_time;
static bool vbat_alert_active;

void pir_motion_detected(void)
{
  motion_detected = true;
}

void sensor_handler_init(void)
{
  motion_sensor_active = false;
  light_sensor_disable_time = 0;
  motion_sensor_disable_time = 0;
  motion_timer = 0;
  motion_detected = false;
  vbat_alert_active = false;
}

int sensor_handler(void)
{
  if (vbat_alert_active)
    return 1;

  if (motion_sensor_active)
    pir_sensor_adc_handler(adc_get());

  if (motion_timer)
  {
    motion_timer--;
    if (!motion_timer)
    {
      pwm_off();
      light_sensor_disable_time = 5;
    }
  }
  else if (light_sensor_disable_time)
    light_sensor_disable_time--;
  else
  {
    unsigned short result;
    int rc = light_sensor_read(&result);
    if (rc || result < LIGHT_SENSOR_LOW_THRESHOLD_RAW)
    {
      if (!motion_sensor_active)
      {
        motion_sensor_active = true;
        enable_adc();
        motion_sensor_disable_time = 20;
      }
    }
    else if (result > LIGHT_SENSOR_HIGH_THRESHOLD_RAW)
    {
      if (motion_sensor_active)
      {
        motion_sensor_active = false;
        disable_adc();
        motion_detected = false;
      }
    }
  }

  if (motion_sensor_disable_time)
  {
    motion_sensor_disable_time--;
    motion_detected = false;
  }
  else if (motion_sensor_active & motion_detected)
  {
    motion_detected = false;
    if (motion_timer)
      motion_timer = MOTION_DETECTOR_ON_TIME;
    else
    {
      unsigned int vbat = get_vbat();
      if (VBAT_BELOW_2V9(vbat))
      {
        disable_adc();
        motion_sensor_active = false;
        motion_detected = false;
        vbat_alert_active = true;
        motion_timer = 0;
        LED_BATTERY_ON;
      }
      else
      {
        motion_timer = MOTION_DETECTOR_ON_TIME;
        if (VBAT_BELOW_3V0(vbat))
          vbat = VBAT_3V0;
        pwm_on(VBAT_TO_DUTY(vbat));
      }
    }
  }
  return 0;
}
