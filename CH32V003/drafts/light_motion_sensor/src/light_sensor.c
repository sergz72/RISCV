#include "light_sensor.h"
#include <veml7700.h>

int light_sensor_init(void)
{
  return veml7700_ex_init(VEML7700_GAIN_2|VEML7700_IT_100ms, VEML7700_PSM_MODE4|VEML7700_PSM_ENABLE);
}

int light_sensor_read(unsigned short *result)
{
  return veml7700_read(VEML7700_REG_ALS, result);
}