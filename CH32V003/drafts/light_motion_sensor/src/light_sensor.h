#ifndef LIGHT_MOTION_SENSOR_LIGHT_SENSOR_H
#define LIGHT_MOTION_SENSOR_LIGHT_SENSOR_H

#define LIGHT_SENSOR_LUX_X100(v) ((unsigned int)v * 336 / 100)
#define VEML7700_LUX_TO_V(l) (l * 30)
#define LIGHT_SENSOR_HIGH_THRESHOLD VEML7700_LUX_TO_V(300) // 3 lux / 0.0336
#define LIGHT_SENSOR_LOW_THRESHOLD  VEML7700_LUX_TO_V(100) // 1 lux / 0.0336

int light_sensor_init(void);
int light_sensor_read(unsigned short *result);

#endif
