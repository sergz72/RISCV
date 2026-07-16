#ifndef POWER_METER_INA_FUNC_H
#define POWER_METER_INA_FUNC_H

int ina_init(void);
int ina_read(void);
int ina_set_high_precision(void);
int ina_set_low_precision(void);

extern unsigned int current_ua, voltage_uv;

#endif
