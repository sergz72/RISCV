#ifndef POWER_METER_INA_FUNC_H
#define POWER_METER_INA_FUNC_H

int ina_init(void);
int ina_read(void);

extern unsigned int current_ua, voltage_uv;

#endif
