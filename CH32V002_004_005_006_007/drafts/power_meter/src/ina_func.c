#include "board.h"
#include "ina_func.h"
#include <ina228.h>

static const INA228Config cfg1 = {
  .bits = {
    .adcrange = 1,
    .tempcomp = 0,
    .convdly = 0,
    .rstacc = 0,
    .rst = 0,
    .reserved = 0
  }
};

static const INA228Config cfg0 = {
  .bits = {
    .adcrange = 0,
    .tempcomp = 0,
    .convdly = 0,
    .rstacc = 0,
    .rst = 0,
    .reserved = 0
  }
};

unsigned int current_ua, voltage_uv;

int ina_init(void)
{
  current_ua = voltage_uv = 0;
  return ina228SetConfig(0, INA_ADDRESS, cfg0);
}

int ina_set_high_precision(void)
{
  return ina228SetConfig(0, INA_ADDRESS, cfg0);
}

int ina_set_low_precision(void)
{
  return ina228SetConfig(0, INA_ADDRESS, cfg1);
}

int ina_read(void)
{
  int value;
  int rc = ina228GetBusVoltage(0, INA_ADDRESS, &value);
  if (rc)
    return rc;
  voltage_uv = value > 0 ? value : 0;
  rc = ina228GetShuntCurrent(0, INA_ADDRESS, 1000, &value);
  if (rc)
    return rc;
  current_ua = value > 0 ? value : 0;
  return 0;
}