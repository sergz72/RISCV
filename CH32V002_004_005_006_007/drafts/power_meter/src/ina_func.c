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

int ina_read(void)
{
  return 1;
}