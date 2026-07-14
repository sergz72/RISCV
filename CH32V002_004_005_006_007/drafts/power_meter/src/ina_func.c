#include "board.h"
#include "ina_func.h"
#include <ina228.h>

static const INA228Config cfg_high = {
  .bits = {
    .adcrange = 1,
    .tempcomp = 0,
    .convdly = 0,
    .rstacc = 0,
    .rst = 0,
    .reserved = 0
  }
};

static const INA228Config cfg_low = {
  .bits = {
    .adcrange = 0,
    .tempcomp = 0,
    .convdly = 0,
    .rstacc = 0,
    .rst = 0,
    .reserved = 0
  }
};

static const INA228AdcConfig adc_cfg = {
  .bits = {
    .awg = INA228_AVG_1024,
    .mode = INA228_MODE_SHUNT_BUS_CONT,
    .vbusct = INA228_CT_540,
    .vshct = INA228_CT_540,
    .vtct = INA228_CT_540
  }
};

static const INA228DiagAlert alert_cfg = {
  .bits = {
    .alatch = 1, //Alert Flag bit remain active following a fault until the DIAG_ALRT Register has been read
    .cnvr = 1, // Alert pin to be asserted when the Conversion Ready Flag (bit 1) is asserted, indicating that a conversion cycle has completed.
    .showalert = 0,
    .apol = 0, // alert is active-low (open-drain)
    .energyof = 0,
    .chargeof = 0,
    .mathof = 0,
    .reserved = 0,
    .tmpol = 0,
    .shuntol = 0,
    .shuntul = 0,
    .busol = 0,
    .busul = 0,
    .pol = 0,
    .cnvrf = 0,
    .memstat = 1 // normal operation
  }
};

unsigned int current_ua, voltage_uv;

int ina_init(void)
{
  current_ua = voltage_uv = 0;
  int rc = ina228SetConfig(0, INA_ADDRESS, cfg_low);
  if (rc)
    return rc;
  rc = ina228SetAdcConfig(0, INA_ADDRESS, adc_cfg);
  if (rc)
    return rc;
  return ina228SetDiagAlertConfig(0, INA_ADDRESS, alert_cfg);
}

int ina_set_high_precision(void)
{
  return ina228SetConfig(0, INA_ADDRESS, cfg_high);
}

int ina_set_low_precision(void)
{
  return ina228SetConfig(0, INA_ADDRESS, cfg_low);
}

int ina_read(void)
{
  INA228DiagAlert alerts;
  int rc = ina228GetDiagAlert(0, INA_ADDRESS, &alerts);
  if (rc)
    return rc;
  int value;
  rc = ina228GetBusVoltage(0, INA_ADDRESS, &value);
  if (rc)
    return rc;
  voltage_uv = value > 0 ? value : 0;
  rc = ina228GetShuntCurrent(0, INA_ADDRESS, 1000, &value);
  if (rc)
    return rc;
  current_ua = value > 0 ? value : 0;
  return 0;
}