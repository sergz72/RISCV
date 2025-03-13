#include "board.h"
#include "dev_internal_pwm.h"
#include <dev_pwm.h>
#include <stdlib.h>
#include <string.h>

static const PWMConfig config =
{
  .bits = 16,
  .channels = 2,
  .dds_clock = 0,
  .mclk = PWM_CLOCK
};

static int pwm_enable_output(DeviceObject *o, int channel, int enable);

static int pwm_set_frequency_duty(DeviceObject *o, int channel, unsigned int frequency, unsigned int duty);

static int pwm_enable_output(DeviceObject *o, int channel, int enable)
{
  return pwm_enable(o->idx, channel, enable);
}

static int pwm_set_frequency_duty(DeviceObject *o, int channel, unsigned int frequency, unsigned int duty)
{
  return pwm_set_frequency_and_duty(o->idx, channel, frequency, duty);
}

void internal_pwm_initializer(DeviceObject *o)
{
  //todo
  dev_pwm *dev = malloc(sizeof(dev_pwm));
  if (!dev)
    return;
  pwm_initializer();
  dev->enable_output = pwm_enable_output;
  dev->set_frequency_and_duty = pwm_set_frequency_duty;
  memcpy(&dev->cfg, &config, sizeof(PWMConfig));
  o->device_config = dev;
}

int internal_pwm_save_config(DeviceObject *o, void *buffer)
{
  return BuildPWMConfig(buffer, &config, "Internal PWM");
}

int PWMAllowed(int module_id)
{
  return module_id == 1 || module_id == 3 || module_id == 4;
}
