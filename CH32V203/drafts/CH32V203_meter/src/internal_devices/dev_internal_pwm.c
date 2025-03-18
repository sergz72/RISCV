#include "board.h"
#include "dev_internal_pwm.h"
#include <dev_pwm.h>
#include <stdlib.h>
#include <string.h>
#include "ch32v20x_gpio.h"
#include "ch32v20x_tim.h"

static const PWMConfig config =
{
  .bits = 16,
  .prescaler_bits = 16,
  .channels = 1,
  .dds_clock = 0,
  .mclk = PWM_CLOCK
};

static int pwm_enable_output(DeviceObject *o, int channel, int enable)
{
  return pwm_enable(o->idx, channel, enable);
}

static int pwm_set_frequency_duty(DeviceObject *o, int channel, unsigned short prescaler, unsigned int frequency, unsigned int duty)
{
  return pwm_set_frequency_and_duty(o->idx, channel, prescaler, frequency, duty);
}

void internal_pwm_initializer(DeviceObject *o)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  switch (o->idx)
  {
    case 0: // module 1
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      TIM_Cmd(TIM3, ENABLE);
      break;
    case 2:
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      TIM_Cmd(TIM4, ENABLE);
      break;
    default:
      return;
  }

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
  return module_id == 0 || module_id == 2;
}
