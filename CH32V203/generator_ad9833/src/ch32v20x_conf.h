#ifndef __CH32V20X_CONF_H
#define __CH32V20X_CONF_H

#define INTERFACE_I2C

#ifdef INTERFACE_I2C
#define SYSCLK_FREQ_48MHz_HSE  48000000
#else
#define SYSCLK_FREQ_144MHz_HSE  144000000
#endif

#include "ch32v20x_gpio.h"
#include "ch32v20x_rcc.h"
#include "ch32v20x_misc.h"
#include "ch32v20x_tim.h"
#include "ch32v20x_adc.h"
#include "ch32v20x_i2c.h"
#include "ch32v20x_spi.h"

#endif
