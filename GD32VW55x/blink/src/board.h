#ifndef _BOARD_H
#define _BOARD_H

#include <gd32vw55x.h>

#define LED_TIMER_CLOCK RCU_GPIOC
#define LED_TIMER_PORT GPIOC
#define LED_TIMER_PIN GPIO_PIN_13
#define LED_TIMER_ON gpio_bit_set(LED_TIMER_PORT,LED_TIMER_PIN)
#define LED_TIMER_OFF gpio_bit_reset(LED_TIMER_PORT,LED_TIMER_PIN)

#endif
