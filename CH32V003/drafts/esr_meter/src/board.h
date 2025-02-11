#ifndef _BOARD_H
#define _BOARD_H

#ifndef NULL
#define NULL 0
#endif

#include <ch32v00x.h>

#define SSD1306_128_32
#define LCD_ORIENTATION LCD_ORIENTATION_LANDSCAPE

#include <lcd_ssd1306.h>

#define LCD_PRINTF_BUFFER_LENGTH 30
#define DRAW_TEXT_MAX 20
#define USE_MYVSPRINTF

#define I2C_TIMEOUT 1000000

/*
 *PC4 = PWM
 */
#define PWM_PIN GPIO_Pin_4
#define PWM_PORT GPIOC
#define PWM_PERIOD 80
#define PWM_PULSE 8
#define PWM_OCINIT TIM_OC4Init
#define PWM_OCPRELOADCONFIG TIM_OC4PreloadConfig

/*
 *PD6 = LED
 */
#define LED_TIMER_PIN GPIO_Pin_6
#define LED_TIMER_PORT GPIOD
#define LED_TIMER_ON GPIOD->BCR = LED_TIMER_PIN
#define LED_TIMER_OFF GPIOD->BSHR = LED_TIMER_PIN

void SysInit(void);

#endif
