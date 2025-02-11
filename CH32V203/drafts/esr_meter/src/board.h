#ifndef _BOARD_H
#define _BOARD_H

#ifndef NULL
#define NULL 0
#endif

#include <ch32v20x.h>

#define SSD1306_128_32
#define LCD_ORIENTATION LCD_ORIENTATION_LANDSCAPE

#include <lcd_ssd1306.h>

#define LCD_PRINTF_BUFFER_LENGTH 30
#define DRAW_TEXT_MAX 20
#define USE_VSNPRINTF

#define I2C_TIMEOUT 1000000

/*
 *PB13 = PWM (TIM1_CH1N)
 */
#define PWM_PIN GPIO_Pin_13
#define PWM_PORT GPIOB
#define PWM_PERIOD 480
#define PWM_PULSE 48
#define PWM_OCINIT TIM_OC1Init
#define PWM_OCPRELOADCONFIG TIM_OC1PreloadConfig

/*
 *PB2 = LED
 */
#define LED_TIMER_PIN GPIO_Pin_2
#define LED_TIMER_PORT GPIOB
#define LED_TIMER_ON GPIOD->BCR = LED_TIMER_PIN
#define LED_TIMER_OFF GPIOD->BSHR = LED_TIMER_PIN

/*
 *PA2 = OPA2_OUT0
 *PA5 = OPA2_CH1N
 *PA7 = OPA2_CH1P
 */
#define OPA2_OUT_PIN GPIO_Pin_2
#define OPA2_OUT_PORT GPIOA
#define OPA2_N_PIN GPIO_Pin_5
#define OPA2_N_PORT GPIOA
#define OPA2_P_PIN GPIO_Pin_7
#define OPA2_P_PORT GPIOA

/*
 *PA3 = OPA1_OUT0
 *PA6 = OPA1_CH1N
 *PB0 = OPA1_CH1P
 */
#define OPA1_OUT_PIN GPIO_Pin_3
#define OPA1_OUT_PORT GPIOA
#define OPA1_N_PIN GPIO_Pin_6
#define OPA1_N_PORT GPIOA
#define OPA1_P_PIN GPIO_Pin_0
#define OPA1_P_PORT GPIOB

#define MCP3426_I2C_INSTANCE I2C1
#define SSD1306_I2C_INSTANCE I2C2

void SysInit(void);

#endif
