#ifndef _BOARD_H
#define _BOARD_H

#ifndef NULL
#define NULL 0
#endif

#include <ch32v00x.h>

#define SSD1306_128_32
#define LCD_ORIENTATION LCD_ORIENTATION_LANDSCAPE
#define SSD1306_INIT_OFF

#include <lcd_ssd1306.h>

#define LCD_PRINTF_BUFFER_LENGTH 30
#define DRAW_TEXT_MAX 20
#define USE_MYVSPRINTF

#define CLOCK_DIVIDER RCC_HPRE_DIV8
#define CLOCK_SPEED   3000000

#define I2C_SPEED   100000
#define I2C_INST    I2C1
#define I2C_TIMEOUT 1000000

/*
 *PA2(3) = button
 */
#define BUTTON_PIN     GPIO_Pin_4
#define BUTTON_PORT    GPIOC
#define BUTTON_PRESSED (!(BUTTON_PORT->INDR & BUTTON_PIN))

/*
 *PD6 = POWER ON
 */
#define POWER_ON_PIN  GPIO_Pin_3
#define POWER_ON_PORT GPIOC
#define POWER_OFF     POWER_ON_PORT->BCR = POWER_ON_PIN
#define POWER_ON      POWER_ON_PORT->BSHR = POWER_ON_PIN

void SysInit(void);
void delayms(int ms);

#endif
