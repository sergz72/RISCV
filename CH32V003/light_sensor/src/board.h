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
#define BUTTON_PIN     GPIO_Pin_2
#define BUTTON_PORT    GPIOA
#define BUTTON_PRESSED (!(BUTTON_PORT->INDR & BUTTON_PIN))

/*
 *PD6 = POWER ON
 */
#define POWER_ON_PIN  GPIO_Pin_1
#define POWER_ON_PORT GPIOA
#define POWER_OFF     GPIOA->BCR = POWER_ON_PIN
#define POWER_ON      GPIOA->BSHR = POWER_ON_PIN

//#define LED_DEBUG

#ifdef LED_DEBUG
#define LED2_PIN  GPIO_Pin_4
#define LED2_PORT GPIOD
#define LED2_OFF  LED2_PORT->BCR = LED2_PIN
#define LED2_ON   LED2_PORT->BSHR = LED2_PIN

#define LED3_PIN  GPIO_Pin_7
#define LED3_PORT GPIOC
#define LED3_OFF  LED3_PORT->BCR = LED3_PIN
#define LED3_ON   LED3_PORT->BSHR = LED3_PIN

#define LED4_PIN  GPIO_Pin_6
#define LED4_PORT GPIOC
#define LED4_OFF  LED4_PORT->BCR = LED4_PIN
#define LED4_ON   LED4_PORT->BSHR = LED4_PIN

#define LED5_PIN  GPIO_Pin_4
#define LED5_PORT GPIOC
#define LED5_OFF  LED5_PORT->BCR = LED5_PIN
#define LED5_ON   LED5_PORT->BSHR = LED5_PIN

#define LED6_PIN  GPIO_Pin_3
#define LED6_PORT GPIOC
#define LED6_OFF  LED6_PORT->BCR = LED6_PIN
#define LED6_ON   LED6_PORT->BSHR = LED6_PIN
#endif

void SysInit(void);
void delayms(int ms);

#endif
