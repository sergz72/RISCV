#ifndef _BOARD_H
#define _BOARD_H

#ifndef NULL
#define NULL 0
#endif

#include <ch32v00x.h>
#include <delay.h>

#define I2C_ADDRESS 2

#define I2C_SOFT
#define I2C_TIMEOUT 1000000
#define i2c_dly Delay_Us(5)

/*
 *PA1 -> SDA
 *PD7 -> SCL
 */
#define SDA_PIN GPIO_Pin_1
#define SDA_PORT GPIOA
#define SCL_PIN GPIO_Pin_7
#define SCL_PORT GPIOD
#define SCL_HIGH(channel) SCL_PORT->BSHR = SCL_PIN
#define SCL_LOW(channel) SCL_PORT->BCR = SCL_PIN
#define SDA_HIGH(channel) SDA_PORT->BSHR = SDA_PIN
#define SDA_LOW(channel) SDA_PORT->BCR = SDA_PIN
#define SDA_IN(channel) (SDA_PORT->INDR & SDA_PIN)
#define SCL_IN(channel) (SCL_PORT->INDR & SCL_PIN)

/*
 *PA2 = PWM
 */
#define PWM_PIN GPIO_Pin_2
#define PWM_PORT GPIOA
#define PWM_PERIOD 480
#define PWM_PULSE 48
#define PWM_OCINIT TIM_OC2Init
#define PWM_OCPRELOADCONFIG TIM_OC2PreloadConfig

/*
 *PC0 = button
 */
#define BUTTON_PIN GPIO_Pin_0
#define BUTTON_PRESSED GPIOD->INDR & BUTTON_PIN
#define BUTTON_PORT GPIOC

/*
 *PD6 = LED
 */
#define LED_TIMER_PIN GPIO_Pin_6
#define LED_TIMER_PORT GPIOD
#define LED_TIMER_ON GPIOD->BCR = LED_TIMER_PIN
#define LED_TIMER_OFF GPIOD->BSHR = LED_TIMER_PIN

/*
 *PD5 = INTERRUPT
 */
#define INTERRUPT_FLAG_PIN GPIO_Pin_5
#define INTERRUPT_FLAG_PORT GPIOD
#define INTERRUPT_FLAG_CLR GPIOD->BCR = INTERRUPT_FLAG_PIN
#define INTERRUPT_FLAG_SET GPIOD->BSHR = INTERRUPT_FLAG_PIN

extern volatile int interrupt_request;
extern const void *txbufs[2];

void SysInit(void);

#endif
