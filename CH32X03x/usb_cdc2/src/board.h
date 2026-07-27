#ifndef _BOARD_H
#define _BOARD_H

#ifndef NULL
#define NULL 0
#endif

#include <ch32x035.h>

//PB12
#define LED_PIN GPIO_Pin_12
#define LED_PORT GPIOB
#define LED_ON GPIOB->BCR = LED_PIN
#define LED_OFF GPIOB->BSHR = LED_PIN

#define CDC_RX_BUF_LEN 1024

extern volatile unsigned int timer_interrupt;

void SysInit(void);
void TimerEnable(void);

#endif
