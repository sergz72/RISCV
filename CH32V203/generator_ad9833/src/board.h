#ifndef _BOARD_H
#define _BOARD_H

#ifndef NULL
#define NULL 0
#endif

#define MAX_TRANSFER_SIZE 512

#define ADC_PIN GPIO_Pin_1
#define ADC_PORT GPIOB
#define ADC_CHANNEL ADC_Channel_9

#define INTERRUPT_PIN GPIO_Pin_7
#define INTERRUPT_PIN_PORT GPIOB

#include <ch32v20x.h>

#define WAIT_FOR_INTERRUPT __WFI()

//PB2
#define LED_TIMER_PIN GPIO_Pin_2
#define LED_TIMER_PORT GPIOB
#define LED_TIMER_ON GPIOB->BCR = LED_TIMER_PIN
#define LED_TIMER_OFF GPIOB->BSHR = LED_TIMER_PIN

//PA3
#define LED_COMMAND_PIN GPIO_Pin_3
#define LED_COMMAND_PORT GPIOA
#define LED_COMMAND_ON GPIOA->BCR = LED_COMMAND_PIN
#define LED_COMMAND_OFF GPIOA->BSHR = LED_COMMAND_PIN

#endif
