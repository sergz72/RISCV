#ifndef _BOARD_H
#define _BOARD_H

#ifndef NULL
#define NULL 0
#endif

#define MAX_TRANSFER_SIZE 512

#define ADC_PIN GPIO_Pin_0
#define ADC_PORT GPIOA
#define ADC_CHANNEL ADC_Channel_0

/*
 * SPI_NSS  = PB11
 * SPI_SCK  = PC15
 * SPI_MOSI = PC14
 */
#define SPI_CS_SET(channel) GPIOB->BSHR = GPIO_Pin_11
#define SPI_CS_CLR(channel) GPIOB->BCR = GPIO_Pin_11
#define SPI_CLK_SET(channel) GPIOC->BSHR = GPIO_Pin_15
#define SPI_CLK_CLR(channel) GPIOC->BCR = GPIO_Pin_15
#define SPI_DELAY(x) 10
#define SPI_CHECK_MISO(channel) 0
#define SPI_MOSI_SET(channel) GPIOC->BSHR = GPIO_Pin_14
#define SPI_MOSI_CLR(channel) GPIOC->BCR = GPIO_Pin_14

#include <ch32x035.h>

#define WAIT_FOR_INTERRUPT __WFI()

//PB12
#define LED_TIMER_PIN GPIO_Pin_12
#define LED_TIMER_PORT GPIOB
#define LED_TIMER_ON GPIOB->BCR = LED_TIMER_PIN
#define LED_TIMER_OFF GPIOB->BSHR = LED_TIMER_PIN

//PB3
#define LED_COMMAND_PIN GPIO_Pin_3
#define LED_COMMAND_PORT GPIOB
#define LED_COMMAND_ON GPIOB->BCR = LED_COMMAND_PIN
#define LED_COMMAND_OFF GPIOB->BSHR = LED_COMMAND_PIN

#endif
