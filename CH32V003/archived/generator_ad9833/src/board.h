#ifndef _BOARD_H
#define _BOARD_H

#ifndef NULL
#define NULL 0
#endif

#define MAX_SPI_TRANSFER_SIZE 512

#define ADC_PIN GPIO_Pin_3
#define ADC_CHANNEL ADC_Channel_4

#define SPI_CS_SET(channel)
#define SPI_CS_CLR(channel)
#define SPI_CLK_SET(channel)
#define SPI_CLK_CLR(channel)
#define SPI_DELAY(x) 10
#define SPI_CHECK_MISO(channel) 0
#define SPI_MOSI_SET(channel)
#define SPI_MOSI_CLR(channel)

#include <ch32v00x.h>

#define WAIT_FOR_INTERRUPT __WFI()

#define LED_TIMER_PIN GPIO_Pin_4
#define LED_TIMER_PORT GPIOD
#define LED_TIMER_ON GPIOD->BCR = LED_TIMER_PIN
#define LED_TIMER_OFF GPIOD->BSHR = LED_TIMER_PIN
#define LED_TIMER_IOPC RCC_IOPDEN

#define LED_COMMAND_PIN GPIO_Pin_5
#define LED_COMMAND_PORT GPIOD
#define LED_COMMAND_ON GPIOD->BCR = LED_COMMAND_PIN
#define LED_COMMAND_OFF GPIOD->BSHR = LED_COMMAND_PIN
#define LED_COMMAND_IOPC RCC_IOPDEN

#endif
