#ifndef _BOARD_H
#define _BOARD_H

#ifndef NULL
#define NULL 0
#endif

#include <delay.h>
#include <ch32v30x_gpio.h>

#define LED_BLUE_PORT GPIOB
#define LED_BLUE_PIN GPIO_Pin_4
#define LED_RED_PORT GPIOA
#define LED_RED_PIN GPIO_Pin_15

#define LED_BLUE_ON GPIO_WriteBit(LED_BLUE_PORT, LED_BLUE_PIN, 1)
#define LED_BLUE_OFF GPIO_WriteBit(LED_BLUE_PORT, LED_BLUE_PIN, 0)

#define LED_RED_ON GPIO_WriteBit(LED_RED_PORT, LED_RED_PIN, 1)
#define LED_RED_OFF GPIO_WriteBit(LED_RED_PORT, LED_RED_PIN, 0)

#define CDC_RX_BUF_LEN 8192
#define USB_CDC_RX_BUFFER_SIZE 512
#define PRINTF_BUFFER_LENGTH 100

#define MAX_SHELL_COMMANDS 30
#define MAX_SHELL_COMMAND_PARAMETERS 10
#define MAX_SHELL_COMMAND_PARAMETER_LENGTH 30
#define SHELL_HISTORY_SIZE 20
#define SHELL_HISTORY_ITEM_LENGTH 100

void HalInit(void);

extern volatile unsigned int timeCnt, timer_interrupt;

#endif
