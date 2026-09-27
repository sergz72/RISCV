#ifndef _BOARD_H
#define _BOARD_H

#ifndef NULL
#define NULL 0
#endif

#include <ch32x035.h>

//PA4
#define LED_PIN GPIO_Pin_4
#define LED_PORT GPIOA
#define LED_ON GPIOA->BCR = LED_PIN
#define LED_OFF GPIOA->BSHR = LED_PIN

#define MAX_SHELL_COMMANDS 20
#define MAX_SHELL_COMMAND_PARAMETERS 10
#define MAX_SHELL_COMMAND_PARAMETER_LENGTH 50
#define SHELL_HISTORY_SIZE 10
#define SHELL_HISTORY_ITEM_LENGTH 100

#define CDC_RX_BUF_LEN       256
#define RX_BUFFER_LENGTH     256
#define COMMAND_LINE_LENGTH  200
#define PRINTF_BUFFER_LENGTH 200

#define PUTS_FUNC puts_
#define PRINTF_FUNC common_printf

extern volatile unsigned int timer_interrupt;

void SysInit(void);
void TimerEnable(void);

#include <common_printf.h>

#endif
