#ifndef _BOARD_H
#define _BOARD_H

#ifndef NULL
#define NULL 0
#endif

#include <ch32v30x_gpio.h>

#define LED_BLUE_PORT GPIOB
#define LED_BLUE_PIN GPIO_Pin_4
#define LED_RED_PORT GPIOA
#define LED_RED_PIN GPIO_Pin_15

#define LED_BLUE_ON GPIO_WriteBit(LED_BLUE_PORT, LED_BLUE_PIN, 1)
#define LED_BLUE_OFF GPIO_WriteBit(LED_BLUE_PORT, LED_BLUE_PIN, 0)

#define LED_RED_ON GPIO_WriteBit(LED_RED_PORT, LED_RED_PIN, 1)
#define LED_RED_OFF GPIO_WriteBit(LED_RED_PORT, LED_RED_PIN, 0)

#define RX_BUF_LEN           256
#define PRINTF_BUFFER_LENGTH 200

#define MAX_SHELL_COMMANDS 30
#define MAX_SHELL_COMMAND_PARAMETERS 10
#define MAX_SHELL_COMMAND_PARAMETER_LENGTH 60
#define SHELL_HISTORY_SIZE 20
#define SHELL_HISTORY_ITEM_LENGTH 120

#define ETH_IRQ_QUEUE_SIZE  4
#define ETH_USER_QUEUE_SIZE 4

#define USART_TX_PIN            GPIO_Pin_0
#define USART_RX_PIN            GPIO_Pin_1
#define USART_PORT              GPIOE
#define USART_BAUDRATE          921600
#define USART_INST              UART4
#define USART_IRQn              UART4_IRQn
#define USART_IRQHandler        UART4_IRQHandler
#define USART_PORT_CLOCK_ENABLE RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
#define USART_CLOCK_ENABLE      RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
#define USART_REMAP             GPIO_PinRemapConfig(GPIO_FullRemap_USART4, ENABLE)

#define USART_INTERRUPT_PRIORITY 1

void HalInit(const unsigned char* ntp_server_address);

extern unsigned char rx_buffer[RX_BUF_LEN];
extern unsigned char *rx_buffer_write_p, *rx_buffer_read_p;

extern volatile unsigned int timeCnt, timer_interrupt;

#endif
