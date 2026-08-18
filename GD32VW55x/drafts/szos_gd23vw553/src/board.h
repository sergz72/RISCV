#ifndef _BOARD_H
#define _BOARD_H

#include <gd32vw55x.h>

#define LED_TIMER_CLOCK RCU_GPIOC
#define LED_TIMER_PORT GPIOC
#define LED_TIMER_PIN GPIO_PIN_13
#define LED_TIMER_ON gpio_bit_set(LED_TIMER_PORT,LED_TIMER_PIN)
#define LED_TIMER_OFF gpio_bit_reset(LED_TIMER_PORT,LED_TIMER_PIN)

#define MAX_SHELL_COMMANDS 40
#define MAX_SHELL_COMMAND_PARAMETERS 10
#define MAX_SHELL_COMMAND_PARAMETER_LENGTH 100
#define SHELL_HISTORY_SIZE 20
#define SHELL_HISTORY_ITEM_LENGTH 120

#define USART_TX_PIN            GPIO_PIN_15
#define USART_TX_PORT           GPIOB
#define USART_TX_AF             GPIO_AF_8
#define USART_RX_PIN            GPIO_PIN_8
#define USART_RX_PORT           GPIOA
#define USART_RX_AF             GPIO_AF_2
#define USART_BAUDRATE          115200
#define USART_INST              USART0
#define USART_IRQn              USART0_IRQn
#define USART_IRQHandler        USART0_IRQHandler
#define USART_PORT_CLOCK_ENABLE rcu_periph_clock_enable(RCU_GPIOA); rcu_periph_clock_enable(RCU_GPIOB);
#define USART_CLOCK_ENABLE      rcu_periph_clock_enable(RCU_USART0);

#define USART_INTERRUPT_PRIORITY 1

#define RX_BUF_LEN           256
#define PRINTF_BUFFER_LENGTH 200

#define PRINTF common_printf
#define PUTS   puts_

#define MAX_TASKS 8

#define FLASH_BASE_ADDR    0x08300000
#define FLASH_STORAGE_SIZE (1024*1024)

void HalInit(void);
void __attribute__((noreturn)) reboot(void);

extern unsigned char rx_buffer[RX_BUF_LEN];
extern unsigned char *rx_buffer_write_p, *rx_buffer_read_p;

#endif
