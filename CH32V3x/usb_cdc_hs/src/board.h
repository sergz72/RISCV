#ifndef _BOARD_H
#define _BOARD_H

#ifndef NULL
#define NULL 0
#endif

#define LED_BLUE_PORT GPIOB
#define LED_BLUE_PIN GPIO_Pin_4
#define LED_RED_PORT GPIOA
#define LED_RED_PIN GPIO_Pin_15

#define TLV_RESET_PORT GPIOB
#define TLV_RESET_PIN GPIO_Pin_14

#define LED_BLUE_ON GPIO_WriteBit(LED_BLUE_PORT, LED_BLUE_PIN, 1)
#define LED_BLUE_OFF GPIO_WriteBit(LED_BLUE_PORT, LED_BLUE_PIN, 0)

#define LED_RED_ON GPIO_WriteBit(LED_RED_PORT, LED_RED_PIN, 1)
#define LED_RED_OFF GPIO_WriteBit(LED_RED_PORT, LED_RED_PIN, 0)

#define CDC_RX_BUF_LEN 8192
#define USB_CDC_RX_BUFFER_SIZE 512

void HalInit(void);

#endif
