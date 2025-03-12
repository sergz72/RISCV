#ifndef _BOARD_H
#define _BOARD_H

#ifndef NULL
#define NULL 0
#endif

#define USB_CDC_RX_BUFFER_SIZE 1024

#define MAX_DEVICES 5

#define INTERNAL_DEVICES

#define INTERNAL_DEVICES_COUNT 0

#define SPI_DELAY(channel) 24
#define SPI_SOFT_CLK_IDLE_LOW

//module reset
#define PIN_RESET GPIO_Pin_5
#define PORT_RESET GPIOA

//module 1
#define PIN_SDA1 GPIO_Pin_11
#define PORT_PIN_SDA1 GPIOB
#define PIN_SCL1 GPIO_Pin_10
#define PORT_PIN_SCL1 GPIOB
#define PIN_1_0  GPIO_Pin_1
#define PORT_PIN_1_0 GPIOB
#define PIN_1_1  GPIO_Pin_0
#define PORT_PIN_1_1 GPIOB
#define PIN_1_2  GPIO_Pin_7
#define PORT_PIN_1_2 GPIOA
#define PIN_1_3  GPIO_Pin_6
#define PORT_PIN_1_3 GPIOA

//module 2
#define PIN_SDA2 GPIO_Pin_3
#define PORT_PIN_SDA2 GPIOA
#define PIN_SCL2 GPIO_Pin_2
#define PORT_PIN_SCL2 GPIOA
#define PIN_2_0  GPIO_Pin_1
#define PORT_PIN_2_0 GPIOA
#define PIN_2_1  GPIO_Pin_0
#define PORT_PIN_2_1 GPIOA
#define PIN_2_2  -1
#define PORT_PIN_2_2 NULL
#define PIN_2_3  -1
#define PORT_PIN_2_3 NULL

//module 3
#define PIN_SDA3 GPIO_Pin_8
#define PORT_PIN_SDA3 GPIOB
#define PIN_SCL3 GPIO_Pin_7
#define PORT_PIN_SCL3 GPIOB
#define PIN_3_0  GPIO_Pin_6
#define PORT_PIN_3_0 GPIOB
#define PIN_3_1  GPIO_Pin_5
#define PORT_PIN_3_1 GPIOB
#define PIN_3_2  -1
#define PORT_PIN_3_2 NULL
#define PIN_3_3  -1
#define PORT_PIN_3_3 NULL

// module 4
#define PIN_SDA4 GPIO_Pin_4
#define PORT_PIN_SDA4 GPIOB
#define PIN_SCL4 GPIO_Pin_3
#define PORT_PIN_SCL4 GPIOB
#define PIN_4_0  GPIO_Pin_15
#define PORT_PIN_4_0 GPIOA
#define PIN_4_1  GPIO_Pin_10
#define PORT_PIN_4_1 GPIOA
#define PIN_4_2  -1
#define PORT_PIN_4_2 NULL
#define PIN_4_3  -1
#define PORT_PIN_4_3 NULL

// module 5
#define PIN_SDA5 GPIO_Pin_9
#define PORT_PIN_SDA5 GPIOA
#define PIN_SCL5 GPIO_Pin_8
#define PORT_PIN_SCL5 GPIOA
#define PIN_5_0  GPIO_Pin_15
#define PORT_PIN_5_0 GPIOB
#define PIN_5_1  GPIO_Pin_14
#define PORT_PIN_5_1 GPIOB
#define PIN_5_2  GPIO_Pin_13
#define PORT_PIN_5_2 GPIOB
#define PIN_5_3  GPIO_Pin_12
#define PORT_PIN_5_3 GPIOB

#include "core_main.h"

int SPI_CHECK_MISO(int channel);
void SPI_MOSI_SET(int channel);
void SPI_MOSI_CLR(int channel);
void SPI_CS_SET(int channel);
void SPI_CS_CLR(int channel);
void SPI_CLK_SET(int channel);
void SPI_CLK_CLR(int channel);

#include <delay.h>

#endif
