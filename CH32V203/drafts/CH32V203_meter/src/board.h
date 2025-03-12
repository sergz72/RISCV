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
#define PIN_RESET 11
#define PORT_RESET GPIOA
//module 1
#define PIN_SDA1 2
#define PORT_PIN_SDA1 GPIOA
#define PIN_SCL1 3
#define PORT_PIN_SCL1 GPIOA
#define PIN_1_0  4
#define PORT_PIN_1_0 GPIOA
#define PIN_1_1  5
#define PORT_PIN_1_1 GPIOA
#define PIN_1_2  0
#define PORT_PIN_1_2 GPIOA
#define PIN_1_3  1
#define PORT_PIN_1_3 GPIOA

//module 2
#define PIN_SDA2 6
#define PORT_PIN_SDA2 GPIOA
#define PIN_SCL2 7
#define PORT_PIN_SCL2 GPIOA
#define PIN_2_0  8
#define PORT_PIN_2_0 GPIOA
#define PIN_2_1  9
#define PORT_PIN_2_1 GPIOA
#define PIN_2_2  -1
#define PORT_PIN_2_2 GPIOA
#define PIN_2_3  -1
#define PORT_PIN_2_3 GPIOA

//module 3
#define PIN_SDA3 12
#define PORT_PIN_SDA3 GPIOA
#define PIN_SCL3 13
#define PORT_PIN_SCL3 GPIOA
#define PIN_3_0  14
#define PORT_PIN_3_0 GPIOA
#define PIN_3_1  15
#define PORT_PIN_3_1 GPIOA
#define PIN_3_2  -1
#define PORT_PIN_3_2 GPIOA
#define PIN_3_3  -1
#define PORT_PIN_3_3 GPIOA

// module 4
#define PIN_SDA4 17
#define PORT_PIN_SDA4 GPIOA
#define PIN_SCL4 16
#define PORT_PIN_SCL4 GPIOA
#define PIN_4_0  19
#define PORT_PIN_4_0 GPIOA
#define PIN_4_1  18
#define PORT_PIN_4_1 GPIOA
#define PIN_4_2  -1
#define PORT_PIN_4_2 GPIOA
#define PIN_4_3  -1
#define PORT_PIN_4_3 GPIOA

// module 5
#define PIN_SDA5 26
#define PORT_PIN_SDA5 GPIOA
#define PIN_SCL5 22
#define PORT_PIN_SCL5 GPIOA
#define PIN_5_0  28
#define PORT_PIN_5_0 GPIOA
#define PIN_5_1  27
#define PORT_PIN_5_1 GPIOA
#define PIN_5_2  21
#define PORT_PIN_5_2 GPIOA
#define PIN_5_3  20
#define PORT_PIN_5_3 GPIOA

#include "core_main.h"

int SPI_CHECK_MISO(int channel);
void SPI_MOSI_SET(int channel);
void SPI_MOSI_CLR(int channel);
void SPI_CS_SET(int channel);
void SPI_CS_CLR(int channel);
void SPI_CLK_SET(int channel);
void SPI_CLK_CLR(int channel);

#endif
