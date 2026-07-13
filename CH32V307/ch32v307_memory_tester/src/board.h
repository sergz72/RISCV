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

#define TLV_RESET_PORT GPIOB
#define TLV_RESET_PIN GPIO_Pin_14

#define LED_BLUE_ON GPIO_WriteBit(LED_BLUE_PORT, LED_BLUE_PIN, 1)
#define LED_BLUE_OFF GPIO_WriteBit(LED_BLUE_PORT, LED_BLUE_PIN, 0)

#define LED_RED_ON GPIO_WriteBit(LED_RED_PORT, LED_RED_PIN, 1)
#define LED_RED_OFF GPIO_WriteBit(LED_RED_PORT, LED_RED_PIN, 0)

#define CDC_RX_BUF_LEN 8192
#define USB_CDC_RX_BUFFER_SIZE 512
#define PRINTF_BUFFER_LENGTH 100

#define I2C_TIMEOUT 200

#define MAX_SHELL_COMMANDS 50
#define MAX_SHELL_COMMAND_PARAMETERS 10
#define MAX_SHELL_COMMAND_PARAMETER_LENGTH 50
#define SHELL_HISTORY_SIZE 20
#define SHELL_HISTORY_ITEM_LENGTH 100

#define QSPI_PORT        GPIOE
#define QSPI_D0_MOSI_PIN GPIO_Pin_10
#define QSPI_D1_MISO_PIN GPIO_Pin_11
#define QSPI_D2_PIN      GPIO_Pin_12
#define QSPI_D3_PIN      GPIO_Pin_13
#define QSPI_CS_PIN      GPIO_Pin_14
#define QSPI_SCK_PIN     GPIO_Pin_15

#define QSPI_SCK_CLR GPIO_WriteBit(QSPI_PORT, QSPI_SCK_PIN, 0)
#define QSPI_CS_SET  GPIO_WriteBit(QSPI_PORT, QSPI_CS_PIN, 1)

#define SPI1_PORT     GPIOA
#define SPI1_CS_PIN   GPIO_Pin_2
#define SPI1_MOSI_PIN GPIO_Pin_7
#define SPI1_MISO_PIN GPIO_Pin_6
#define SPI1_SCK_PIN  GPIO_Pin_5

#define SPI1_CS_SET GPIO_WriteBit(SPI1_PORT, SPI1_CS_PIN, 1)
#define SPI1_CS_CLR GPIO_WriteBit(SPI1_PORT, SPI1_CS_PIN, 0)

#define I2C2_PORT    GPIOB
#define I2C2_SCL_PIN GPIO_Pin_10
#define I2C2_SDA_PIN GPIO_Pin_11

#define I2C1_PORT    GPIOB
#define I2C1_SCL_PIN GPIO_Pin_8
#define I2C1_SDA_PIN GPIO_Pin_9

#define SPI3_PORT     GPIOB
#define SPI3_MOSI_PIN GPIO_Pin_5
#define SPI3_MISO_PIN GPIO_Pin_4
#define SPI3_SCK_PIN  GPIO_Pin_3
#define SPI3_CS_PORT  GPIOD
#define SPI3_CS_PIN   GPIO_Pin_7

#define SPI3_CS_SET GPIO_WriteBit(SPI3_CS_PORT, SPI3_CS_PIN, 1)
#define SPI3_CS_CLR GPIO_WriteBit(SPI3_CS_PORT, SPI3_CS_PIN, 0)

#define SPI_MEMORY_MAX_CHANNELS 4

#define MEMORY_BUFFER_SIZE 1024

#define _93CXX_DI_LOW(channel)   GPIO_WriteBit(SPI3_PORT, SPI3_MOSI_PIN, 0)
#define _93CXX_DI_HIGH(channel)  GPIO_WriteBit(SPI3_PORT, SPI3_MOSI_PIN, 1)
#define _93CXX_CLK_LOW(channel)  GPIO_WriteBit(SPI3_PORT, SPI3_SCK_PIN, 0)
#define _93CXX_CLK_HIGH(channel) GPIO_WriteBit(SPI3_PORT, SPI3_SCK_PIN, 1)
#define _93CXX_CS_LOW(channel)   GPIO_WriteBit(SPI3_CS_PORT, SPI3_CS_PIN, 0)
#define _93CXX_CS_HIGH(channel)  GPIO_WriteBit(SPI3_CS_PORT, SPI3_CS_PIN, 1)
#define _93CXX_DO(channel)       GPIO_ReadInputDataBit(SPI3_PORT, SPI3_MISO_PIN)
#define _93CXX_DELAY             Delay_Us(1)
#define _93XX_TIMEOUT            10000

#define delayms(ms) Delay_Ms(ms)

void HalInit(void);
void enter_93xx_mode(void);

#endif
