#ifndef _BOARD_H
#define _BOARD_H

#ifndef NULL
#define NULL 0
#endif

#include <ch32v00X.h>

#define LED_TIMER_PIN  GPIO_Pin_4
#define LED_TIMER_PORT GPIOD
#define LED_TIMER_OFF  GPIOD->BCR = LED_TIMER_PIN
#define LED_TIMER_ON   GPIOD->BSHR = LED_TIMER_PIN

#define SSD1306_128_64
#define LCD_ORIENTATION LCD_ORIENTATION_LANDSCAPE

#include <lcd_ssd1306.h>

#define LCD_PRINTF_BUFFER_LENGTH 30
#define DRAW_TEXT_MAX 20

#define I2C_TIMEOUT 10000
#define I2C_INST    I2C1
#define I2C_CLOCK   RCC_PB1Periph_I2C1
#define I2C_SPEED   400000

#define USART_TX_PIN       GPIO_Pin_5
#define USART_RX_PIN       GPIO_Pin_6
#define USART_PORT         GPIOD
#define USART_BAUDRATE     115200
#define USART_INST         USART1
#define USART_IRQn         USART1_IRQn
#define USART_IRQHandler   USART1_IRQHandler
#define USART_CLOCK_ENABLE RCC_PB2PeriphClockCmd(RCC_PB2Periph_USART1, ENABLE);
#define USART_BUF_LEN      256

#define __weak
#define SPI_TIMEOUT        100000
#define SPI_SCK_PIN        GPIO_Pin_4
#define SPI_SCK_PORT       GPIOC
#define SPI_MOSI_PIN       GPIO_Pin_3
#define SPI_MOSI_PORT      GPIOC
#define SPI_MISO_PIN       GPIO_Pin_0
#define SPI_MISO_PORT      GPIOC
#define SPI_CS_PIN         GPIO_Pin_2
#define SPI_CS_PORT        GPIOA
#define SPI_CHANNELS       1
#define SPI_DELAY(ch)      0
#define SPI_CLK_ACTIVE(ch) SPI_SCK_PORT->BSHR = SPI_SCK_PIN
#define SPI_CLK_IDLE(ch)   SPI_SCK_PORT->BCR = SPI_SCK_PIN
#define SPI_CS_CLR(ch)     SPI_CS_PORT->BCR = SPI_CS_PIN
#define SPI_CS_SET(ch)     SPI_CS_PORT->BSHR = SPI_CS_PIN
#define SPI_MOSI_CLR(ch)   SPI_MOSI_PORT->BCR = SPI_MOSI_PIN
#define SPI_MOSI_SET(ch)   SPI_MOSI_PORT->BSHR = SPI_MOSI_PIN
#define SPI_CHECK_MISO(ch) (SPI_MISO_PORT->INDR & SPI_MISO_PIN)
#define SPI_MEMORY_MAX_CHANNELS 1

#define PRINTF_BUFFER_LENGTH 100
#define COMMMAND_LINE_SIZE 50
#define USE_MYVSPRINTF

#define USART_INTERRUPT_PRIORITY 1
#define TIMER_INTERRUPT_PRIORITY 0

#define INA228_MAX_CHANNELS 1
#define INA_ADDRESS 0x40

#define BUTTON1_PORT GPIOC
#define BUTTON1_PIN  GPIO_Pin_7
#define BUTTON1_PRESSED (!(BUTTON1_PORT->INDR & BUTTON1_PIN))
#define BUTTON2_PORT GPIOA
#define BUTTON2_PIN  GPIO_Pin_1
#define BUTTON2_PRESSED (!(BUTTON2_PORT->INDR & BUTTON2_PIN))
#define BUTTON3_PORT GPIOD
#define BUTTON3_PIN  GPIO_Pin_7
#define BUTTON3_PRESSED (!(BUTTON3_PORT->INDR & BUTTON3_PIN))

#define ALERT_PORT GPIOC
#define ALERT_PIN  GPIO_Pin_6
#define CHECK_ALERT (ALERT_PORT->INDR & ALERT_PIN)

#define DISPLAY_MAX_ROWS       4
#define DISPLAY_MAX_COLUMNS    10
#define DISPLAY_MAX_RECTANGLES 0
#define CHAR_SPACE             0x20

extern volatile bool timer_interrupt;
extern volatile char command;
extern volatile unsigned int time_since_boot_ms;

void SysInit(void);
void delayms(unsigned int);
void TimerEnable(void);
void usart_transmit(char c);
unsigned int get_keyboard_status(void);

#endif
