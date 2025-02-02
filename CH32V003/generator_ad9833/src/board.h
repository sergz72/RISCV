#ifndef _BOARD_H
#define _BOARD_H

#define MAX_SPI_DMA_TRANSFER_SIZE 512

#define SPI_NCS_EXTI_LINE EXTI_Line4
#define SPI_NCS_PIN GPIO_Pin_4
#define SPI_NCS_EXTI_SOURCE GPIO_PinSource4

void SysInit(void *rxaddress, const void *txaddress);
void spi_enable(void);
void spi_disable(void *rxaddress, const void *txaddress);

#endif
