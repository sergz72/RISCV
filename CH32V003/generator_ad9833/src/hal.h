#ifndef HAL_H
#define HAL_H

#include "board.h"

extern volatile unsigned int command_ready, timer_interrupt;
extern const void *spi_txbufs[];
extern unsigned char spi_rxbuf[MAX_SPI_TRANSFER_SIZE];

void SysInit(void *rxaddress, const void *txaddress);
void spi_enable(void);
void spi_disable(void *rxaddress, const void *txaddress);
void handler_init(void);
unsigned short adc_get(void);
void set_frequency_code(int channel, unsigned long long int code, unsigned short divider);
void set_mode(int channel, int mode);
void enable_output(int channel, int enable);
void timer_enable(void);

#endif
