#include <ch32v00x_misc.h>
#include <system_ch32v00x.h>
#include "board.h"

static unsigned char status;
static unsigned char spi_rxbuf[MAX_SPI_DMA_TRANSFER_SIZE];
static unsigned int command_ready;

static const unsigned char device_id = 1;
static const unsigned char device_config = 2; // todo
static const unsigned char *spi_txbuf[3] =
{
    &device_id,
    &device_config,
    &status
};

void __attribute__((interrupt("WCH-Interrupt-fast"))) EXTI7_0_IRQHandler(void)
{
    if (EXTI->INTFR & SPI_NCS_EXTI_LINE)
    {
        if (GPIOC->INDR & SPI_NCS_PIN) // Rising edge
        {
            spi_disable(spi_rxbuf, spi_txbuf[spi_rxbuf[0]]);
            command_ready = 1;
        }
        else
            spi_enable();
        EXTI->INTFR = SPI_NCS_EXTI_LINE;
    }
}

int main(void)
{
    command_ready = 0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    SysInit(spi_rxbuf, &device_id);

    while(1)
    {
        __WFI();
        if (command_ready)
        {
            //todo
            command_ready = 0;
        }
    }
}
