#include "board.h"
#include <ch32v30x.h>
#include "debug.h"
#include "delay.h"

static void GPIOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // led green
  GPIO_InitStructure.GPIO_Pin = LED_BLUE_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED_BLUE_PORT, &GPIO_InitStructure);

  // led red
  GPIO_InitStructure.GPIO_Pin = LED_RED_PIN;
  GPIO_Init(LED_RED_PORT, &GPIO_InitStructure);
}

static void I2C1Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitTSturcture;

  RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

  AFIO->PCFR1 |= AFIO_PCFR1_I2C1_REMAP;
  GPIO_InitStructure.GPIO_Pin = I2C1_SCL_PIN | I2C1_SDA_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( I2C2_PORT, &GPIO_InitStructure );

  I2C_InitTSturcture.I2C_ClockSpeed = 400000;
  I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
  I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
  I2C_InitTSturcture.I2C_OwnAddress1 = 0;
  I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
  I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init( I2C1, &I2C_InitTSturcture );

  I2C_Cmd( I2C1, ENABLE );
}

static void I2C2Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitTSturcture;

  RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C2, ENABLE );

  GPIO_InitStructure.GPIO_Pin = I2C2_SCL_PIN | I2C2_SDA_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( I2C2_PORT, &GPIO_InitStructure );

  I2C_InitTSturcture.I2C_ClockSpeed = 400000;
  I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
  I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
  I2C_InitTSturcture.I2C_OwnAddress1 = 0;
  I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
  I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init( I2C2, &I2C_InitTSturcture );

  I2C_Cmd( I2C2, ENABLE );
}

static void SPI1Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  GPIO_InitStructure.GPIO_Pin = SPI1_MOSI_PIN | SPI1_MISO_PIN | SPI1_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( SPI1_PORT, &GPIO_InitStructure );

  SPI1_CS_SET;
  GPIO_InitStructure.GPIO_Pin = SPI1_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init( SPI1_PORT, &GPIO_InitStructure );

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init( SPI1, &SPI_InitStructure );

  SPI_Cmd( SPI1, ENABLE );
}

static void SPI3Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

  GPIO_InitStructure.GPIO_Pin = SPI3_MOSI_PIN | SPI3_MISO_PIN | SPI3_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( SPI3_PORT, &GPIO_InitStructure );

  SPI3_CS_SET;
  GPIO_InitStructure.GPIO_Pin = SPI3_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init( SPI3_CS_PORT, &GPIO_InitStructure );

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init( SPI3, &SPI_InitStructure );

  SPI_Cmd( SPI3, ENABLE );
}

static void QSPIInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  QSPI_SCK_CLR;
  QSPI_CS_SET;
  GPIO_InitStructure.GPIO_Pin   = QSPI_CS_PIN | QSPI_SCK_PIN | QSPI_D0_MOSI_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( QSPI_PORT, &GPIO_InitStructure );

  GPIO_InitStructure.GPIO_Pin   = QSPI_D1_MISO_PIN | QSPI_D2_PIN | QSPI_D3_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( QSPI_PORT, &GPIO_InitStructure );
}

void HalInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  Delay_Init();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA
    | RCC_APB2Periph_GPIOB
    | RCC_APB2Periph_GPIOD
    | RCC_APB2Periph_GPIOE
    | RCC_APB2Periph_AFIO, ENABLE);

  GPIOInit();
  I2C1Init();
  I2C2Init();
  SPI1Init();
  SPI3Init();
  QSPIInit();
}
