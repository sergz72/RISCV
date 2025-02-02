#include "board.h"
#include <ch32v00x.h>

static void dma_disable(void *rxaddress, const void *txaddress)
{
  DMA1_Channel3->CFGR &= ~DMA_CFGR3_EN;
  DMA1_Channel2->CFGR &= ~DMA_CFGR3_EN;
  //Buffer address
  DMA1_Channel3->MADDR = (uint32_t)txaddress;
  //Number of data transfer
  DMA1_Channel3->CNTR = 65535;
  //Buffer address
  DMA1_Channel2->MADDR = (uint32_t)rxaddress;
  //Number of data transfer
  DMA1_Channel2->CNTR = 65535;
}

/*
  CH32V003F4P6
  PC1 - NSS
  PC5 - SCK
  PC6 - MOSI
  PC7 - MISO
 */

static void spi_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  //Enable clock for PORTC, SPI1 and DMA1
  RCC->APB2PCENR |=  RCC_IOPCEN | RCC_SPI1EN;
  RCC->AHBPCENR |= RCC_DMA1EN;

  //PC6(MOSI) and PC5(SCK) - Floating inputs
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  //PC7(MISO) - Multiplexed push pull output
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  //Set SPI1, max clock 48Mhz/2 = 24Mhz, slave mode, full-duplex mode,8bit data length
  //SPI1->CTLR1 = SPI_CTLR1_SSI | SPI_CTLR1_SSM;
  //Enable DMA for SPI TX and SPI RX
  SPI1->CTLR2 = SPI_CTLR2_TXDMAEN | SPI_CTLR2_TXDMAEN;

  //Set DMA1 - Channel 3, Very high priority, 8bit size, mem -> spi direction
  // memory increment and no perpheral increment
  DMA1_Channel3->CFGR |= DMA_CFGR1_PL_0 | DMA_CFGR1_PL_1 | DMA_CFGR1_DIR | DMA_CFGR1_MINC;
  //Take SPI1 Data register address
  DMA1_Channel3->PADDR = (uint32_t)&SPI1->DATAR;

  //Set DMA1 - Channel 2, Very high priority, 8bit size, spi -> mem direction
  // memory increment and no perpheral increment
  DMA1_Channel2->CFGR = DMA_CFGR1_PL_0 | DMA_CFGR1_PL_1 | DMA_CFGR1_MINC;
  //Take SPI1 Data register address
  DMA1_Channel2->PADDR = (uint32_t)&SPI1->DATAR;
}

static void exti_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  GPIO_InitStructure.GPIO_Pin = SPI_NCS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, SPI_NCS_EXTI_SOURCE);
  EXTI_InitStructure.EXTI_Line = SPI_NCS_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void SysInit(void *rxaddress, const void *txaddress)
{
  spi_init();
  exti_init();
  dma_disable(rxaddress, txaddress);
}

static void dma_enable(void)
{
  //Enable DMA Channels
  DMA1_Channel3->CFGR |= DMA_CFGR3_EN;
  DMA1_Channel2->CFGR |= DMA_CFGR2_EN;
}

void spi_enable(void)
{
  //Enable SPI1
  SPI1->CTLR1 |= SPI_CTLR1_SPE;
  dma_enable();
}

void spi_disable(void *rxaddress, const void *txaddress)
{
  //Disable SPI1
  dma_disable(rxaddress, txaddress);
  SPI1->CTLR1 &= ~SPI_CTLR1_SPE;
}
