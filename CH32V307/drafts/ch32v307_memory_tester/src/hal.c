#include "board.h"
#include <ch32v30x.h>
#include "debug.h"
#include <spi_memory.h>

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

  GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
  GPIO_InitStructure.GPIO_Pin = I2C1_SCL_PIN | I2C1_SDA_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( I2C1_PORT, &GPIO_InitStructure );

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

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  GPIO_InitStructure.GPIO_Pin = SPI1_MOSI_PIN | SPI1_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( SPI1_PORT, &GPIO_InitStructure );

  GPIO_InitStructure.GPIO_Pin = SPI1_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( SPI1_PORT, &GPIO_InitStructure );

  SPI1_CS_SET;
  GPIO_InitStructure.GPIO_Pin = SPI1_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init( SPI1_PORT, &GPIO_InitStructure );

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
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

  GPIO_InitStructure.GPIO_Pin = SPI3_MOSI_PIN | SPI3_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( SPI3_PORT, &GPIO_InitStructure );

  GPIO_InitStructure.GPIO_Pin = SPI3_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( SPI3_PORT, &GPIO_InitStructure );

  SPI3_CS_SET;
  GPIO_InitStructure.GPIO_Pin = SPI3_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init( SPI3_CS_PORT, &GPIO_InitStructure );

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
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

void spi_finish(int channel)
{
  switch (channel)
  {
  case 1:
    SPI1_CS_SET;
    break;
  case 2:
    //SPI2_CS_SET;
    //break;
    break;
  case 3:
    SPI3_CS_SET;
    break;
  default: break;
  }
}

static unsigned char spi_send_receive(SPI_TypeDef *instance, unsigned char data)
{
  while( SPI_I2S_GetFlagStatus( instance, SPI_I2S_FLAG_TXE ) == RESET )
    ;
  SPI_I2S_SendData(instance, data);
  while(SPI_I2S_GetFlagStatus( instance, SPI_I2S_FLAG_RXNE ) == RESET )
    ;
  return SPI_I2S_ReceiveData(instance);
}

static void qspi_spi_trfr(int nwrite, const unsigned char *wdata, int nop_cycles, int nread, unsigned char *rdata, int set_cs)
{
  while (nwrite--)
  {
    unsigned char data = *wdata++;
    for (int i = 0; i < 8; i++)
    {
      unsigned char bit = data & 0x80 ? 1 : 0;
      QSPI_PORT->OUTDR = bit;
      data <<= 1;
      QSPI_PORT->OUTDR = bit | 0x10; // set clk
    }
  }
  while (nop_cycles--)
  {
    QSPI_PORT->OUTDR = 0;
    QSPI_PORT->OUTDR = 0x10; // set clk
  }
  if (nread)
  {
    while (nread--)
    {
      unsigned char data = 0;
      for (int i = 0; i < 8; i++)
      {
        data <<= 1;
        QSPI_PORT->OUTDR = 0;
        QSPI_PORT->OUTDR = 0x10; // set clk
        data |= QSPI_PORT->INDR & 1;
      }
      *rdata++ = data;
    }
  }
  if (set_cs)
    QSPI_PORT->OUTDR = 0x20; //set cs
}

void spi_trfr(int channel, int nwrite, const unsigned char *wdata, int nop_cycles, int nread, unsigned char *rdata, int set_cs)
{
  SPI_TypeDef *instance;
  switch (channel)
  {
    case 0:
      qspi_spi_trfr(nwrite, wdata, nop_cycles, nread, rdata, set_cs);
      return;
    case 1:
      SPI1_CS_CLR;
      instance = SPI1;
      break;
    case 2:
      //SPI2_CS_CLR;
      //instance = SPI2;
      //break;
      return;
    case 3:
      SPI3_CS_CLR;
      instance = SPI3;
      break;
    default: return;
  }

  while (nwrite--)
    spi_send_receive(instance, *wdata++);
  nop_cycles >>= 3;
  while (nop_cycles--)
    spi_send_receive(instance, 0);
  if (nread)
  {
    while (nread--)
      *rdata++ = spi_send_receive(instance, 0);
  }

  if (set_cs)
    spi_finish(channel);
}

void qspi_set_sio_direction(int out0, int out1, int out2, int out3)
{
  switch (out0 + out1)
  {
    case 0:
      QSPI_PORT->CFGLR = 0x88338888; // all in
      break;
    case 1:
      QSPI_PORT->CFGLR = 0x88338883; // out0
      break;
    default:
      QSPI_PORT->CFGLR = 0x88333333; // all out
      break;
  }
}

void qspi_trfr(int channel, int nwrite, const unsigned char *wdata, int nop_cycles, int nread, unsigned char *rdata, int set_cs)
{
  if (channel != 0)
    return;
  qspi_set_sio_direction(1, 1, 1, 1);
  while (nwrite--)
  {
    unsigned char data = *wdata;
    unsigned char d1 = data >> 4;
    QSPI_PORT->OUTDR = d1;
    QSPI_PORT->OUTDR = d1 | 0x10; // set clk
    data &= 0x0F;
    QSPI_PORT->OUTDR = data;
    wdata++;
    QSPI_PORT->OUTDR = data | 0x10; // set clk
  }
  if (set_cs)
    qspi_set_sio_direction(0, 0, 0, 0);
  while (nop_cycles--)
  {
    QSPI_PORT->OUTDR = 0;
    QSPI_PORT->OUTDR = 0x10; // set clk
  }
  if (nread)
  {
    while (nread--)
    {
      QSPI_PORT->OUTDR = 0;
      QSPI_PORT->OUTDR = 0x10; // set clk
      unsigned char data = (QSPI_PORT->INDR & 0x0F) << 4;
      QSPI_PORT->OUTDR = 0;
      QSPI_PORT->OUTDR = 0x10; // set clk
      *rdata++ = data | (QSPI_PORT->INDR & 0x0F);
    }
  }
  if (set_cs)
    QSPI_PORT->OUTDR = 0x20; //set cs
}

void enter_93xx_mode(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, DISABLE);

  _93CXX_CS_LOW(0);
  GPIO_InitStructure.GPIO_Pin = SPI3_MOSI_PIN | SPI3_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( SPI3_PORT, &GPIO_InitStructure );
}
