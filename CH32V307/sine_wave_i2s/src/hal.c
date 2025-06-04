#include "board.h"
#include <ch32v30x.h>
#include "delay.h"
#include "sound.h"
#include <tlv320dac3100.h>
#include <i2c_func.h>

static void GPIOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // LED green
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // LED red
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*
  SPI2-I2S2:
     WS -- PB12
     CK -- PB13
     SD -- PB15
     MCK-- PC6
*/

void I2S2Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2S_InitTypeDef  I2S_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  I2S_InitStructure.I2S_Mode = I2S_Mode_MasterTx;
  I2S_InitStructure.I2S_Standard = I2S_Standard_Phillips;
  I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
  I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Enable;
  I2S_InitStructure.I2S_AudioFreq = SAMPLE_RATE;
  I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
  I2S_Init(SPI2, &I2S_InitStructure);
  // calculate new i2s divider
  unsigned int i2sdiv = 144000000 / 256 / SAMPLE_RATE;
  unsigned int temp = SPI2->I2SPR & ~(SPI_I2SPR_I2SDIV|SPI_I2SPR_ODD);
  temp |= i2sdiv / 2;
  if (i2sdiv & 1)
    temp |= SPI_I2SPR_ODD;
  SPI2->I2SPR = temp;
  RCC->CFGR2 |= RCC_I2S2SRC; // i2s2 src = PLL3

  SPI_I2S_DMACmd( SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
  I2S_Cmd(SPI2, ENABLE);
}

void DMA_I2S2_Init(void)
{
  DMA_InitTypeDef DMA_InitStructure;

  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE );

  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI2->DATAR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)sound_out_buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = SOUND_OUT_BUFFER_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);

  // half transfer interrupt
  DMA_ITConfig(DMA1_Channel5, DMA_IT_HT, ENABLE);
  // transfer complete interrupt
  DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);

  NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

static void PLL3Init(unsigned int mul)
{
  unsigned int temp = RCC->CFGR2;
  temp &= ~RCC_PLL3MUL;
  RCC->CFGR2 = temp | mul;
  RCC->CTLR |= RCC_PLL3ON;
  while (!(RCC->CTLR & RCC_PLL3RDY))
    ;
}

/*
 * PB10 -> SCL
 * PB11 -> SDA
 */

static void I2C2Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitTSturcture;

  RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C2, ENABLE );

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( GPIOB, &GPIO_InitStructure );

  I2C_InitTSturcture.I2C_ClockSpeed = 100000;
  I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
  I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
  I2C_InitTSturcture.I2C_OwnAddress1 = 0;
  I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
  I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init( I2C2, &I2C_InitTSturcture );

  I2C_Cmd( I2C2, ENABLE );
}

static void TLVInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // TLV reset pin
  GPIO_InitStructure.GPIO_Pin = TLV_RESET_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(TLV_RESET_PORT, &GPIO_InitStructure);
}

void HalInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  Delay_Init();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

  PLL3Init(RCC_PLL3Mul_9); // pll3vco = 8 * 9 * 2 = 144 mhz
  GPIOInit();
  I2S2Init();
  DMA_I2S2_Init();
  I2C2Init();
  TLVInit();
}

int tlv320dac3100_read(unsigned char register_number, unsigned char *value)
{
  return 1;
}

int tlv320dac3100_write(unsigned char register_number, unsigned char value)
{
  unsigned char data[2];
  data[0] = register_number;
  data[1] = value;
  return i2c_write(I2C2, TLV320DAC3100_I2C_ADDRESS, data, 2, I2C_TIMEOUT);
}
