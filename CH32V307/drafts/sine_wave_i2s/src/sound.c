#include "board.h"
#include "sound.h"
#include <string.h>
#include <generator.h>
#include <sine_table_16.h>
#include <ch32v30x.h>

unsigned int sound_out_buffer[SOUND_OUT_BUFFER_SIZE];

static SignalGenerator generator;

int sound_init(void)
{
  memset(sound_out_buffer, 0, sizeof(sound_out_buffer));
  generator_init(&generator, SAMPLE_RATE, sine_table_16, DEFAULT_TABLE_LENGTH);
  generator_set_frequency(&generator, 440);
  return 0;
}

// SPI3 stream
void __attribute__((interrupt("WCH-Interrupt-fast"))) DMA1_Channel5_IRQHandler(void)
{
  unsigned short *pbuffer = DMA1->INTFCR & DMA1_FLAG_HT5 ? (unsigned short*)sound_out_buffer : (unsigned short*)sound_out_buffer + SOUND_OUT_BUFFER_SIZE / 2;

  for (int i = 0; i < SOUND_OUT_BUFFER_SIZE / 4; i++)
  {
    unsigned short value = generator_calculate_next_value(&generator);
    *pbuffer++ = value;
    *pbuffer++ = value;
  }

  DMA1->INTFCR = 0xFFFFFFFF;
}
