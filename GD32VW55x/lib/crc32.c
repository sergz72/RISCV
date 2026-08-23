#include "board.h"
#include <crc32.h>
#include <string.h>

void crc32_init(void)
{
  rcu_periph_clock_enable(RCU_CRC);
}

void crc32_start(void)
{
  crc_data_register_reset();
}

void crc32_add(const void *data, unsigned int length)
{
  unsigned int v;
  const unsigned char *d = data;
  while (length >= 4)
  {
    memcpy(&v, d, 4);
    CRC_DATA = v;
    length -= 4;
    d += 4;
  }

  if (length)
  {
    v = 0;
    memcpy(&v, d, length);
    CRC_DATA = v;
  }
}

/*void crc32_add(const void *data, unsigned int length)
{
  const unsigned char *d = data;
  while (length--)
    CRC_DATA = *d++;
}*/

unsigned int crc32_end(void)
{
  return CRC_DATA;
}
