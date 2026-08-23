#include "board.h"
#include <trng.h>

int trng_init(void)
{
  /* enable TRNG module clock */
  rcu_periph_clock_enable(RCU_TRNG);

  /* reset TRNG */
  trng_deinit();
  trng_enable();

  return 0;
}

int trng_generate(unsigned int *data, unsigned int length)
{
  while (length)
  {
    if (TRNG_STAT & (TRNG_STAT_CECS | TRNG_STAT_SECS))
      return 1;
    if (TRNG_STAT & TRNG_STAT_DRDY)
    {
      length--;
      *data++ = TRNG_DATA;
    }
  }
  return 0;
}
