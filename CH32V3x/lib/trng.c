#include "board.h"
#include <trng.h>

void trng_generate(unsigned int *data, unsigned int length)
{
  while (length--)
  {
    while(!RNG_GetFlagStatus(RNG_FLAG_DRDY))
      ;
    *data++ = RNG_GetRandomNumber();
  }
}
