#include "board.h"
#include "usart_handler.h"

void usart_handler(void)
{
  if (command)
  {
    switch (command)
    {
      default:
        usart_transmit('e');
        break;
    }
    command = 0;
  }
}
