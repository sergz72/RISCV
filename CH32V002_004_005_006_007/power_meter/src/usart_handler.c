#include "board.h"
#include "usart_handler.h"
#include "ui.h"
#include "common_printf.h"
#include <spi_memory.h>

void usart_handler(void)
{
  if (command)
  {
    switch (command)
    {
      case 'r': // start recording
        if (StartRecording())
          usart_transmit('e');
        else
          usart_transmit('k');
        break;
      case 's': // stop recording
        if (StopRecording())
          usart_transmit('e');
        else
          usart_transmit('k');
        break;
      case 'l': // start logging
        if (StartLogging())
          usart_transmit('e');
        else
          usart_transmit('k');
        break;
      case 'k': // stop logging
        if (StopLogging())
          usart_transmit('e');
        else
          usart_transmit('k');
        break;
      case 'i': // read flash id
        unsigned int id;
        int rc = flash_read_id(0, &id);
        if (rc)
          puts_("Flash ID read failed\n");
        else
          common_printf("Flash ID: 0x%08X\n", id);
        break;
      default:
        usart_transmit('e');
        break;
    }
    command = 0;
  }
}
