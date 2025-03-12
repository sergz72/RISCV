#include "board.h"

//static unsigned char usb_cdc_buffer[USB_CDC_RX_BUFFER_SIZE];

int main(void)
{
  //int i = 0;

  core_main();
  
  /*while (1)
  {
    Delay_Ms(100);
    GPIO_WriteBit(GPIOB, GPIO_Pin_2, (i == 0) ? (i = Bit_SET) : (i = Bit_RESET));
    unsigned int length = USB_CDC_Receive(usb_cdc_buffer, sizeof(usb_cdc_buffer));
    if (length)
      USB_CDC_Transmit(usb_cdc_buffer, length);
  }*/
}
