#include "board.h"
#include "debug.h"
#include "ch32v30x_usbfs_device.h"
#include "delay.h"
#include <usb_cdc.h>

static unsigned char usb_cdc_buffer[USB_CDC_RX_BUFFER_SIZE];

int main(void)
{
  int i = 0;

  HalInit();

  USBFS_RCC_Init( );
  USBFS_Device_Init( ENABLE );

  while (1)
  {
    Delay_Ms(100);
    GPIO_WriteBit(GPIOB, GPIO_Pin_4, (i == 0) ? (i = Bit_SET) : (i = Bit_RESET));
    unsigned int length = CDC_Receive(usb_cdc_buffer, sizeof(usb_cdc_buffer));
    if (length)
      CDC_Transmit(usb_cdc_buffer, length);
  }
}
