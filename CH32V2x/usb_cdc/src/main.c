#include "board.h"
#include "debug.h"
#include "usb_cdc.h"
#include "hw_config.h"
#include "usb_init.h"
#include "usb_pwr.h"
#include "delay.h"

static unsigned char usb_cdc_buffer[USB_CDC_RX_BUFFER_SIZE];

int main(void)
{
  int i = 0;

  fSuspendEnabled = FALSE;

  HalInit();

  Set_USBConfig();
  USB_Init();
  USB_Interrupts_Config();

  USB_Endp_Init();

  while (1)
  {
    Delay_Ms(100);
    GPIO_WriteBit(GPIOB, GPIO_Pin_2, (i == 0) ? (i = Bit_SET) : (i = Bit_RESET));
    unsigned int length = USB_CDC_Receive(usb_cdc_buffer, sizeof(usb_cdc_buffer));
    if (length)
      USB_CDC_Transmit(usb_cdc_buffer, length);
  }
}
