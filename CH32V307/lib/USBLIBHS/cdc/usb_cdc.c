#include "board.h"
#include "usb_cdc.h"
#include "ch32v30x_usbhs_device.h"

static volatile unsigned int receive_size;

void CDC_Init(void)
{
  receive_size = 0;
}

void CDC_Rx(unsigned int size)
{
  receive_size = size;
}

unsigned int CDC_Receive(unsigned char **buffer)
{
  *buffer = USBHS_EP2_Rx_Buf;
  return receive_size;
}

void CDC_ReceiveEnable(void)
{
  USBHSD->UEP2_RX_CTRL = (USBHSD->UEP2_RX_CTRL & 0xFC) | USBHS_UEP_R_RES_ACK;
}

void CDC_Transmit(unsigned char *buffer, unsigned int length)
{
  while (length)
  {
    while (CDC_Tx_InProgress)
      __WFI();
    unsigned int l = length > DEF_USBD_FS_PACK_SIZE ? DEF_USBD_FS_PACK_SIZE : length;
    NVIC_DisableIRQ( USBHS_IRQn );
    CDC_Tx_InProgress = 1;
    memcpy(USBHS_EP2_Tx_Buf, buffer, l);
    USBHSD->UEP2_TX_LEN  = l;
    USBHSD->UEP2_TX_CTRL &= ~USBHS_UEP_T_RES_MASK;
    USBHSD->UEP2_TX_CTRL |= USBHS_UEP_T_RES_ACK;
    NVIC_EnableIRQ( USBHS_IRQn );
    length -= l;
    buffer += l;
  }
}
