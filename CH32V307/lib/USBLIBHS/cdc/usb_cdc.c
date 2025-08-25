#include "board.h"
#include "usb_cdc.h"
#include "ch32v30x_usbhs_device.h"

static unsigned char cdc_rx_buffer[CDC_RX_BUF_LEN];
static unsigned char *cdc_rx_buffer_write_p, *cdc_rx_buffer_read_p;

void CDC_Init(void)
{
  cdc_rx_buffer_write_p = cdc_rx_buffer_read_p = cdc_rx_buffer;
}

void CDC_Rx(unsigned int size)
{
  unsigned char *endp = cdc_rx_buffer_write_p + size;
  unsigned int l = endp - cdc_rx_buffer > CDC_RX_BUF_LEN ? &cdc_rx_buffer[CDC_RX_BUF_LEN] - cdc_rx_buffer_write_p : size;
  unsigned char *p = USBHS_EP2_Rx_Buf;
  memcpy(cdc_rx_buffer_write_p, p, l);
  size -= l;
  if (size)
  {
    p += l;
    cdc_rx_buffer_write_p = cdc_rx_buffer;
    memcpy(cdc_rx_buffer_write_p, p, size);
    cdc_rx_buffer_write_p += size;
  }
  else
  {
    cdc_rx_buffer_write_p += l;
    if (cdc_rx_buffer_write_p >= &cdc_rx_buffer[CDC_RX_BUF_LEN])
      cdc_rx_buffer_write_p = cdc_rx_buffer;
  }
}

unsigned int CDC_Receive(unsigned char *buffer, unsigned int buffer_size)
{
  unsigned int l = 0;
  while (buffer_size && cdc_rx_buffer_read_p != cdc_rx_buffer_write_p)
  {
    *buffer++ = *cdc_rx_buffer_read_p++;
    if (cdc_rx_buffer_read_p >= &cdc_rx_buffer[CDC_RX_BUF_LEN])
      cdc_rx_buffer_read_p = cdc_rx_buffer;
    buffer_size--;
    l++;
  }
  return l;
}

void CDC_Transmit(unsigned char *buffer, unsigned int length)
{
  int send_zero = 0;

  while (length != 0 || send_zero)
  {
    while (CDC_Tx_InProgress)
      ;
    unsigned int l = length > DEF_USBD_HS_PACK_SIZE ? DEF_USBD_HS_PACK_SIZE : length;
    NVIC_DisableIRQ( USBHS_IRQn );
    CDC_Tx_InProgress = 1;
    memcpy(USBHS_EP2_Tx_Buf, buffer, l);
    USBHSD->UEP2_TX_LEN  = l;
    USBHSD->UEP2_TX_CTRL &= ~USBHS_UEP_T_RES_MASK;
    USBHSD->UEP2_TX_CTRL |= USBHS_UEP_T_RES_ACK;
    NVIC_EnableIRQ( USBHS_IRQn );
    length -= l;
    buffer += l;
    send_zero = l == DEF_USBD_HS_PACK_SIZE;
  }
}
