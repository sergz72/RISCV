#include "board.h"
#include "usb_cdc.h"
#include "ch32x035_usbfs_device.h"

static unsigned char cdc_rx_buffer[CDC_RX_BUF_LEN];
static unsigned char *cdc_rx_buffer_write_p, *cdc_rx_buffer_read_p;

void CDC_Init(void)
{
  cdc_rx_buffer_write_p = cdc_rx_buffer_read_p = cdc_rx_buffer;
}

void CDC_Rx(unsigned int size)
{
  //todo
  unsigned char *endp = cdc_rx_buffer_write_p + size;
  unsigned int l = endp - cdc_rx_buffer > CDC_RX_BUF_LEN ? &cdc_rx_buffer[CDC_RX_BUF_LEN] - cdc_rx_buffer_write_p : size;
  unsigned char *p = CDC_Rx_Buf;
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
  while (length)
  {
    unsigned int l = length > DEF_USBD_FS_PACK_SIZE ? DEF_USBD_FS_PACK_SIZE : length;
    NVIC_DisableIRQ( USBFS_IRQn );
    CDC_Tx_InProgress = 1;
    USBFS_Endp_DataUp(DEF_UEP3, buffer, l, DEF_UEP_CPY_LOAD);
    NVIC_EnableIRQ( USBFS_IRQn );
    length -= l;
    if (length)
    {
      buffer += l;
      while (CDC_Tx_InProgress)
        ;
    }
  }
}
