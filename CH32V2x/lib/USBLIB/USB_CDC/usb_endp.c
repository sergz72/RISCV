/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_endp.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/08/08
 * Description        : Endpoint routines
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include <memory.h>
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_istr.h"
#include "usb_prop.h"
#include "usb_cdc.h"
#include "board.h"

uint8_t USBD_Endp3_Busy;
static unsigned int expected_free;

static unsigned char usb_cdc_buffer[USB_CDC_RX_BUFFER_SIZE], *usb_cdc_buffer_read_p;
static volatile unsigned char *usb_cdc_buffer_p;
static volatile unsigned int free;
static unsigned char endp_buffer[DEF_USBD_MAX_PACK_SIZE];

void USB_Endp_Init(void)
{
  usb_cdc_buffer_p = usb_cdc_buffer_read_p = usb_cdc_buffer;
  free = USB_CDC_RX_BUFFER_SIZE;
  expected_free = 0;
}

unsigned int USB_CDC_Receive(unsigned char *buffer_p, unsigned int buffer_size)
{
  unsigned int len;
  unsigned char *bp = usb_cdc_buffer_p;

  if (bp == usb_cdc_buffer_read_p)
    return 0;
  len = bp > usb_cdc_buffer_read_p
      ? bp - usb_cdc_buffer_read_p
      : usb_cdc_buffer + USB_CDC_RX_BUFFER_SIZE - usb_cdc_buffer_read_p;
  if (len > buffer_size)
    len = buffer_size;
  memcpy(buffer_p, usb_cdc_buffer_read_p, len);
  usb_cdc_buffer_read_p += len;
  if (usb_cdc_buffer_read_p == usb_cdc_buffer + USB_CDC_RX_BUFFER_SIZE)
    usb_cdc_buffer_read_p = usb_cdc_buffer;

  NVIC_DisableIRQ( USB_LP_CAN1_RX0_IRQn );
  NVIC_DisableIRQ( USB_HP_CAN1_TX_IRQn );

  free += len;
  if (expected_free != 0 && free >= expected_free)
  {
    SetEPRxValid(ENDP2);
    expected_free = 0;
  }

  NVIC_EnableIRQ( USB_LP_CAN1_RX0_IRQn );
  NVIC_EnableIRQ( USB_HP_CAN1_TX_IRQn );

  return len;
}

void USB_CDC_Transmit(unsigned char *buffer_p, unsigned int size)
{
  while (size)
  {
    unsigned int packlen = size > DEF_USBD_MAX_PACK_SIZE ? DEF_USBD_MAX_PACK_SIZE : size;
    while (USBD_ENDPx_DataUp(ENDP3, buffer_p, packlen) != USB_SUCCESS)
      ;
    size -= packlen;
    buffer_p += packlen;
  }
}

/*********************************************************************
 * @fn      EP2_IN_Callback
 *
 * @brief  Endpoint 1 IN.
 *
 * @return  none
 */
void EP1_IN_Callback(void) {

}


/*********************************************************************
 * @fn      EP2_OUT_Callback
 *
 * @brief  Endpoint 2 OUT.
 *
 * @return  none
 */
void EP2_OUT_Callback(void)
{
  unsigned int len = GetEPRxCount(EP2_OUT & 0x7F);
  if (len > free)
  {
    expected_free = len;
    return;
  }
  free -= len;
  unsigned int left_to_end = usb_cdc_buffer + USB_CDC_RX_BUFFER_SIZE - usb_cdc_buffer_p;
  if (left_to_end > len)
  {
    PMAToUserBufferCopy(usb_cdc_buffer_p, GetEPRxAddr(EP2_OUT & 0x7F), len);
    usb_cdc_buffer_p += len;
  }
  else
  {
    PMAToUserBufferCopy(endp_buffer, GetEPRxAddr(EP2_OUT & 0x7F), len);
    memcpy(usb_cdc_buffer_p, endp_buffer, left_to_end);
    usb_cdc_buffer_p = usb_cdc_buffer;
    len -= left_to_end;
    memcpy(usb_cdc_buffer_p, &endp_buffer[left_to_end], len);
    usb_cdc_buffer_p += len;
  }
  SetEPRxValid(ENDP2);
}

/*********************************************************************
 * @fn      EP3_IN_Callback
 *
 * @brief  Endpoint 3 IN.
 *
 * @return  none
 */
void EP3_IN_Callback(void) {
  USBD_Endp3_Busy = 0;
}

/*********************************************************************
 * @fn      USBD_ENDPx_DataUp
 *
 * @brief  USBD ENDPx DataUp Function
 * 
 * @param   endp - endpoint num.
 *          *pbuf - A pointer points to data.
 *          len - data length to transmit.
 * 
 * @return  data up status.
 */
uint8_t USBD_ENDPx_DataUp(uint8_t endp, uint8_t *pbuf, uint16_t len) {
  if (endp == ENDP3) {
    if (USBD_Endp3_Busy) {
      return USB_ERROR;
    }
    USB_SIL_Write(EP3_IN, pbuf, len);
    USBD_Endp3_Busy = 1;
    SetEPTxStatus(ENDP3, EP_TX_VALID);
  } else {
    return USB_ERROR;
  }
  return USB_SUCCESS;
}
