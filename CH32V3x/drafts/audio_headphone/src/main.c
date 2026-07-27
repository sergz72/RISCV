/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2022/08/20
* Description        : Main program body.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "ch32v30x_usbfs_device.h"
#include "uac10_headphone.h"
#include "debug.h"
#include "delay.h"
#include "tlv.h"
#include "board.h"

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
  SystemCoreClockUpdate();

  HalInit();

  if (tlv_init())
  {
    LED_RED_ON;
    while (1)
      Delay_Ms(100);
  }

  I2SInit();

  /* USBFSD device init */
  USBFS_RCC_Init();
  USBFS_Device_Init(ENABLE);

  while (1)
  {
    //__WFI();
    //AUDIO_ChangesHandler();
  }
}
