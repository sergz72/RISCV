#ifndef _BOARD_H
#define _BOARD_H

#ifndef NULL
#define NULL 0
#endif

#include <ch32x035.h>

//PB12
#define LED_PIN GPIO_Pin_12
#define LED_PORT GPIOB
#define LED_OFF GPIOB->BCR = LED_PIN
#define LED_ON GPIOB->BSHR = LED_PIN

#define USB_MAX_DEVICE_CONFIGURATIONS 1
#define USB_MAX_CONFIGURATIUON_INTERFACES 1
#define USB_MAX_INTERFACE_ENDPOINTS 2

#ifdef __cplusplus
extern "C" {
#endif
  void SysInit(void);
#ifdef __cplusplus
}
#endif


#endif
