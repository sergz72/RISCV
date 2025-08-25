#ifndef USB_CDC_H
#define USB_CDC_H

void CDC_Rx(unsigned int size);

unsigned int CDC_Receive(unsigned char **buffer);
void CDC_Transmit(unsigned char *buffer, unsigned int length);
void CDC_ReceiveEnable(void);
void CDC_Init(void);

#endif
