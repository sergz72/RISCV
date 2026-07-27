#ifndef USB_CDC_H
#define USB_CDC_H

void USB_CDC_Transmit(unsigned char *buffer_p, unsigned int size);
unsigned int USB_CDC_Receive(unsigned char *buffer_p, unsigned int buffer_size);
void USB_Endp_Init(void);

#endif
