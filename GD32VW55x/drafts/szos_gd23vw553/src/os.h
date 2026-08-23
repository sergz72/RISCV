#ifndef _GD32VW553_OS_H
#define _GD32VW553_OS_H

void os_delay(void);
void ecall_handler(unsigned int a0, unsigned int a1, unsigned int a2, unsigned int a3, unsigned int a4, unsigned int a5,
                   unsigned int a6, unsigned int a7);
void nmi_handler(unsigned long mcause, unsigned long sp);

#endif
