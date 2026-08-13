#ifndef _HANDLERS_H
#define _HANDLERS_H

#include <stdint.h>

void Exception_Init(void);
void Exception_Register_EXC(uint32_t EXCn, unsigned long exc_handler);
void nmi_handler(unsigned long mcause, unsigned long sp);
uint32_t core_exception_handler(unsigned long mcause, unsigned long sp);

#endif //SZOS_GD32VW553_HANDLERS_H
