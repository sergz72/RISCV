#ifndef _SYSCALLS_H
#define _SYSCALLS_H

#include <stdbool.h>

void osTaskSwitch(void);
void osExit(int code);
void osDelay(int ms);
void osLeds(bool on, unsigned int leds);

#endif
