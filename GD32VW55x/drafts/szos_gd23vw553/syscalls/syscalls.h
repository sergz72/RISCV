#ifndef _SYSCALLS_H
#define _SYSCALLS_H

void osExit(int code);
void osDelay(int ms);
void osLeds(bool on, unsigned int leds);

#endif
