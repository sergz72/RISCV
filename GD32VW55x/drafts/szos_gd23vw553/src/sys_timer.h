#ifndef _GD32VW553_SYS_TIMER_H
#define _GD32VW553_SYS_TIMER_H

void sys_timer_init(void);
void delayms(unsigned int ms);
void delay(unsigned int us);

extern volatile unsigned long long int time_since_boot;

#endif
