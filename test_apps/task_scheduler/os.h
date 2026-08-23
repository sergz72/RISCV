#ifndef _OS_H
#define _OS_H

void osInit(unsigned int max_tasks, unsigned int task_stack_size);
int osTaskCreate(void (*main)(void));
void __attribute__((noreturn)) osExit(int code);
void osDelay(unsigned int ms);

#endif
