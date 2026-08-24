#ifndef _GD32VW553_OS_H
#define _GD32VW553_OS_H

#include "exceptions.h"

typedef struct
{
  char *name;
  void *image;
  unsigned int image_size;
  unsigned int text_size;
  void (*entry)(int argc, const char **argv);
  int argc;
  const char **argv;
} os_task_t;

void ecall_handler(unsigned int a0, unsigned int a1, unsigned int a2, unsigned int a3, unsigned int a4, unsigned int a5,
                   unsigned int a6, unsigned int a7);
void nmi_handler(unsigned long mcause, unsigned long sp);

void os_delay(void);
int os_create_task(const os_task_t *task);
void os_end_task(task_data *task);

#endif
