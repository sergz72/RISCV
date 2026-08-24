#ifndef _EXCEPTIONS_H
#define _EXCEPTIONS_H

#define EXCEPTION_STACK_SIZE 1024

#ifndef __ASSEMBLER__
#include "board.h"

typedef struct
{
  unsigned int mcause;
  unsigned int registers[31];
  unsigned int mepc;
  unsigned int mstatus;
  bool is_active;
  unsigned long long int sleep_to;
  void *image;
  unsigned int image_size;
  unsigned int text_size;
  char name[256];
} task_data;

extern task_data tasks[MAX_TASKS];
extern task_data *current_task_data;
#endif

#endif
