#include "os.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* saved registers(6):
push rbx
push rbp
push r12
push r13
push r14
push r15
*/
#define NUM_SAVED_REGISTERS 6

typedef struct
{
  unsigned long long int RSP;
  unsigned long long int sleep_to;
  bool is_active;
  void *stack;
} task_t;

void osTaskSwitch(task_t *task);

static task_t *tasks;
static unsigned int max_tasks;
static unsigned int task_stack_size;

task_t *current_task;

void osInit(unsigned int _max_tasks, unsigned int _task_stack_size)
{
  max_tasks = _max_tasks;
  task_stack_size = _task_stack_size;
  tasks = calloc(_max_tasks, sizeof(task_t));
  current_task = tasks;
  current_task->is_active = true;
}

int osTaskCreate(void (*main)(void))
{
  for (task_t *task = tasks; task < tasks + max_tasks; task++)
  {
    if (!task->is_active)
    {
      task->stack = malloc(task_stack_size);
      unsigned long long int end_stack = (unsigned long long int)task->stack + task_stack_size;
      task->RSP = end_stack - (NUM_SAVED_REGISTERS+1)*8;
      *((unsigned long long int *)(end_stack - 8)) = (unsigned long long int)main;
      task->sleep_to = 0;
      task->is_active = true;
      return 0;
    }
  }
  return 1;
}

void __attribute__((noreturn)) osExit(int code)
{
  current_task->is_active = false;
  if (current_task->stack)
    free(current_task->stack);
  osDelay(0);
  puts("should not happen");
  exit(1000);
}

void osDelay(unsigned int ms)
{
  struct timespec ts;
  if (clock_gettime(CLOCK_REALTIME_COARSE, &ts) == -1)
  {
    perror("clock_gettime");
    exit(1);
  }
  unsigned long long int now = (unsigned long long int)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
  current_task->sleep_to = now + ms;
  unsigned long long int min_sleep_to = ULLONG_MAX;
  task_t *min_sleep_to_task = nullptr;
  for (task_t *task = tasks; task < tasks + max_tasks; task++)
  {
    if (task->is_active && task->sleep_to < min_sleep_to)
    {
      min_sleep_to = task->sleep_to;
      min_sleep_to_task = task;
    }
  }
  if (min_sleep_to > now)
  {
    min_sleep_to -= now;
    ts.tv_sec = (__time_t)(min_sleep_to / 1000);
    ts.tv_nsec = (__syscall_slong_t)((min_sleep_to % 1000) * 1000000);
    nanosleep(&ts, nullptr);
  }
  if (min_sleep_to_task == current_task)
    return;
  osTaskSwitch(min_sleep_to_task);
}