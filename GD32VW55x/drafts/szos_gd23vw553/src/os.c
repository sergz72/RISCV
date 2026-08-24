#include "board.h"
#include <common_printf.h>
#include "os.h"
#include <limits.h>
#include <stdlib.h>
#include "exceptions.h"
#include "sys_timer.h"
#include <syscalls.h>
#include "pmp.h"
#include <core_feature_cache.h>

/**
 * @brief Performs an atomic 64-bit load on RV32.
 * @param src Pointer to the aligned 64-bit target memory location.
 * @return The atomically loaded 64-bit value.
 */
static inline unsigned long long int atomic_load_64(const volatile unsigned long long int *src) {
  union
  {
    struct
    {
      unsigned int low, high;
    };
    unsigned long long int v64;
  } v;
  unsigned int tmp;

  __asm__ __volatile__ (
      "1:\n"
      "fence r, r\n"         // Memory barrier
      "lw    %0, 0(%3)\n"    // Load lower 32 bits
      "lw    %1, 4(%3)\n"    // Load upper 32 bits
      "fence r, r\n"         // Memory barrier
      "lw    %2, 0(%3)\n"    // Load lower 32 bits
      "bne   %0, %2, 1b\n"   // If is not the same - retry loop
      : "=&r" (v.low), "=&r" (v.high), "=&r" (tmp)
      : "r" (src)
      :
  );

  return v.v64;
}

void os_delay(void)
{
  volatile unsigned long long int now = atomic_load_64(&time_since_boot);
  current_task_data->sleep_to = now + current_task_data->registers[9]; // a0

  unsigned long long int min_sleep_to = ULLONG_MAX;
  task_data *min_sleep_to_task = nullptr;
  for (task_data *task = tasks; task < tasks + MAX_TASKS; task++)
  {
    if (task->is_active && task->sleep_to < min_sleep_to)
    {
      min_sleep_to = task->sleep_to;
      min_sleep_to_task = task;
    }
  }
  if (min_sleep_to > now)
  {
    unsigned int ms = (unsigned int)(min_sleep_to -= now);
    delayms(ms);
  }
  if (min_sleep_to_task->image)
    pmp_init_user((unsigned int)min_sleep_to_task->image, min_sleep_to_task->image_size, min_sleep_to_task->text_size);
  current_task_data = min_sleep_to_task;
}

int os_create_task(const os_task_t *task)
{
  for (task_data *tdata = tasks; tdata < tasks + MAX_TASKS; tdata++)
  {
    if (!tdata->is_active)
    {
      tdata->image = task->image;
      tdata->image_size = task->image_size;
      tdata->text_size = task->text_size;
      tdata->registers[1] = (unsigned int)task->image + task->image_size; // x2(sp)
      tdata->registers[9] = task->argc; // a0
      tdata->registers[10] = (unsigned int)task->argv; // a1
      tdata->mepc = (unsigned int)task->entry - 4;
      tdata->mstatus = __RV_CSR_READ(CSR_MSTATUS);
      tdata->mstatus = __RV_INSERT_FIELD(tdata->mstatus, MSTATUS_MPP, PRV_U);
      /* Set previous MIE disabled */
      tdata->mstatus = __RV_INSERT_FIELD(tdata->mstatus, MSTATUS_MPIE, 0);
      tdata->sleep_to = 0;
      tdata->is_active = true;
      return 0;
    }
  }
  return 1;
}

void os_end_task(task_data *task)
{
  task->is_active = false;
  if (task->image)
  {
    free(task->image);
    task->image = nullptr;
  }
}

void ecall_handler(unsigned int a0, unsigned int a1, unsigned int a2, unsigned int a3, unsigned int a4, unsigned int a5,
                   unsigned int a6, unsigned int a7)
{
  switch (a7)
  {
  case 1: // osExit
    os_end_task(current_task_data);
    osDelay(0);
    break;
  case 2: // osLeds
    puts_("osLeds ecall\n");
    break;
  default:
    PRINTF("Unknown environment call %x from U-mode. Rebooting...\n", a7);
    reboot();
  }
}

void nmi_handler(unsigned long mcause, unsigned long sp)
{
}
