#include "board.h"
#include <common_printf.h>
#include "os.h"

#include <limits.h>

#include "exceptions.h"
#include "sys_timer.h"

/**
 * @brief Performs an atomic 64-bit load on RV32.
 * @param src Pointer to the aligned 64-bit target memory location.
 * @return The atomically loaded 64-bit value.
 */
static inline unsigned long long int atomic_load_64(const unsigned long long int *src) {
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
  current_task_data = min_sleep_to_task;
}

void ecall_handler(unsigned int a0, unsigned int a1, unsigned int a2, unsigned int a3, unsigned int a4, unsigned int a5,
                   unsigned int a6, unsigned int a7)
{
  switch (a7)
  {
  default:
    PRINTF("Unknown environment call %x from U-mode. Rebooting...\n", a7);
    reboot();
  }
}

void nmi_handler(unsigned long mcause, unsigned long sp)
{
}
