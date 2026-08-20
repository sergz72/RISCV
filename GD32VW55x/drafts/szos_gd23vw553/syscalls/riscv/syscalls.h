#ifndef _SYSCALLS_H
#define _SYSCALLS_H

#include <stdbool.h>

static inline void osTaskSwitch(void)
{
  __asm__ __volatile__ (
      "li a7, 0\n"
      "ecall"
      : /* No output operands */
      : /* No input operands */
      : "a7" /* Clobber list: tells the compiler we modified a7 */
  );
}

static inline void __attribute__((noreturn)) osExit(int code)
{
  register int r_a0 __asm__("a0") = code;

  __asm__ __volatile__ (
      "li a7, 1\n"
      "ecall"
      : /* No output operands */
      : "r" (r_a0)
      : "a7" /* Clobber list: tells the compiler we modified a7 */
  );
}

static inline void osDelay(int ms)
{
  register int r_a0 __asm__("a0") = ms;

  __asm__ __volatile__ (
      "li a7, 2\n"
      "ecall"
      : /* No output operands */
      : "r" (r_a0)
      : "a7" /* Clobber list: tells the compiler we modified a7 */
  );
}

static inline void osLeds(bool on, unsigned int leds)
{
  register bool r_a0 __asm__("a0") = on;
  register unsigned int r_a1 __asm__("a1") = leds;

  __asm__ __volatile__ (
      "li a7, 2\n"
      "ecall"
      : /* No output operands */
      : "r" (r_a0), "r" (r_a1)
      : "a7" /* Clobber list: tells the compiler we modified a7 */
  );
}

#endif
