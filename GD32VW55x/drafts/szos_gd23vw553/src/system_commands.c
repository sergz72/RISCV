#include "board.h"
#include "system_commands.h"
#include <shell.h>
#include <string.h>
#include <getstring.h>
#include "pmp.h"
#include <malloc.h>
#include <stdlib.h>

static int reboot_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem reboot_command_items[] = {
  {nullptr, nullptr, reboot_handler}
};
static const ShellCommand reboot_command = {
  reboot_command_items,
  "reboot",
  "reboot",
  nullptr,
  nullptr
};

static int memalloc_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem memalloc_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, memalloc_handler}
};
static const ShellCommand memalloc_command = {
  memalloc_command_items,
  "memalloc",
  "memalloc bytes",
  nullptr,
  nullptr
};

static int test_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem test_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, test_handler}
};
static const ShellCommand test_command = {
  test_command_items,
  "test",
  "test parameters",
  nullptr,
  nullptr
};

static int echo_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem echo_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, echo_handler}
};
static const ShellCommand echo_command = {
  echo_command_items,
  "echo",
  "echo on|off",
  nullptr,
  nullptr
};

#define UMODE_STACK_SIZE 2048

/* Create a stack for user mode execution */
uint8_t umode_stack[UMODE_STACK_SIZE] __attribute__((aligned(16)));
uintptr_t umode_sp = (uintptr_t) (umode_stack + sizeof(umode_stack));

static int reboot_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  pfunc("Rebooting...\n");
  reboot();
}

static void trigger_illegal_instruction_exception(void)
{
  __ASM volatile(".word 0xffffffff");
}

static void trigger_ecall_exception(void)
{
  __ASM volatile("li a7, 64");
  __ASM volatile("ecall");
}

static void trigger_task_switch(void)
{
  __ASM volatile("mv a7, zero");
  __ASM volatile("ecall");
  __ASM volatile("li a7, 1");
  __ASM volatile("ecall");
}

static int call(bool user_mode, void (*f)(void))
{
  if (user_mode)
    __switch_mode(PRV_U, umode_sp, f);
  else
    f();

  return 1;
}
static int test_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  size_t l = strlen(argv[0]);
  bool user_mode = argv[0][0] == 'u';
  if (l != 1 || (!user_mode && argv[0][0] != 'm'))
  {
    pfunc("Unknown CPU mode - please specify m or u\n");
    return 1;
  }
  l = strlen(argv[1]);
  switch (l)
  {
    case 2:
      if (!strcmp(argv[1], "ii"))
        return call(user_mode, trigger_illegal_instruction_exception);
      if (!strcmp(argv[1], "sw"))
        return call(user_mode, trigger_task_switch);
      break;
    case 5:
      if (!strcmp(argv[1], "ecall"))
        return call(user_mode, trigger_ecall_exception);
      break;
    default:
      break;
  }
  pfunc("Unknown test parameter %s - please specify ii or sw or ecall\n", argv[1]);
  return 1;
}

static int echo_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  if (!strcmp(argv[0], "on"))
    getstring_echo(true);
  else if (!strcmp(argv[0], "off"))
    getstring_echo(false);
  else
  {
    pfunc("Unknown echo parameter %s - please specify on or off\n", argv[0]);
    return 1;
  }
  return 0;
}

static int memalloc_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  int bytes = atoi(argv[0]);
  if (bytes < 0)
  {
    pfunc("Incorrect number of bytes");
    return 1;
  }
  if (bytes != 0)
  {
    void *p = malloc(bytes);
    pfunc("Allocated pointer %08X\n", p);
    free(p);
  }
  struct mallinfo mi = mallinfo();
  pfunc("Total space allocated from system: %d bytes\n", mi.arena);
  pfunc("Total allocated space: %d bytes\n", mi.uordblks);
  pfunc("Total free space: %d bytes\n", mi.fordblks);
  pfunc("Number of mmapped regions: %d\n", mi.hblks);
  return 0;
}

void register_system_commands(void)
{
  pmp_init();
  shell_register_command(&reboot_command);
  shell_register_command(&test_command);
  shell_register_command(&echo_command);
  shell_register_command(&memalloc_command);
}
