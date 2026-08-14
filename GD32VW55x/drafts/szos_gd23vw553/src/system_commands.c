#include "board.h"
#include "system_commands.h"
#include <shell.h>
#include <string.h>
#include "pmp.h"

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

#define UMODE_STACK_SIZE 2048

/* Create a stack for user mode execution */
uint8_t umode_stack[UMODE_STACK_SIZE] __attribute__((aligned(16)));
uintptr_t umode_sp = (uintptr_t) (umode_stack + sizeof(umode_stack));

static int reboot_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  pfunc("Rebooting...\r\n");
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
    pfunc("Unknown CPU mode - please specify m or u\r\n");
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
  pfunc("Unknown test parameter %s - please specify ii or sw or ecall\r\n", argv[1]);
  return 1;
}

void register_system_commands(void)
{
  pmp_init();
  shell_register_command(&reboot_command);
  shell_register_command(&test_command);
}
