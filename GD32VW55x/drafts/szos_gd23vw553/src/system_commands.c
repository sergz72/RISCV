#include "board.h"
#include "system_commands.h"
#include "fs_commands.h"
#include <string.h>
#include <getstring.h>
#include "pmp.h"
#include <malloc.h>
#include <stdlib.h>
#include <elf_file_loader.h>
#include "os.h"
#include <common_printf.h>

static const function_def function_map[] = {
  {.name = "printf", .pointer = common_printf}
};

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

static int run_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem run_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, param_handler, run_handler},
  {nullptr, param_handler, run_handler},
  {nullptr, param_handler, run_handler},
  {nullptr, param_handler, run_handler},
  {nullptr, param_handler, run_handler},
  {nullptr, param_handler, run_handler},
  {nullptr, param_handler, run_handler},
  {nullptr, param_handler, run_handler},
  {nullptr, nullptr, run_handler}
};
static const ShellCommand run_command = {
  run_command_items,
  "run",
  "run file_name [parameters]",
  nullptr,
  nullptr
};

#define UMODE_STACK_SIZE 2048

/* Create a stack for user mode execution */
uint8_t umode_stack[UMODE_STACK_SIZE] __attribute__((aligned(16)));
uintptr_t umode_sp = (uintptr_t) (umode_stack + sizeof(umode_stack));
volatile unsigned int v;

static int reboot_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  pfunc("Rebooting...\n");
  reboot();
}

static void __attribute((naked)) trigger_illegal_instruction_exception(void)
{
  __ASM volatile("li x1, 1");
  __ASM volatile("li x2, 2");
  __ASM volatile("li x3, 3");
  __ASM volatile("li x4, 4");
  __ASM volatile("li x5, 5");
  __ASM volatile("li x6, 6");
  __ASM volatile("li x7, 7");
  __ASM volatile("li x8, 8");
  __ASM volatile("li x9, 9");
  __ASM volatile("li x10, 10");
  __ASM volatile("li x11, 11");
  __ASM volatile("li x12, 12");
  __ASM volatile("li x13, 13");
  __ASM volatile("li x14, 14");
  __ASM volatile("li x15, 15");
  __ASM volatile("li x16, 16");
  __ASM volatile("li x17, 17");
  __ASM volatile("li x18, 18");
  __ASM volatile("li x19, 19");
  __ASM volatile("li x20, 20");
  __ASM volatile("li x21, 21");
  __ASM volatile("li x22, 22");
  __ASM volatile("li x23, 23");
  __ASM volatile("li x24, 24");
  __ASM volatile("li x25, 25");
  __ASM volatile("li x26, 26");
  __ASM volatile("li x27, 27");
  __ASM volatile("li x28, 28");
  __ASM volatile("li x29, 29");
  __ASM volatile("li x30, 30");
  __ASM volatile("li x31, 31");
  __ASM volatile(".word 0xffffffff");
}

static void __attribute((naked)) trigger_ecall_exception(void)
{
  v = 0;
  __ASM volatile("li a7, 64");
  __ASM volatile("ecall");
}

static void __attribute((naked)) trigger_task_switch(void)
{
  __ASM volatile("mv a7, zero");
  __ASM volatile("ecall");
  __ASM volatile("li a7, 10");
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

void *elf_file_alloc(unsigned int size, unsigned int text_size)
{
  return aligned_alloc(text_size, size);
}

void elf_file_free(void *p, unsigned int size)
{
  free(p);
}

static int run_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  const char *outpath;
  const storage_t *storage = get_storage(pfunc, argc, argv[0], &outpath);
  if (storage == nullptr)
    return 1;
  void *f = storage->fs_operations->fopen(storage->fs_context, outpath, "r");
  if (f == nullptr)
  {
    pfunc("Failed to open file for read\n");
    return 2;
  }
  unsigned int size;
  int rc = storage->fs_operations->file_size(storage->fs_context, f, &size);
  if (rc)
    return rc;
  void *p = malloc(size);
  if (p == nullptr)
  {
    storage->fs_operations->fclose(storage->fs_context, f);
    pfunc("Failed to allocate memory\n");
    return 3;
  }
  int read_size = storage->fs_operations->fread(storage->fs_context, p, size, 1, f);
  if (read_size != size)
  {
    storage->fs_operations->fclose(storage->fs_context, f);
    free(p);
    pfunc("Failed to read file\n");
    return 4;
  }
  storage->fs_operations->fclose(storage->fs_context, f);
  app_image image;
  rc = elf_file_load(p, function_map, sizeof(function_map) / sizeof(function_def), 2048, argc, (const char**)argv, &image);
  if (rc)
  {
    free(p);
    pfunc("Failed to load file\n");
    return rc;
  }
  free(p);
  pfunc("Image base=0x%08X size=%d(0x%08X), text_size=%d(0x%08X)\n", image.address, image.size, image.size, image.text_size, image.text_size);
  os_task_t task;
  task.image = image.address;
  task.argc = argc;
  task.argv = image.argvp;
  task.entry = image.main;
  task.image_size = image.size;
  task.text_size = image.text_size;
  rc = os_create_task(&task);
  if (rc)
  {
    pfunc("Failed to create task\n");
    elf_file_free(image.address, image.size);
  }
  return rc;
}

void register_system_commands(void)
{
  pmp_init_flash_rx_ram_rw();
  shell_register_command(&reboot_command);
  shell_register_command(&test_command);
  shell_register_command(&echo_command);
  shell_register_command(&memalloc_command);
  shell_register_command(&run_command);
}
