#include "board.h"
#include "93xx_commands.h"
#include "i2c_commands.h"
#include <stdlib.h>
#include <shell.h>
#include <93cXX_16.h>

static int wren_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem wren_command_items[] = {
  {NULL, NULL, wren_handler}
};
static const ShellCommand wren_command = {
  wren_command_items,
  "93xx_wren",
  "93xx_wren"
};

static int write_random_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem write_random_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, NULL, write_random_handler}
};
static const ShellCommand write_random_command = {
  write_random_command_items,
  "93xx_write_random",
  "93xx_write_random address num_bytes"
};

static int check_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem check_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, NULL, check_handler}
};
static const ShellCommand check_command = {
  check_command_items,
  "93xx_check",
  "93xx_check address num_bytes"
};

static int read_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem read_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, NULL, read_handler}
};
static const ShellCommand read_command = {
  read_command_items,
  "93xx_read",
  "93xx_read address num_bytes"
};

static int erase_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem erase_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, NULL, erase_handler}
};
static const ShellCommand erase_command = {
  erase_command_items,
  "93xx_erase",
  "93xx_erase address num_bytes"
};

static int wren_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  enter_93xx_mode();
  _93CXX_16_write_enable(0);
  return 0;
}

static int write_random_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  unsigned long int address = strtoul(argv[0], NULL, 16);

  int nbytes = atoi(argv[1]);
  if (nbytes <= 0)
  {
    pfunc("Invalid data size\r\n");
    return 1;
  }
  pfunc("num_bytes %d\n", nbytes);
  iv[0]++;
  chacha20_zero(&rng, (const uint32_t*)iv);
  enter_93xx_mode();
  for (int i = 0; i < nbytes; i++)
  {
    unsigned short v = chacha_u16(&rng);
    int rc = _93CXX_16_write(0, address++, v, _93XX_TIMEOUT);
    if (rc)
      return rc;
  }
  return 0;
}

static int check_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  unsigned long int address = strtoul(argv[0], NULL, 16);

  int nbytes = atoi(argv[1]);
  if (nbytes <= 0)
  {
    pfunc("Invalid data size\r\n");
    return 1;
  }
  pfunc("num_bytes %d\n", nbytes);
  chacha20_zero(&rng, (const uint32_t*)iv);
  enter_93xx_mode();
  for (int i = 0; i < nbytes; i++)
  {
    unsigned short v;
    unsigned short expected = chacha_u16(&rng);
    _93CXX_16_read(0, address++, &v);
    if (v != expected)
      return 2;
  }
  return 0;
}

static int read_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  unsigned long int address = strtoul(argv[0], NULL, 16);

  int nbytes = atoi(argv[1]);
  if (nbytes <= 0 || nbytes > MEMORY_BUFFER_SIZE)
  {
    pfunc("Invalid data size\r\n");
    return 1;
  }
  enter_93xx_mode();
  unsigned short *p = (unsigned short*)memory_buffer;
  for (int i = 0; i < nbytes; i++)
    _93CXX_16_read(0, address++, p++);
  print_hex_buffer(memory_buffer, nbytes * 2, pfunc);
  return 0;
}

static int erase_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  unsigned long int address = strtoul(argv[0], NULL, 16);

  int nbytes = atoi(argv[1]);
  if (nbytes <= 0)
  {
    pfunc("Invalid data size\r\n");
    return 1;
  }
  enter_93xx_mode();
  for (int i = 0; i < nbytes; i++)
  {
    int rc = _93CXX_16_erase(0, address++, _93XX_TIMEOUT);
    if (rc)
      return rc;
  }
  return 0;
}

void register_93xx_commands(void)
{
  shell_register_command(&wren_command);
  shell_register_command(&read_command);
  shell_register_command(&write_random_command);
  shell_register_command(&check_command);
  shell_register_command(&erase_command);
}
