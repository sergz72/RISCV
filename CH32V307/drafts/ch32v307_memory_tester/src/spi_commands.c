#include "board.h"
#include <shell.h>
#include "spi_commands.h"
#include "i2c_commands.h"
#include <spi_memory.h>
#include <chacha.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

typedef struct
{
  int address_length;
  int max_data_length;
  void (*reset)(int channel);
  int (*enter_qspi_mode)(int channel, int check_sr2);
  int (*exit_qspi_mode)(int channel);
  int (*read_id)(int channel, unsigned int *id);
  int (*wren)(int channel);
  int (*read)(int channel, unsigned int address, unsigned char *buffer, int count);
  void (*write)(int channel, int page_size, unsigned int address, unsigned char *buffer, int count);
  int (*read_callback)(int channel, unsigned int address, int (*set_byte)(unsigned char c), int count);
  void (*write_callback)(int channel, int page_size, unsigned int address, unsigned char (*next_byte)(void), int count);
  void (*erase)(int channel, enum spi_memory_erase_command command, unsigned int address);
  unsigned char (*read_sr1)(int channel);
  unsigned char (*read_sr2)(int channel);
} spi_memory;

static const spi_memory flash = {
  .address_length = 3,
  .max_data_length = INT_MAX,
  .reset = flash_reset,
  .enter_qspi_mode = flash_enter_qspi_mode,
  .exit_qspi_mode = flash_exit_qspi_mode,
  .read_id = flash_read_id,
  .wren = flash_wren,
  .read = flash_fast_read,
  .write = flash_write,
  .read_callback = flash_fast_read_cb,
  .write_callback = flash_write_cb,
  .erase = flash_erase,
  .read_sr1 = flash_read_sr1,
  .read_sr2 = flash_read_sr2
};

static const spi_memory *device = &flash;

static int read_id_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem read_id_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, NULL, read_id_handler}
};
static const ShellCommand read_id_command = {
  read_id_command_items,
  "spi_mem_read_id",
  "spi_mem_read_id channel"
};

static int read_sr1_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem read_sr1_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, NULL, read_sr1_handler}
};
static const ShellCommand read_sr1_command = {
  read_sr1_command_items,
  "spi_mem_read_sr1",
  "spi_mem_read_sr1 channel"
};

static int read_sr2_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem read_sr2_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, NULL, read_sr2_handler}
};
static const ShellCommand read_sr2_command = {
  read_sr2_command_items,
  "spi_mem_read_sr2",
  "spi_mem_read_sr2 channel"
};

static int wren_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem wren_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, NULL, wren_handler}
};
static const ShellCommand wren_command = {
  wren_command_items,
  "spi_mem_wren",
  "spi_mem_wren channel"
};

static int write_random_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem write_random_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, NULL, write_random_handler}
};
static const ShellCommand write_random_command = {
  write_random_command_items,
  "spi_mem_write_random",
  "spi_mem_write_random channel address num_bytes"
};

static int check_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem check_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, NULL, check_handler}
};
static const ShellCommand check_command = {
  check_command_items,
  "spi_mem_check",
  "spi_mem_check channel address num_bytes"
};

static int read_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem read_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, NULL, read_handler}
};
static const ShellCommand read_command = {
  read_command_items,
  "spi_mem_read",
  "spi_mem_read channel address num_bytes"
};

static int erase_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem erase_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, param_handler, erase_handler},
  {NULL, NULL, erase_handler}
};
static const ShellCommand erase_command = {
  erase_command_items,
  "spi_mem_erase",
  "spi_mem_erase channel chip|sector|block32|block64 [address]"
};

static int enter_qspi_mode_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem enter_qspi_mode_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, NULL, enter_qspi_mode_handler}
};
static const ShellCommand enter_qspi_mode_command = {
  enter_qspi_mode_command_items,
  "spi_mem_enter_qspi_mode",
  "spi_mem_enter_qspi_mode channel check_sr2"
};

static int exit_qspi_mode_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem exit_qspi_mode_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, NULL, exit_qspi_mode_handler}
};
static const ShellCommand exit_qspi_mode_command = {
  exit_qspi_mode_command_items,
  "spi_mem_exit_qspi_mode",
  "spi_mem_exit_qspi_mode channel"
};

static int parse_size(const char *str, unsigned int *size)
{
  char *endptr;
  unsigned long value = strtoul(str, &endptr, 10);
  switch (*endptr)
  {
  case 0: break;
  case 'k': value *= 1024; break;
  case 'm': value *= 1024 * 1024; break;
  default: return 1;
  }
  *size = (unsigned int)value;
  return 0;
}

static int check_supported(printf_func pfunc, void *ptr)
{
  if (ptr == NULL)
  {
    pfunc("not supported.\r\n");
    return 0;
  }
  return 1;
}

static int read_id_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  if (!check_supported(pfunc, device->read_id))
    return 1;
  int channel = atoi(argv[0]);
  unsigned int id;
  int rc = device->read_id(channel, &id);
  if (rc)
    return rc;
  pfunc("Device id: %x\r\n", id);
  return 0;
}

static int wren_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  if (!check_supported(pfunc, device->wren))
    return 1;
  int channel = atoi(argv[0]);
  return device->wren(channel);
}

static int read_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *_data)
{
  int nbytes;
  unsigned long int address;

  int channel = atoi(argv[0]);

  address = strtoul(argv[1], NULL, 16);

  nbytes = atoi(argv[2]);
  if (nbytes <= 0 || nbytes > MEMORY_BUFFER_SIZE || nbytes > device->max_data_length)
  {
    pfunc("Invalid data size\r\n");
    return 2;
  }

  device->read(channel, address, memory_buffer, nbytes);
  print_hex_buffer(memory_buffer, nbytes, pfunc);

  return 0;
}

static unsigned char next_byte(void)
{
  return chacha_u8(&rng);
}

static int write_random_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *_data)
{
  unsigned int nbytes;
  unsigned long int address;

  if (!check_supported(pfunc, device->write_callback))
    return 1;

  int channel = atoi(argv[0]);
  address = strtoul(argv[1], NULL, 16);

  int rc = parse_size(argv[2], &nbytes);
  if (rc || nbytes > device->max_data_length)
  {
    pfunc("Invalid data size\r\n");
    return 2;
  }

  iv[0]++;
  chacha20_zero(&rng, (const uint32_t*)iv);

  pfunc("num_bytes %d\n", nbytes);
  device->write_callback(channel, 256, address, next_byte, (int)nbytes);
  return 0;
}

static int set_byte(unsigned char byte)
{
  return chacha_u8(&rng) != byte;
}

static int check_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *_data)
{
  unsigned int nbytes;
  unsigned long int address;

  if (!check_supported(pfunc, device->read_callback))
    return 1;

  int channel = atoi(argv[0]);
  address = strtoul(argv[1], NULL, 16);

  int rc = parse_size(argv[2], &nbytes);
  if (rc || nbytes > device->max_data_length)
  {
    pfunc("Invalid data size\r\n");
    return 2;
  }

  chacha20_zero(&rng, (const uint32_t*)iv);

  pfunc("num_bytes %d\n", nbytes);
  return device->read_callback(channel, address, set_byte, (int)nbytes);
}

static int erase_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *_data)
{
  if (!check_supported(pfunc, device->erase))
    return 1;

  int channel = atoi(argv[0]);
  if (argc == 2)
  {
    if (strcmp(argv[1], "chip"))
      return 1;
    pfunc("chip erase\r\n");
    flash_erase(channel, CHIP, 0);
  }
  else
  {
    unsigned long int address = strtoul(argv[2], NULL, 16);
    if (!strcmp(argv[1], "sector"))
    {
      pfunc("sector erase, address %06x\r\n", address);
      flash_erase(channel, SECTOR, address);
    }
    else if (!strcmp(argv[1], "block32"))
    {
      pfunc("block32 erase, address %06x\r\n", address);
      flash_erase(channel, BLOCK32, address);
    }
    else if (!strcmp(argv[1], "block64"))
    {
      pfunc("block64 erase, address %06x\r\n", address);
      flash_erase(channel, BLOCK64, address);
    }
    else
      return 1;
  }
  return 0;
}

static int read_sr1_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *_data)
{
  if (!check_supported(pfunc, device->read_sr1))
    return 1;

  int channel = atoi(argv[0]);
  unsigned int sr1 = device->read_sr1(channel);
  pfunc("SR1: %02x\r\n", sr1);
  return 0;
}

static int read_sr2_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *_data)
{
  if (!check_supported(pfunc, device->read_sr2))
    return 1;

  int channel = atoi(argv[0]);
  unsigned int sr1 = device->read_sr2(channel);
  pfunc("SR2: %02x\r\n", sr1);
  return 0;
}

static int enter_qspi_mode_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  if (!check_supported(pfunc, device->enter_qspi_mode))
    return 1;
  int channel = atoi(argv[0]);
  if (channel != 0)
    return 2;
  int check_sr2 = atoi(argv[1]);
  return device->enter_qspi_mode(channel, check_sr2);
}

static int exit_qspi_mode_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  if (!check_supported(pfunc, device->exit_qspi_mode))
    return 1;
  int channel = atoi(argv[0]);
  if (channel != 0)
    return 2;
  return device->exit_qspi_mode(channel);
}

void register_spi_commands(void)
{
  shell_register_command(&read_id_command);
  shell_register_command(&wren_command);
  shell_register_command(&read_command);
  shell_register_command(&write_random_command);
  shell_register_command(&check_command);
  shell_register_command(&erase_command);
  shell_register_command(&read_sr1_command);
  shell_register_command(&read_sr2_command);
  shell_register_command(&enter_qspi_mode_command);
  shell_register_command(&exit_qspi_mode_command);
}
