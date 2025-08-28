#include "board.h"
#include <i2c_commands.h>
#include <shell.h>
#include <i2c_func.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <chacha.h>

static int scan_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem scan_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, NULL, scan_handler}
};
static const ShellCommand scan_command = {
  scan_command_items,
  "i2c_scan",
  "i2c_scan instance",
  NULL,
  NULL
};

static int write_random_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem write_random_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, NULL, write_random_handler}
};
static const ShellCommand write_random_command = {
  write_random_command_items,
  "i2c_write_random",
  "i2c_write_random instance i2c_address memory_address_length memory_address page_length length",
  NULL,
  NULL
};

static int read_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem read_command_items[] = {
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, param_handler, NULL},
  {NULL, NULL, read_handler}
};
static const ShellCommand read_command = {
  read_command_items,
  "i2c_read",
  "i2c_read instance i2c_address memory_address_length memory_address length",
  NULL,
  NULL
};

unsigned char memory_buffer[MEMORY_BUFFER_SIZE];
ChaCha rng;

static I2C_TypeDef *get_instance(char *arg)
{
  if (!strcmp(arg, "i2c1"))
    return I2C1;
  if (!strcmp(arg, "i2c2"))
    return I2C2;
  return NULL;
}

static int scan_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  I2C_TypeDef *instance = get_instance(argv[0]);
  if (!instance)
    return 1;

  pfunc("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
  pfunc("00:   ");

  for (unsigned char i = 1; i <= 0x7F; i++)
  {
    if (i % 16 == 0)
      pfunc("\n%.2x:", i);
    int rc = i2c_check(instance, i << 1, I2C_TIMEOUT_SCAN);
    if (rc == 0)
      pfunc(" %.2x", i);
    else
      pfunc(" --");
  }

  return 0;
}

void print_hex_buffer(unsigned char *buffer, int length, printf_func pfunc)
{
  pfunc("   ");
  for (int i = 0; i < 16; i++)
    pfunc(" %02x", i);

  for (int i = 0; i < length; i++)
  {
    int row_no = i / 16;
    if (i % 16 == 0)
      pfunc("\n%02x:", row_no);
    pfunc(" %02x", *buffer++);
  }
  pfunc("\n");
}

static int read_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  I2C_TypeDef *instance = get_instance(argv[0]);
  if (!instance)
    return 100;
  unsigned long int i2c_address = strtoul(argv[1], NULL, 16);
  if (i2c_address <= 0 || i2c_address > 0x7F)
    return 200;
  int memory_address_length = atoi(argv[2]);
  if (memory_address_length <= 0 || memory_address_length > 4)
    return 300;
  unsigned long int memory_address = strtoul(argv[3], NULL, 16);
  int length = atoi(argv[4]);
  if (length <= 0 || length > sizeof(memory_buffer))
    return 400;
  int rc = i2c_memory_read(instance, i2c_address << 1, memory_address, memory_address_length, memory_buffer, length, I2C_TIMEOUT);
  if (rc)
    return rc;
  print_hex_buffer(memory_buffer, length, pfunc);
  return 0;
}

static int write_random_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  I2C_TypeDef *instance = get_instance(argv[0]);
  if (!instance)
    return 100;
  unsigned long int i2c_address = strtoul(argv[1], NULL, 16);
  if (i2c_address <= 0 || i2c_address > 0x7F)
    return 200;
  int memory_address_length = atoi(argv[2]);
  if (memory_address_length <= 0 || memory_address_length > 4)
    return 300;
  unsigned long int memory_address = strtoul(argv[3], NULL, 16);
  int page_length = atoi(argv[4]);
  if (page_length <= 0)
    return 400;
  int length = atoi(argv[5]);
  if (length <= 0 || length > sizeof(memory_buffer))
    return 500;

  chacha20_zero(&rng, 0);

  for (int i = 0; i < length; i++)
    memory_buffer[i] = chacha_u8(&rng);

  print_hex_buffer(memory_buffer, length, pfunc);
  return i2c_memory_write_pages(instance, i2c_address << 1, memory_address, memory_address_length,
                          page_length, memory_buffer, length, I2C_TIMEOUT);
}

void register_i2c_commands(void)
{
  shell_register_command(&scan_command);
  shell_register_command(&read_command);
  shell_register_command(&write_random_command);
}
