#include <string.h>

#include "board.h"
#include "usb_desc.h"

#define COMMAND_CONTINUE 0
#define COMMAND_READ     1
#define COMMAND_WRITE    2
#define COMMAND_ERASE    3
#define COMMAND_PROBE    4
#define COMMAND_ABORT    5

#define INTERFACE_SPI1 0
#define INTERFACE_SPI2 1
#define INTERFACE_I2C1 2
#define INTERFACE_I2C2 3
#define INTERFACE_QSPI 4

typedef struct __attribute__((packed))
{
  unsigned char command;
  char dry_run;
  unsigned char interface_id;
  unsigned char i2c_address;
  unsigned int speed;
  unsigned int address;
  unsigned int length;
} Command;

static const unsigned char no_device[] = {0x00, 0xFF, 0xFF, 0xFF};

static Command current_command;
static unsigned char response_data[DEF_USBD_HS_PACK_SIZE];

static unsigned int send_error(const char *error_message)
{
  unsigned int l = strlen(error_message);
  memcpy(response_data, error_message, l);
  return l;
}

static unsigned int send_ok(void)
{
  response_data[0] = 'k';
  return 1;
}

static unsigned int continue_write(const unsigned char *buffer, unsigned int length)
{
  if (current_command.command != COMMAND_WRITE || current_command.length == 0)
    return send_error("unexpected continue command");
  length--;
  buffer++;
  unsigned int address = current_command.address;
  if (length > current_command.length)
    return send_error("invalid continue write length");
  current_command.length -= length;
  current_command.address += length;
  if (current_command.dry_run)
    return send_ok();
  return send_error("not implemented");
}

static unsigned int erase(void)
{
  if (current_command.dry_run)
    return send_ok();
  return send_error("not implemented");
}

static unsigned int probe(void)
{
  if (current_command.dry_run)
  {
    memcpy(response_data, no_device, sizeof(no_device));
    return sizeof(no_device);
  }
  return send_error("not implemented");
}

static unsigned int write_start(const unsigned char *buffer, unsigned int length)
{
  unsigned int l = length - sizeof(Command);
  unsigned int address = current_command.address;
  if (l > current_command.length)
    return send_error("invalid write length");
  current_command.length -= l;
  current_command.address += l;
  if (current_command.dry_run)
    return send_ok();
  return send_error("not implemented");
}

static unsigned int continue_read(void)
{
  if (current_command.length == 0)
    return 0;
  unsigned int l = current_command.length > sizeof(response_data) ? sizeof(response_data) : current_command.length;
  unsigned int address = current_command.address;
  current_command.length -= l;
  current_command.address += l;
  if (current_command.dry_run)
    return l;
  return l;
}

unsigned int run_command(const unsigned char *buffer, unsigned int length, unsigned char **response)
{
  unsigned char command = buffer[0];
  if (command != COMMAND_CONTINUE)
    memcpy(&current_command, buffer, sizeof(Command));
  *response = response_data;
  switch (command)
  {
    case COMMAND_CONTINUE:
      if (length < 2)
        return send_error("invalid continue command length");
      return continue_write(buffer, length);
    case COMMAND_READ:
      if (length != sizeof(Command))
        return send_error("invalid read command length");
      return send_ok();
    case COMMAND_WRITE:
      if (length <= sizeof(Command))
        return send_error("invalid write command length");
      return write_start(buffer, length);
    case COMMAND_ERASE:
      if (length != sizeof(Command))
        return send_error("invalid erase command length");
      return erase();
    case COMMAND_PROBE:
      if (length != 8)
        return send_error("invalid probe command length");
      return probe();
    case COMMAND_ABORT: return send_ok();
    default: return send_error("unknown command");
  }
}

unsigned int continue_command_execution(unsigned char **response)
{
  *response = response_data;
  if (current_command.command == COMMAND_READ)
    return continue_read();
  return 0;
}
