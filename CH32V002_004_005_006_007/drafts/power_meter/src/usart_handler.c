#include "board.h"
#include "usart_handler.h"
#include <string.h>
#include <common_printf.h>
#include <spi_memory.h>
#include <ina228.h>

static const INA228Config cfg1 = {
  .bits = {
    .adcrange = 1,
    .tempcomp = 0,
    .convdly = 0,
    .rstacc = 0,
    .rst = 0,
    .reserved = 0
  }
};

static const INA228Config cfg0 = {
  .bits = {
    .adcrange = 0,
    .tempcomp = 0,
    .convdly = 0,
    .rstacc = 0,
    .rst = 0,
    .reserved = 0
  }
};

static const INA228Config cfg_reset = {
  .bits = {
    .adcrange = 0,
    .tempcomp = 0,
    .convdly = 0,
    .rstacc = 0,
    .rst = 1,
    .reserved = 0
  }
};

static void run_command(void)
{
  size_t l = strlen((const char*)command_line);
  switch (l)
  {
    case 8:
      if (!strcmp((const char*)command_line, "flash_id"))
      {
        unsigned int id;
        flash_read_id(0, &id);
        common_printf("FLASH id: %x\n", id);
        return;
      }
      if (!strcmp((const char*)command_line, "ina_read"))
      {
        int value;
        int rc = ina228GetBusVoltage(0, INA_ADDRESS, &value);
        if (rc != 0)
        {
          puts_("ina228GetBusVoltage failed\r\n");
          return;
        }
        common_printf("Bus voltage %d uV\n", value);
        rc = ina228GetShuntCurrent(0, INA_ADDRESS, 1000, &value);
        if (rc != 0)
        {
          puts_("ina228GetShuntCurrent failed\r\n");
          return;
        }
        common_printf("Shunt current %d nA\n", value);
        rc = ina228GetTemperature(0, INA_ADDRESS, &value);
        if (rc != 0)
        {
          puts_("ina228GetTemperature failed\r\n");
          return;
        }
        common_printf("Temperature %d.%03d grad\n", value / 1000, value % 1000);
        return;
      }
      break;
    case 9:
      if (!strcmp((const char*)command_line, "ina_init0"))
      {
        int rc = ina228SetConfig(0, INA_ADDRESS, cfg0);
        if (rc != 0)
          puts_("ina228SetConfig failed\r\n");
        else
          puts_("OK\r\n");
        return;
      }
      if (!strcmp((const char*)command_line, "ina_init1"))
      {
        int rc = ina228SetConfig(0, INA_ADDRESS, cfg1);
        if (rc != 0)
          puts_("ina228SetConfig failed\r\n");
        else
          puts_("OK\r\n");
        return;
      }
      break;
    default:
      break;
  }
  puts_("unknown command\r\n");
}

static void usart_echo(void)
{
  while (command_line_echo_p != command_line_p)
    usart_transmit(*command_line_echo_p++);
}

void usart_handler(void)
{
  usart_echo();
  if (command_ready)
  {
    puts_("\r\n");
    *command_line_p = 0;
    run_command();
    command_line_p = command_line;
    command_line_echo_p = command_line;
    command_ready = false;
  }
}
