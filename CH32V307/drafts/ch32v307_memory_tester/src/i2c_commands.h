#ifndef _I2C_COMMANDS_H
#define _I2C_COMMANDS_H

#include <shell.h>

void register_i2c_commands(void);
void print_hex_buffer(unsigned char *buffer, int length, printf_func pfunc);

#endif
