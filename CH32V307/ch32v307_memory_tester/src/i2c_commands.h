#ifndef _I2C_COMMANDS_H
#define _I2C_COMMANDS_H

#include <shell.h>
#include <chacha.h>

void register_i2c_commands(void);
void print_hex_buffer(unsigned char *buffer, int length, printf_func pfunc);

extern unsigned char memory_buffer[MEMORY_BUFFER_SIZE];
extern ChaCha rng;
extern unsigned int iv[3];

#endif
