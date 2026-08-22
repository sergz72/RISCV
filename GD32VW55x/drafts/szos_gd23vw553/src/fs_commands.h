#ifndef SZOS_GD32VW553_FS_COMMANDS_H
#define SZOS_GD32VW553_FS_COMMANDS_H

#include "storage.h"
#include "shell.h"

void register_fs_commands(void);
const storage_t *get_storage(printf_func pfunc, int argc, const char *inpath, const char **outpath);

#endif
