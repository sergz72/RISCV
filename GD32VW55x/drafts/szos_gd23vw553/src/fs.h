#ifndef _FS_H
#define _FS_H

#include "storage.h"

int fs_init(void);
const storage_t *fs_get_storage(const char *path, const char **outpath);

#endif
