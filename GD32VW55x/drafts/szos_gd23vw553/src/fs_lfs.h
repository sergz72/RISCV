#ifndef _FS_LFS_H
#define _FS_LFS_H

#include "storage.h"

int fs_lfs_init(storage_t *storage, unsigned int cache_size, unsigned int lookahead_size, int block_cycles);

#endif
