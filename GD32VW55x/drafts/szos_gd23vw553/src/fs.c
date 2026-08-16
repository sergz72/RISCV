#include "fs.h"
#include "fs_lfs.h"
#include "storage_internal_flash.h"
#include <string.h>

static storage_t internal_flash_storage;

int fs_init(void)
{
  internal_flash_storage_init(&internal_flash_storage);
  return fs_lfs_init(&internal_flash_storage, internal_flash_storage.size >> 11, internal_flash_storage.size >> 15, 500);
}

const storage_t *fs_get_storage(const char *path, const char **outpath)
{
  if (!strncmp(path, "/flash", 6) && (path[6] == '/' || path[6] == 0))
  {
    if (path[6] == 0)
      *outpath = "/";
    else
      *outpath = path + 6;
    return &internal_flash_storage;
  }
  return nullptr;
}