#include <lfs.h>
#include <sys/errno.h>
#include "storage.h"
#include <string.h>

static void * lfs_opendir(void *fs_context, const char *path);
static struct dirent *lfs_readdir(void *fs_context, void *dirp);
static int lfs_closedir(void *fs_context, void *dirp);
static int lfs_mkdir_op(void *fs_context, const char *path);
static int lfs_rename_op(void *fs_context, const char *old_name, const char *new_name);
static int lfs_remove_op(void *fs_context, const char *path);
static void * lfs_fopen(void *fs_context, const char *path, const char *mode);
static int lfs_fclose(void *fs_context, void *f);
static int lfs_fread(void *fs_context, void *ptr, size_t size, size_t count, void *stream);
static int lfs_fwrite(void *fs_context, const void *ptr, size_t size, size_t count, void *stream);
static int lfs_fseek(void *fs_context, void *stream, long offset, int whence);
static int lfs_truncate(void *fs_context, void *stream, unsigned int length);
static int lfs_file_size_op(void *fs_context, void *stream, unsigned int *length);
static int lfs_stat_op(void *fs_context, const char *path, struct file_stat *st);
static int lfs_fsstat_op(void *fs_context, struct fs_stat *st);

const fs_operations_t lfs_operations =
{
  .opendir = lfs_opendir,
  .readdir = lfs_readdir,
  .closedir = lfs_closedir,
  .mkdir = lfs_mkdir_op,
  .rmdir = lfs_remove_op,
  .rename = lfs_rename_op,
  .remove = lfs_remove_op,
  .fopen = lfs_fopen,
  .fclose = lfs_fclose,
  .fread = lfs_fread,
  .fseek = lfs_fseek,
  .fwrite = lfs_fwrite,
  .truncate = lfs_truncate,
  .file_size = lfs_file_size_op,
  .stat = lfs_stat_op,
  .fs_stat = lfs_fsstat_op
};

static void * lfs_opendir(void *fs_context, const char *path)
{
  lfs_dir_t *dir = malloc(sizeof(lfs_dir_t));
  if (dir == nullptr)
    return nullptr;
  const int rc = lfs_dir_open(fs_context, dir, path);
  if (rc != 0)
  {
    free(dir);
    return nullptr;
  }
  return dir;
}

static struct dirent *lfs_readdir(void *fs_context, void *dirp)
{
  struct lfs_info *info = malloc(sizeof(struct lfs_info));
  if (info == nullptr)
    return nullptr;
  const int rc = lfs_dir_read(fs_context, dirp, info);
  if (rc <= 0)
  {
    free(info);
    return nullptr;
  }
  struct dirent *entry = malloc(sizeof(struct dirent));
  if (entry == nullptr)
  {
    free(info);
    return nullptr;
  }
  strcpy(entry->name, info->name);
  entry->type = info->type;
  entry->size = info->size;
  free(info);
  return entry;
}

static int lfs_closedir(void *fs_context, void *dirp)
{
  const int rc = lfs_dir_close(fs_context, dirp);
  free(dirp);
  return rc;
}

static int lfs_mkdir_op(void *fs_context, const char *path)
{
  return lfs_mkdir(fs_context, path);
}

static int lfs_remove_op(void *fs_context, const char *path)
{
  return lfs_remove(fs_context, path);
}

static int lfs_rename_op(void *fs_context, const char *old_name, const char *new_name)
{
  return lfs_rename(fs_context, old_name, new_name);
}

static void * lfs_fopen(void *fs_context, const char *path, const char *mode)
{
  size_t l = strlen(mode);
  if (l == 0 || l > 2)
    return nullptr;
  int flags;
  switch (mode[0])
  {
  case 'r':
    switch (mode[1])
    {
    case 0:
      flags = LFS_O_RDONLY;
      break;
    case '+':
      flags = LFS_O_RDWR;
      break;
    default:
      return nullptr;
    }
    break;
  case 'w':
    switch (mode[1])
    {
    case 0:
      flags = LFS_O_WRONLY | LFS_O_CREAT;
      break;
    case '+':
      flags = LFS_O_RDWR | LFS_O_CREAT;
      break;
    default:
      return nullptr;
    }
    break;
  case 'a':
    switch (mode[1])
    {
    case 0:
      flags = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND;
      break;
    case '+':
      flags = LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND;
      break;
    default:
      return nullptr;
    }
    break;
  default:
    return nullptr;
  }
  lfs_file_t* f = malloc(sizeof(lfs_file_t));
  if (f == nullptr)
    return nullptr;
  const int rc = lfs_file_open(fs_context, f, path, flags);
  if (rc != 0)
  {
    free(f);
    return nullptr;
  }
  return f;
}

static int lfs_fclose(void *fs_context, void *f)
{
  const int rc = lfs_file_close(fs_context, f);
  free(f);
  return rc;
}

static int lfs_fread(void *fs_context, void *ptr, const size_t size, const size_t count, void *stream)
{
  return lfs_file_read(fs_context, stream, ptr, size * count);
}

static int lfs_fwrite(void *fs_context, const void *ptr, const size_t size, const size_t count, void *stream)
{
  return lfs_file_write(fs_context, stream, ptr, size * count);
}

static int lfs_fseek(void *fs_context, void *stream, long offset, const int whence)
{
  return lfs_file_seek(fs_context, stream, offset, whence);
}

static int lfs_truncate(void *fs_context, void *stream, const unsigned int length)
{
  return lfs_file_truncate(fs_context, stream, length);
}

static int lfs_file_size_op(void *fs_context, void *stream, unsigned int *length)
{
  const int rc = lfs_file_size(fs_context, stream);
  if (rc < 0)
    return rc;
  *length = rc;
  return 0;
}

static int lfs_stat_op(void *fs_context, const char *path, struct file_stat *st)
{
  struct lfs_info info;
  const int rc = lfs_stat(fs_context, path, &info);
  if (rc >= 0)
  {
    st->size = info.size;
    st->type = info.type;
  }
  return rc;
}

static int lfs_fsstat_op(void *fs_context, struct fs_stat *st)
{
  int in_use_blocks = lfs_fs_size(fs_context);
  if (in_use_blocks < 0)
    return in_use_blocks;
  struct lfs_fsinfo info;
  const int rc = lfs_fs_stat(fs_context, &info);
  if (rc >= 0)
  {
    st->total_size = info.block_count * info.block_size;
    st->used_size = in_use_blocks * info.block_size;
  }
  return rc;
}

static int read_storage(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size)
{
  storage_t *storage = c->context;

  unsigned int addr = block * c->block_size + off;

  return storage->read(addr, size, buffer);
}

static int prog_storage(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size)
{
  storage_t *storage = c->context;

  unsigned int addr = block * c->block_size + off;

  return storage->write(addr, size, buffer);
}

static int erase_storage(const struct lfs_config* c, lfs_block_t block)
{
  storage_t *storage = c->context;

  unsigned int addr = block * c->block_size;

  return storage->erase(addr);
}

static int sync_storage(const struct lfs_config* c)
{
  return LFS_ERR_OK;
}

int fs_lfs_init(storage_t *storage, unsigned int cache_size, unsigned int lookahead_size, int block_cycles)
{
  struct lfs_config *cfg = calloc(1, sizeof(struct lfs_config));
  if (cfg == nullptr)
    return ENOMEM;
  cfg->context = storage;
  lfs_t *lfs = calloc(1, sizeof(lfs_t));
  if (lfs == nullptr)
  {
    free(cfg);
    return ENOMEM;
  }
  cfg->read  = read_storage;
  cfg->prog  = prog_storage;
  cfg->erase = erase_storage;
  cfg->sync  = sync_storage;
  // block device configuration
  cfg->read_size = storage->minimum_read_size,
  cfg->prog_size = storage->minimum_write_size,
  cfg->block_size = storage->page_size,
  cfg->block_count = storage->size / storage->page_size,
  cfg->cache_size = cache_size,
  cfg->lookahead_size = lookahead_size,
  cfg->block_cycles = block_cycles;

  // mount the filesystem
  int err = lfs_mount(lfs, cfg);

  // reformat if we can't mount the filesystem
  // this should only happen on the first boot
  if (err)
  {
    err = lfs_format(lfs, cfg);
    if (err)
      return err;
    err = lfs_mount(lfs, cfg);
    if (err)
    {
      free(lfs);
      free(cfg);
      return err;
    }
  }

  storage->fs_context = lfs;
  storage->fs_operations = &lfs_operations;

  return 0;
}
