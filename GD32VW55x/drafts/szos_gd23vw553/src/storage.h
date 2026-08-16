#ifndef SZOS_GD32VW553_STORAGE_H
#define SZOS_GD32VW553_STORAGE_H

#include <stddef.h>

#define DT_REG 1
#define DT_DIR 2

struct dirent
{
  size_t size;
  unsigned char type;
  char name[256];
};

typedef struct
{
  void * (*opendir)(void *fs_context, const char *path);
  struct dirent *(*readdir) (void *fs_context, void *dirp);
  int (*closedir) (void *fs_context, void *dirp);
  int (*mkdir)(void *fs_context, const char *path);
  int (*rmdir)(void *fs_context, const char *path);
  int (*rename)(void *fs_context, const char *old_name, const char *new_name);
  int (*remove)(void *fs_context, const char *path);
  void * (*fopen)(void *fs_context, const char *path, const char *mode);
  int (*fclose)(void *fs_context, void *f);
  int (*fread)(void *fs_context, void *ptr, size_t size, size_t count, void *stream);
  int (*fwrite)(void *fs_context, const void *ptr, size_t size, size_t count, void *stream);
  int (*fseek)(void *fs_context, void *stream, long offset, int whence);
  int (*truncate)(void *fs_context, void *stream, unsigned int length);
} fs_operations_t;

typedef struct {
  unsigned int size;
  unsigned int page_size;
  unsigned int minimum_write_size;
  unsigned int minimum_read_size;
  void *fs_context;
  const fs_operations_t *fs_operations;
  int (*read)(unsigned int addr, unsigned int size, void* buffer);
  int (*write)(unsigned int addr, unsigned int size, const void* buffer);
  int (*erase)(unsigned int addr);
} storage_t;

#endif
