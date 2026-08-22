#include "board.h"
#include "fs_commands.h"
#include <string.h>
#include <stdlib.h>
#include "fs.h"
#include <base64.h>
#include <crc32.h>

static char cwd[256];
static char temp_path[256];

static int pwd_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem pwd_command_items[] = {
  {nullptr, nullptr, pwd_handler}
};
static const ShellCommand pwd_command = {
  pwd_command_items,
  "pwd",
  "pwd",
  nullptr,
  nullptr
};

static int cd_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem cd_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, cd_handler}
};
static const ShellCommand cd_command = {
  cd_command_items,
  "cd",
  "cd path",
  nullptr,
  nullptr
};

static int ls_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem ls_command_items[] = {
  {nullptr, param_handler, ls_handler},
  {nullptr, nullptr, ls_handler}
};
static const ShellCommand ls_command = {
  ls_command_items,
  "ls",
  "ls [path]",
  nullptr,
  nullptr
};

static int mkdir_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem mkdir_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, mkdir_handler}
};
static const ShellCommand mkdir_command = {
  mkdir_command_items,
  "mkdir",
  "mkdir path",
  nullptr,
  nullptr
};

static int rmdir_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem rmdir_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, rmdir_handler}
};
static const ShellCommand rmdir_command = {
  rmdir_command_items,
  "rmdir",
  "rmdir path",
  nullptr,
  nullptr
};

static int rename_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem rename_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, rename_handler}
};
static const ShellCommand rename_command = {
  rename_command_items,
  "rename",
  "rename path1 path2",
  nullptr,
  nullptr
};

static int cat_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem cat_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, cat_handler}
};
static const ShellCommand cat_command = {
  cat_command_items,
  "cat",
  "cat path",
  nullptr,
  nullptr
};

static int truncate_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem truncate_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, truncate_handler}
};
static const ShellCommand truncate_command = {
  truncate_command_items,
  "truncate",
  "truncate path",
  nullptr,
  nullptr
};

static int puts_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem puts_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, puts_handler}
};
static const ShellCommand puts_command = {
  puts_command_items,
  "puts",
  "puts path text",
  nullptr,
  nullptr
};

static int fopen_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem fopen_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, fopen_handler}
};
static const ShellCommand fopen_command = {
  fopen_command_items,
  "fopen",
  "fopen path mode",
  nullptr,
  nullptr
};

static int fwrite_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem fwrite_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, fwrite_handler}
};
static const ShellCommand fwrite_command = {
  fwrite_command_items,
  "fwrite",
  "fwrite data_base64",
  nullptr,
  nullptr
};

static int fread_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem fread_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, fread_handler}
};
static const ShellCommand fread_command = {
  fread_command_items,
  "fread",
  "fread length",
  nullptr,
  nullptr
};

static int fclose_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem fclose_command_items[] = {
  {nullptr, nullptr, fclose_handler}
};
static const ShellCommand fclose_command = {
  fclose_command_items,
  "fclose",
  "fclose",
  nullptr,
  nullptr
};

static int fcrc32_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem fcrc32_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, fcrc32_handler}
};
static const ShellCommand fcrc32_command = {
  fcrc32_command_items,
  "fcrc32",
  "fcrc32 path",
  nullptr,
  nullptr
};

static int rm_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem rm_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, rm_handler}
};
static const ShellCommand rm_command = {
  rm_command_items,
  "rm",
  "rm path",
  nullptr,
  nullptr
};

static int fsize_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem fsize_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, fsize_handler}
};
static const ShellCommand fsize_command = {
  fsize_command_items,
  "fsize",
  "fsize path",
  nullptr,
  nullptr
};

static int df_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem df_command_items[] = {
  {nullptr, nullptr, df_handler}
};
static const ShellCommand df_command = {
  df_command_items,
  "df",
  "df",
  nullptr,
  nullptr
};


const storage_t *current_storage;
void *current_file;

const storage_t *get_storage(printf_func pfunc, int argc, const char *inpath, const char **outpath)
{
  const char *path;
  if (argc == 0)
    path = cwd;
  else if (inpath[0] == '/')
    path = inpath;
  else
  {
    size_t l = strlen(cwd);
    strcpy(temp_path, cwd);
    if (l == 0 || temp_path[l-1] != '/')
      temp_path[l++] = '/';
    strcpy(&temp_path[l], inpath);
    path = temp_path;
  }
  const storage_t *storage = fs_get_storage(path, outpath);
  if (storage == nullptr)
    pfunc("unknown mount point");
  return storage;
}

static int ls_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  const char *outpath;
  const storage_t *storage = get_storage(pfunc, argc, argv[0], &outpath);
  if (storage == nullptr)
    return 1;
  void *dir = storage->fs_operations->opendir(storage->fs_context, outpath);
  if (dir == nullptr)
  {
    pfunc("opendir failed");
    return 2;
  }
  for (;;)
  {
    struct dirent *de = storage->fs_operations->readdir(storage->fs_context, dir);
    if (de == nullptr)
      break;
    pfunc("%s %40s %7d\n", de->type == DT_DIR ? "dir " : "file", de->name, de->size);
  }
  storage->fs_operations->closedir(storage->fs_context, dir);
  return 0;
}

static int mkdir_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  const char *outpath;
  const storage_t *storage = get_storage(pfunc, argc, argv[0], &outpath);
  if (storage == nullptr)
    return 1;
  return storage->fs_operations->mkdir(storage->fs_context, outpath);
}

static int rmdir_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  const char *outpath;
  const storage_t *storage = get_storage(pfunc, argc, argv[0], &outpath);
  if (storage == nullptr)
    return 1;
  return storage->fs_operations->rmdir(storage->fs_context, outpath);
}

static int rm_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  const char *outpath;
  const storage_t *storage = get_storage(pfunc, argc, argv[0], &outpath);
  if (storage == nullptr)
    return 1;
  return storage->fs_operations->remove(storage->fs_context, outpath);
}

static int rename_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  const char *outpath1;
  const storage_t *storage1 = get_storage(pfunc, argc, argv[0], &outpath1);
  if (storage1 == nullptr)
    return 1;
  char *p1 = strdup(outpath1);
  if (p1 == nullptr)
    return 2;
  const char *outpath2;
  const storage_t *storage2 = get_storage(pfunc, argc, argv[1], &outpath2);
  if (storage2 == nullptr)
  {
    free(p1);
    return 3;
  }
  if (storage1 != storage2)
  {
    free(p1);
    pfunc("files should be in the same storage");
    return 4;
  }
  int rc = -storage1->fs_operations->rename(storage1->fs_context, p1, outpath2);
  free(p1);
  return rc;
}

static int pwd_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  pfunc("%s\n", cwd);
  return 0;
}

static int cd_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  if (argv[0][0] == '/')
    strcpy(cwd, argv[0]);
  else
  {
    size_t l = strlen(cwd);
    cwd[l] = '/';
    strcpy(&cwd[l+1], argv[0]);
  }
  return 0;
}

static int cat_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  const char *outpath;
  const storage_t *storage = get_storage(pfunc, argc, argv[0], &outpath);
  if (storage == nullptr)
    return 1;
  void *f = storage->fs_operations->fopen(storage->fs_context, outpath, "r");
  if (f == nullptr)
  {
    pfunc("Failed to open file for read\n");
    return 1;
  }
  int size;
  for (;;)
  {
    size = storage->fs_operations->fread(storage->fs_context, temp_path, sizeof(temp_path) - 1, 1, f);
    if (size <= 0)
      break;
    temp_path[size] = 0;
    pfunc("%s", temp_path);
    if (size < sizeof(temp_path) - 1)
    {
      size = 0;
      break;
    }
  }
  storage->fs_operations->fclose(storage->fs_context, f);
  return -size;
}

static int truncate_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  const char *outpath;
  const storage_t *storage = get_storage(pfunc, argc, argv[0], &outpath);
  if (storage == nullptr)
    return 1;
  void *f = storage->fs_operations->fopen(storage->fs_context, outpath, "w");
  if (f == nullptr)
  {
    pfunc("Failed to open file for write\n");
    return 1;
  }
  int rc = storage->fs_operations->truncate(storage->fs_context, f, 0);
  storage->fs_operations->fclose(storage->fs_context, f);
  return rc;
}

static int puts_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  const char *outpath;
  const storage_t *storage = get_storage(pfunc, argc, argv[0], &outpath);
  if (storage == nullptr)
    return 1;
  void *f = storage->fs_operations->fopen(storage->fs_context, outpath, "a");
  if (f == nullptr)
  {
    pfunc("Failed to open file for append\n");
    return 1;
  }
  size_t l = strlen(argv[1]);
  int rc = 0;
  if (l != 0)
  {
    strcpy(temp_path, argv[1]);
    temp_path[l++] = '\n';
    temp_path[l] = 0;
    rc = storage->fs_operations->fwrite(storage->fs_context, temp_path, l, 1, f);
  }

  storage->fs_operations->fclose(storage->fs_context, f);
  return rc < l ? 1 : 0;
}

static int fopen_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  if (current_file)
    return 1;
  const char *outpath;
  current_storage = get_storage(pfunc, argc, argv[0], &outpath);
  if (current_storage == nullptr)
    return 2;
  current_file = current_storage->fs_operations->fopen(current_storage->fs_context, outpath, argv[1]);
  if (current_file == nullptr)
  {
    current_storage = nullptr;
    pfunc("Failed to open file\n");
    return 3;
  }
  return 0;
}

static int fwrite_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  if (current_file == nullptr || current_storage == nullptr)
    return 1;

  unsigned int l = base64decode(argv[0], strlen(argv[0]), temp_path);
  int rc = current_storage->fs_operations->fwrite(current_storage->fs_context, temp_path, l, 1, current_file);
  return rc < l ? 2 : 0;
}

static int fread_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  if (current_file == nullptr || current_storage == nullptr)
    return 1;
  int l = atoi(argv[0]);
  if (l <= 0 || l > sizeof(temp_path) / 3)
    return 2;
  char buffer[l];
  int rc = current_storage->fs_operations->fread(current_storage->fs_context, buffer, l, 1, current_file);
  if (rc < 0)
    return 3;
  unsigned int ol = base64encode(buffer, l, temp_path);
  temp_path[ol] = 0;
  pfunc("%s\n", temp_path);
  return INT32_MAX;
}

static int fclose_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  if (current_file == nullptr || current_storage == nullptr)
    return 1;
  current_storage->fs_operations->fclose(current_storage->fs_context, current_file);
  current_file = nullptr;
  current_storage = nullptr;
  return 0;
}

static int fcrc32_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  const char *outpath;
  const storage_t *storage = get_storage(pfunc, argc, argv[0], &outpath);
  if (storage == nullptr)
    return 1;
  void *f = storage->fs_operations->fopen(storage->fs_context, outpath, "r");
  if (f == nullptr)
  {
    pfunc("Failed to open file for read\n");
    return 1;
  }
  int size;
  crc32_start();
  for (;;)
  {
    size = storage->fs_operations->fread(storage->fs_context, temp_path, sizeof(temp_path), 1, f);
    if (size <= 0)
      break;
    crc32_add(temp_path, size);
    if (size < sizeof(temp_path))
    {
      size = 0;
      break;
    }
  }
  pfunc("%08x\n", crc32_end());
  storage->fs_operations->fclose(storage->fs_context, f);
  return -size;
}

static int fsize_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  const char *outpath;
  const storage_t *storage = get_storage(pfunc, argc, argv[0], &outpath);
  if (storage == nullptr)
    return 1;
  void *f = storage->fs_operations->fopen(storage->fs_context, outpath, "r");
  if (f == nullptr)
  {
    pfunc("Failed to open file for read\n");
    return 2;
  }
  unsigned int size;
  int rc = storage->fs_operations->file_size(storage->fs_context, f, &size);
  if (rc)
  {
    pfunc("Failed to get file size\n");
    storage->fs_operations->fclose(storage->fs_context, f);
    return 3;
  }
  pfunc("%d\n", size);
  storage->fs_operations->fclose(storage->fs_context, f);
  return INT32_MAX;
}

static int df_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  unsigned int number_of_storages;
  const storage_t *storage = fs_get_storages(&number_of_storages);
  if (!storage)
    return 0;
  while (number_of_storages--)
  {
    struct fs_stat st;
    const int rc = storage->fs_operations->fs_stat(storage->fs_context, &st);
    if (rc)
      pfunc("%s: Failed to get file system status\n", storage->mount_point);
    else
      pfunc("%s: total %8d used %8d free %8d\n", storage->mount_point, st.total_size, st.used_size, st.total_size - st.used_size);
    storage++;
  }
  return 0;
}

void register_fs_commands(void)
{
  cwd[0] = '/';
  cwd[1] = 0;
  current_file = nullptr;
  current_storage = nullptr;
  shell_register_command(&pwd_command);
  shell_register_command(&cd_command);
  shell_register_command(&ls_command);
  shell_register_command(&mkdir_command);
  shell_register_command(&rmdir_command);
  shell_register_command(&rm_command);
  shell_register_command(&rename_command);
  shell_register_command(&cat_command);
  shell_register_command(&truncate_command);
  shell_register_command(&puts_command);
  shell_register_command(&fopen_command);
  shell_register_command(&fwrite_command);
  shell_register_command(&fread_command);
  shell_register_command(&fclose_command);
  shell_register_command(&fcrc32_command);
  shell_register_command(&fsize_command);
  shell_register_command(&df_command);
}
