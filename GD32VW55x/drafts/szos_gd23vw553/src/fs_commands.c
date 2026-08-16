#include "board.h"
#include "fs_commands.h"
#include <shell.h>
#include <string.h>
#include <stdlib.h>
#include "fs.h"
#include "storage.h"

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

static const storage_t *get_storage(printf_func pfunc, int argc, const char *inpath, const char **outpath)
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
    pfunc("%s %40s %7d\r\n", de->type == DT_DIR ? "dir" : "file", de->name, de->size);
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

static int rename_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  const char *outpath1;
  const storage_t *storage1 = get_storage(pfunc, argc, argv[0], &outpath1);
  if (storage1 == nullptr)
    return 1;
  const char *p1 = strdup(outpath1);
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
  pfunc("%s\r\n", cwd);
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

void register_fs_commands(void)
{
  cwd[0] = '/';
  cwd[1] = 0;
  shell_register_command(&pwd_command);
  shell_register_command(&cd_command);
  shell_register_command(&ls_command);
  shell_register_command(&mkdir_command);
  shell_register_command(&rmdir_command);
  shell_register_command(&rename_command);
}
