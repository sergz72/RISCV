#include "board.h"
#include "storage_internal_flash.h"
#include <string.h>

static int internal_flash_read(unsigned int addr, unsigned int size, void* buffer)
{
  addr += FLASH_BASE_ADDR;
  memcpy(buffer, (const void *)addr, size);
  return 0;
}

static int internal_flash_write(unsigned int addr, unsigned int size, const void* buffer)
{
  addr += FLASH_BASE_ADDR;

  fmc_unlock();

  /* clear all pending flags */
  fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR );

  const unsigned int *p = buffer;
  size /= 4;
  /* program flash */
  for (uint32_t program_counter = 0; program_counter < size; program_counter++)
  {
    fmc_state_enum rc = fmc_word_program(addr, *p++);
    addr += 4;
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR);
    if (rc != FMC_READY)
    {
      fmc_lock();
      return rc;
    }
  }

  /* lock the main FMC after the program operation */
  fmc_lock();
  return 0;
}

static int internal_flash_erase(unsigned int addr)
{
  addr += FLASH_BASE_ADDR;

  fmc_unlock();
  fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR);
  fmc_state_enum rc = fmc_page_erase(addr);
  fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR );
  fmc_lock();
  return rc;
}

void internal_flash_storage_init(storage_t *storage)
{
  storage->size = FLASH_STORAGE_SIZE;
  storage->page_size = 4096;
  storage->minimum_read_size = 1;
  storage->minimum_write_size = 4;
  storage->read = internal_flash_read;
  storage->write = internal_flash_write;
  storage->erase = internal_flash_erase;
}
