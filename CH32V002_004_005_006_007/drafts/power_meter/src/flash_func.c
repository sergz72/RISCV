#include "board.h"
#include "flash_func.h"
#include "ina_func.h"
#include <string.h>
#include <spi_memory.h>

typedef struct
{
  unsigned int current_ua;
  unsigned int voltage_uv;
} flash_record;

#define FLASH_BUFFER_SIZE (FLASH_PAGE_SIZE / sizeof(flash_record))

static flash_record flash_buffer[FLASH_BUFFER_SIZE];
static unsigned int current_flash_buffer_idx;
static unsigned int current_flash_address;

void flash_init(void)
{
  current_flash_buffer_idx = current_flash_address = 0;
  memset(flash_buffer, 0xFF, sizeof(flash_buffer));
  flash_erase(0, CHIP, 0);
}

void flash_save(void)
{
  flash_write_page(0, current_flash_address, (unsigned char*)flash_buffer, FLASH_PAGE_SIZE);
  current_flash_address += FLASH_PAGE_SIZE;
}

void flash_add_record(void)
{
  flash_record *r = &flash_buffer[current_flash_buffer_idx];
  r->current_ua = current_ua;
  r->voltage_uv = voltage_uv;
  if (current_flash_buffer_idx == FLASH_BUFFER_SIZE - 1)
  {
    flash_save();
    memset(flash_buffer, 0xFF, sizeof(flash_buffer));
  }
  else
    current_flash_buffer_idx++;
}
