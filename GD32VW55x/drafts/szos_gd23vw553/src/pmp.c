#include "board.h"
#include "pmp.h"
#include <core_feature_pmp.h>

#define FLASH_SIZE   (4*1024*1024)

#define RAM_BASE   0x20000000
#define RAM_SIZE   (256*1024)

// Flash region: Read/Execute in U mode
#define U_EXECUTE_READ    (PMP_R | PMP_X)
// RAM region: Read/Write in U mode
#define U_READ_WRITE      (PMP_R | PMP_W)

void pmp_init_flash_rx_ram_rw(void)
{
  pmp_config pmp_config_flash, pmp_config_ram;

  pmp_config_flash.protection = U_EXECUTE_READ;
  pmp_config_flash.order = __CTZ(FLASH_SIZE);
  pmp_config_flash.base_addr = FLASH_BASE;

  pmp_config_ram.protection = U_READ_WRITE;
  pmp_config_ram.order = __CTZ(RAM_SIZE);
  pmp_config_ram.base_addr = RAM_BASE;

  __set_PMPENTRYx(0, &pmp_config_flash);
  __set_PMPENTRYx(1, &pmp_config_ram);
}

void pmp_init_user(unsigned int image, unsigned int image_size, unsigned int text_size)
{
  pmp_config pmp_config_text, pmp_config_data;

  pmp_config_text.protection = U_EXECUTE_READ;
  pmp_config_text.order = __CTZ(text_size);
  pmp_config_text.base_addr = image;

  pmp_config_data.protection = U_READ_WRITE;
  pmp_config_data.order = __CTZ(image_size - text_size);
  pmp_config_data.base_addr = image + text_size;

  __set_PMPENTRYx(1, &pmp_config_text);
  __set_PMPENTRYx(2, &pmp_config_data);
}