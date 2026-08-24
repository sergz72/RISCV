#ifndef _PMP_H
#define _PMP_H

void pmp_init_flash_rx_ram_rw(void);
void pmp_init_user(unsigned int image, unsigned int image_size, unsigned int text_size);

#endif
