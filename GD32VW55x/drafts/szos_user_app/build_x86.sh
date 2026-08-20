#! /bin/sh

gcc \
-mcmodel=small \
-O3 \
-pie \
-I../szos_gd23vw553/syscalls/x64 \
-o main.elf \
-nostdlib \
src/main.c \
-L../szos_gd23vw553/syscalls/x64 \
-lsyscalls_x86 \
-T ../szos_gd23vw553/syscalls/x64/ldscript.ld \
-Wl,--defsym=FLASH_SIZE=8k,--defsym=RAM_SIZE=1k
