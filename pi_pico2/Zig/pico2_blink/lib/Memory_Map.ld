ENTRY(Startup_Init)

__KERNEL_STACK_SIZE = 8K;
__EXCEPTION_STACK_SIZE = 4K;

MEMORY
{
  FLASH(rx)   : ORIGIN = 0x10000000, LENGTH = 4M
  RAM(rwx)    : ORIGIN = 0x20000000, LENGTH = 520K
}

SECTIONS
{
  .text : ALIGN(4)
  {
    KEEP(*(.riscv_intvect))
    KEEP(*(.image_start_block))
    *(.text)
    *(.text*)
  } > FLASH

  .rodata : ALIGN(4)
  {
    *(.rodata)
  } > FLASH

  .data :  ALIGN(4)
  {
    PROVIDE(_data_start = .);
    PROVIDE( __global_pointer$ = . + 0x100);
    *(.data)
    *(.data*)
  } > RAM  AT>FLASH

  .bss : ALIGN(4)
  {
    PROVIDE(_bss_start = .);
    *(.bss)
    *(.bss*)
    . = ALIGN(8);
    PROVIDE(_bss_end = .);
    PROVIDE( _heap_start = . );
  } > RAM

  .stack ORIGIN(RAM) + LENGTH(RAM) - __KERNEL_STACK_SIZE * 2 - __EXCEPTION_STACK_SIZE * 2 :
  {
    PROVIDE( _heap_end = . );
    . = ALIGN(8);
    . = . + __EXCEPTION_STACK_SIZE;
    PROVIDE(__CORE0_EXCEPTION_STACK_TOP = .) ;
    . = . + __KERNEL_STACK_SIZE;
    PROVIDE(__CORE0_STACK_TOP = .) ;
    . = . + __EXCEPTION_STACK_SIZE;
    PROVIDE(__CORE1_EXCEPTION_STACK_TOP = .) ;
    . = . + __KERNEL_STACK_SIZE;
    PROVIDE(__CORE1_STACK_TOP = .) ;
  } > RAM
}
