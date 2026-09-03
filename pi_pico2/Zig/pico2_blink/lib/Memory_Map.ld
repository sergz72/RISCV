/******************************************************************************************
  Filename    : Memory_Map.ld

  Core        : ARM Cortex-M33 / RISC-V Hazard3

  MCU         : RP2350

  Author      : Chalandi Amine

  Owner       : Chalandi Amine

  Date        : 04.09.2024

  Description : Linker description file for Raspberry Pi Pico 2

******************************************************************************************/

/******************************************************************************************
 ELF Entrypoint
******************************************************************************************/
ENTRY(Startup_Init)

/******************************************************************************************
 Globals
******************************************************************************************/
__STACK_SIZE_CORE0 = 8K;
__STACK_SIZE_CORE1 = 8K;
__EXCEPTION_STACK_SIZE = 4K;

/******************************************************************************************
 Memory configuration
******************************************************************************************/

MEMORY
{
  FLASH(rx)   : ORIGIN = 0x10000000, LENGTH = 4M
  RAM(rwx)    : ORIGIN = 0x20000000, LENGTH = 520K
}

/******************************************************************************************
 Sections definition
******************************************************************************************/
SECTIONS
{
  /* Program code */
  .program : ALIGN(4)
  {
    PROVIDE(__CODE_BASE_ADDRESS = .);
    PROVIDE(__INTVECT_BASE_ADDRESS = .);
    *(.riscv_intvect)
    KEEP(*(.riscv_intvect))
    *(.intvect_c0)
    *(.intvect_c1)
    KEEP(*(.intvect_c0))
    KEEP(*(.intvect_c1))
    PROVIDE(__IMAGE_DEF_BLOCK_START_ADDRESS = .);
    KEEP(*(.image_start_block))
    *(.text)
    *(.text*)
  } > FLASH

  /* Read-only data (.rodata) */
  .rodata : ALIGN(4)
  {
    PROVIDE(__RODATA_BASE_ADDRESS = .);
    *(.rodata)
  } > FLASH

 /* Section for constructors */
  .ctors : ALIGN(4)
  {
    PROVIDE(__CPPCTOR_LIST__ = .);
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    LONG(-1) ;
    PROVIDE(__CPPCTOR_END__ = .);
    . = ALIGN(4);
  }  > FLASH


  /* Section for destructors */
  .dtors : ALIGN(4)
  {
    PROVIDE(__CPPDTOR_LIST__ = .);
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array))
    LONG(-1) ;
    PROVIDE(__CPPDTOR_END__ = .);
    . = ALIGN(4);
  } > FLASH

  /* Runtime clear table */
  .clear_sec : ALIGN(4)
  {
    PROVIDE(__RUNTIME_CLEAR_TABLE = .) ;
    LONG(0 + ADDR(.bss));   LONG(SIZEOF(.bss));
    LONG(-1);                 LONG(-1);
    . = ALIGN(4);
  } > FLASH

  /* Runtime copy table */
  .copy_sec : ALIGN(4)
  {
    PROVIDE(__RUNTIME_COPY_TABLE = .) ;
    LONG(LOADADDR(.data));  LONG(0 + ADDR(.data));  LONG(SIZEOF(.data));
    LONG(-1);                 LONG(-1);                  LONG(-1);
    . = ALIGN(4);
  } > FLASH

  /* The ROM-to-RAM initialized data section */
  .data :  ALIGN(4)
  {
    PROVIDE(__DATA_BASE_ADDRESS = .);
    PROVIDE( __global_pointer$ = . + 0x100);
    *(.data)
    *(.data*)
  } > RAM  AT>FLASH

  /* The uninitialized (zero-cleared) bss section */
  .bss : ALIGN(4)
  {
    PROVIDE(__BSS_BASE_ADDRESS = .);
    *(.bss)
    *(.bss*)
    . = ALIGN(8);
    PROVIDE(__BSS_END = .);
    . = . + __EXCEPTION_STACK_SIZE;
    PROVIDE(__EXCEPTION_STACK_TOP = .) ;
    . = . + __STACK_SIZE_CORE0;
    PROVIDE(__CORE0_STACK_TOP = .) ;
    . = ALIGN(8);
    . = . + __STACK_SIZE_CORE1;
    PROVIDE(__CORE1_STACK_TOP = .) ;
  } > RAM
}
