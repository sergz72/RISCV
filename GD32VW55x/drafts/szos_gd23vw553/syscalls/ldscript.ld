OUTPUT_ARCH( "riscv" )

ENTRY( main )

/*MEMORY
{
    code (rx): ORIGIN = 0x00000000, LENGTH = 32k
    data (rw): ORIGIN = 0x80000000, LENGTH = 32k
}*/

SECTIONS
{
  . = 0x0;

  .init           :
  {
    KEEP (*(SORT_NONE(.init)))
  }

  .text           :
  {
    *(.rodata .rodata.*)
    *(.text.unlikely .text.unlikely.*)
    *(.text.startup .text.startup.*)
    *(.text .text.*)
    *(.gnu.linkonce.t.*)
  }

  .data          :
  {
    *(.rdata)

    *(.gnu.linkonce.r.*)
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    . = ALIGN(8);
    *(.sdata .sdata.*)
    *(.gnu.linkonce.s.*)
    . = ALIGN(8);
    *(.srodata.cst16)
    *(.srodata.cst8)
    *(.srodata.cst4)
    *(.srodata.cst2)
    *(.srodata .srodata.*)
  }

  PROVIDE( __bss_start = . );
  .bss            :
  {
    *(.sbss*)
    *(.gnu.linkonce.sb.*)
    *(.bss .bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4);
  }

  . = ALIGN(8);
  PROVIDE( __bss_end = . );
}
