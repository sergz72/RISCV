OUTPUT_ARCH( "riscv" )

ENTRY( main )

SECTIONS
{
  . = 0x0;
  
  .text           :
  {
    KEEP (*(SORT_NONE(.init)))
    *(.rodata .rodata.*)
    *(.text.unlikely .text.unlikely.*)
    *(.text.startup .text.startup.*)
    *(.text .text.*)
    *(.gnu.linkonce.t.*)

    *(.rdata)

    *(.gnu.linkonce.r.*)
    . = ALIGN(8);
    *(.srodata.cst16)
    *(.srodata.cst8)
    *(.srodata.cst4)
    *(.srodata.cst2)
    *(.srodata .srodata.*)
  }

  .data          :
  {
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    . = ALIGN(8);
    *(.sdata .sdata.*)
    *(.gnu.linkonce.s.*)
  }

  .bss           :
  {
    *(.sbss*)
    *(.gnu.linkonce.sb.*)
    *(.bss .bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
  }
}
