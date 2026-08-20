OUTPUT_ARCH(i386:x86-64)

ENTRY(main)

PHDRS
{
  text PT_LOAD FLAGS(5);      /* Read + Execute */
  data PT_LOAD FLAGS(6);      /* Read + Write */
  dynamic PT_DYNAMIC;         /* Dynamic linking information */
  interp PT_INTERP;           /* Path to the program interpreter */
}

MEMORY
{
    flash (rx) : ORIGIN = 0,          LENGTH = FLASH_SIZE
    ram   (rw) : ORIGIN = FLASH_SIZE, LENGTH = RAM_SIZE
}

SECTIONS
{
  /* --- PROGRAM CODE & READ-ONLY DATA --- */
  
  .text : {
    _stext = .;
    *(.text .stub .text.* .gnu.linkonce.t.*)
    _etext = .;
  } >flash : text

  /* Procedure Linkage Table (PLT) - Trampolines for external function calls */
  .plt : { *(.plt) } >flash : text

  /* Global Offset Table (GOT) - Holds pointers to external variables/functions */
  .got : { *(.got.plt) *(.got) } >flash : text

  .rodata : { *(.rodata .rodata.* .gnu.linkonce.r.*) } >flash : text

  /* --- DYNAMIC LINKING SYSTEM SECTIONS --- */

  /* Relocation sections for dynamic symbols */
  .rela.dyn : {
    *(.rela.text .rela.text.*)
    *(.rela.rodata .rela.rodata.*)
    *(.rela.data .rela.data.*)
    *(.rela.got)
    *(.rela.bss .rela.bss.*)
  } >flash : text
  
  /* Specifies the path to the dynamic linker (e.g., /lib/ld-linux-riscv32-ilp32.so.1) */
  .interp : { *(.interp) } >flash : text : interp

  /* Hash tables for fast symbol lookup at runtime */
  .hash     : { *(.hash) } >flash    : text
  .gnu.hash : { *(.gnu.hash) } >flash : text

  /* Dynamic Symbol Table and String Table */
  .dynsym   : { *(.dynsym) } >flash  : text
  .dynstr   : { *(.dynstr) } >flash  : text

  /* Versioning tracking sections */
  .gnu.version    : { *(.gnu.version) } >flash   : text
  .gnu.version_d  : { *(.gnu.version_d) } >flash : text
  .gnu.version_r  : { *(.gnu.version_r) } >flash : text

  .rela.plt : { *(.rela.plt) } >flash : text

  /* --- READ-WRITE DATA & TABLES --- */

  /* Dynamic Array Section - tells the loader where tables are located */
  .dynamic : { *(.dynamic) } >flash : data : dynamic

  .data : {
    _sdata = .;
    *(.data .data.* .gnu.linkonce.d.*)
    _edata = .;
  } >ram : data

  /* RISC-V Small Data sections (requires 'gp' global pointer initialization) */
  .sdata : {
    __global_pointer$ = .;
    *(.sdata .sdata.* .gnu.linkonce.s.*)
  } >ram : data

  /* Uninitialized Data (Zero filled) */
  .sbss : { *(.sbss .sbss.* .gnu.linkonce.sb.*) } >ram : data
  
  .bss : {
    _sbss = .;
    *(.bss .bss.* .gnu.linkonce.b.*)
    _ebss = .;
  } >ram : data

  /* Discard unused debugging and note symbols if needed */
  /DISCARD/ : { *(.note.GNU-stack) *(.comment) }
}
