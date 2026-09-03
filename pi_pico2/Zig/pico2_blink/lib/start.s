.equ PICOBIN_BLOCK_MARKER_START,        0xffffded3
.equ PICOBIN_BLOCK_MARKER_END,          0xab123579
.equ BOOTROM_ENTRY_OFFSET,              0x7dfc

.section .image_start_block, "a"
.align 3

// Mandatory RP2350 RISC-V Boot Block Header
embedded_block:
.word PICOBIN_BLOCK_MARKER_START

### Item 1
.byte   0x42    # Type: Image definition
.byte   0x01    # Size: 1 word
.hword  0x1101  # Flags: EXE | RISC-V | RP2350

### Item 2
.byte   0x44            # Type: Entry Point definition
.byte   0x03            # Size: 3 words
.hword  0x00            # 16-bit pad
.word _reset_handler    # Initial PC address
.word __CORE0_STACK_TOP # Initial SP address

### Item 3
.byte   0xff    # Type: BLOCK_ITEM_LAST
# Other items' size
.hword  (embedded_block_end - embedded_block - 16) / 4
.byte   0x00    # 8-bit pad

### Link (0 == to self)
.word 0

.word PICOBIN_BLOCK_MARKER_END
embedded_block_end:

.section .text, "ax"
.global _entry_point
.global _reset_handler
_entry_point:
    li      t0, BOOTROM_ENTRY_OFFSET
    jr      t0

_reset_handler:
    /* disable all interrupts flag */
    li t0, ~0x08
    csrc mstatus, t0

    /* disable all specific interrupt sources */
    csrw mie, x0

    // setup interrupt handlers
    // mtvec bit 0 = VECTORED: Vectored entry to a 16-entry jump table starting at mtvec
    la t0, _VectoredInterruptVectorTable
    addi t0, t0, 1
    csrw mtvec, t0

    // copy data segment to RAM
    la t0, __DATA_BASE_ADDRESS
    la t1, __BSS_BASE_ADDRESS
    bgeu t0, t1, copy_end
copy_loop:
    lw   t2, (t0)
    sw   t2, (t1)
    addi t0, t0, 4
    addi t1, t1, 4
    bltu t0, t1, copy_loop
copy_end:
    // clear bss
    la   t0, __BSS_BASE_ADDRESS
    la   t1, __BSS_END
    bgeu t0, t1, zero_end
bss_fill_loop:
    sw   x0, (t0)
    addi t0, t0, 4
    bltu t0, t1, bss_fill_loop

zero_end:
    la   sp, __CORE0_STACK_TOP
    la   gp, __global_pointer$
    csrw mscratch, zero
    j  main
