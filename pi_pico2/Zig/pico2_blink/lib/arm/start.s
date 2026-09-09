.syntax unified

.cpu cortex-m33

.equ PICOBIN_BLOCK_MARKER_START,        0xffffded3
.equ PICOBIN_BLOCK_MARKER_END,          0xab123579

.section .image_start_block, "a"
.align 3

// Mandatory RP2350 Boot Block Header
embedded_block:
.word PICOBIN_BLOCK_MARKER_START

// Item 1
.byte   0x42    // Type: Image definition
.byte   0x01    // Size: 1 word
.hword  0x0121  // Flags: EXE | ARM | RP2350

// Item 2
.byte   0x44            // Type: Entry Point definition
.byte   0x03            // Size: 3 words
.hword  0x00            // 16-bit pad
.word _reset_handler    // Initial PC address
.word __CORE0_STACK_TOP // Initial SP address

// Item 3
.byte   0xff    // Type: BLOCK_ITEM_LAST
// Other items' size
.hword  (embedded_block_end - embedded_block - 16) / 4
.byte   0x00    // 8-bit pad

// Link (0 == to self)
.word 0

.word PICOBIN_BLOCK_MARKER_END
embedded_block_end:

.thumb_func
.section .text, "ax"
.global _entry_point
.global _reset_handler
_entry_point:
    bx lr

_reset_handler:
    bx lr

.globl cpu1_reset_handler
cpu1_reset_handler:
    bx lr
