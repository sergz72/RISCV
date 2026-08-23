#include "exceptions.h"

.equ X5,     20
.equ SP,      16
.equ MSTATUS, 12
.equ MEPC,    8
.equ X6,      4
.equ MCAUSE,  0

.section .text.trap
/* In CLIC mode, the exeception entry must be 64bytes aligned */
.align 6
.global exc_entry
exc_entry:
  csrw mscratch, sp
  la sp, exception_stack + EXCEPTION_STACK_SIZE * 4 - X5 - 4
/*stack:
  x5
  old_sp
  mstatus
  mepc
  x6
  mcause
*/
  sw   x5, X5(sp)
  csrr x5, mscratch
  sw   x5, SP(sp)
  csrr x5, mstatus
  sw x5,   MSTATUS(sp)
  csrr x5, mepc
  sw x5,   MEPC(sp)
  sw   x6, X6(sp)
  li   x5, 8 //UmodeEcall_EXCn
  csrr x6, mcause
  sw   x6, MCAUSE(sp)

  csrsi mstatus, 8 // enable interrupts

  andi x6, x6, 0x7FF
  bne  x5, x6, exc_other
  beq  a7, zero, exc_other // osDelay ecall
  jal  ecall_handler
  lw   sp, SP(sp)
  mret

exc_other:
  la x5, current_task_data
  lw x5, 0(x5)
  lw x6, MCAUSE(sp) // mcause
  sw x6, 0(x5)
  sw x1, 4(x5)
  lw x6, SP(sp) // sp
  sw x6, 8(x5)
  sw x3, 12(x5)
  sw x4, 16(x5)
  lw x1, X5(sp)  // x5
  sw x1, 20(x5)
  lw x1, X6(sp)  // x6
  sw x1, 24(x5)
  sw x7, 28(x5)

  sw x8, 32(x5)
  sw x9, 36(x5)
  sw x10, 40(x5)
  sw x11, 44(x5)
  sw x12, 48(x5)
  sw x13, 52(x5)
  sw x14, 56(x5)
  sw x15, 60(x5)

  sw x16, 64(x5)
  sw x17, 68(x5)
  sw x18, 72(x5)
  sw x19, 76(x5)
  sw x20, 80(x5)
  sw x21, 84(x5)
  sw x22, 88(x5)
  sw x23, 92(x5)

  sw x24, 96(x5)
  sw x25, 100(x5)
  sw x26, 104(x5)
  sw x27, 108(x5)
  sw x28, 112(x5)
  sw x29, 116(x5)
  sw x30, 120(x5)
  sw x31, 124(x5)

  lw x6, MEPC(sp) // mepc
  sw x6, 128(x5)
  lw x6, MSTATUS(sp) // mstatus
  sw x6, 132(x5)
  addi sp, sp, X5 + 4

  jal  exc_handler

  csrci mstatus, 8 // disable interrupts

  la x5, current_task_data
  lw x5, 0(x5)

  lw x6, 132(x5)
  csrw mstatus, x6

  lw x6, 128(x5)
  addi x6, x6, 4
  csrw mepc, x6

  lw x1, 4(x5)
  lw x2, 8(x5)
  lw x3, 12(x5)
  lw x4, 16(x5)
  lw x6, 24(x5)
  lw x7, 28(x5)

  lw x8, 32(x5)
  lw x9, 36(x5)
  lw x10, 40(x5)
  lw x11, 44(x5)
  lw x12, 48(x5)
  lw x13, 52(x5)
  lw x14, 56(x5)
  lw x15, 60(x5)

  lw x16, 64(x5)
  lw x17, 68(x5)
  lw x18, 72(x5)
  lw x19, 76(x5)
  lw x20, 80(x5)
  lw x21, 84(x5)
  lw x22, 88(x5)
  lw x23, 92(x5)

  lw x24, 96(x5)
  lw x25, 100(x5)
  lw x26, 104(x5)
  lw x27, 108(x5)
  lw x28, 112(x5)
  lw x29, 116(x5)
  lw x30, 120(x5)
  lw x31, 124(x5)

  lw x5, 16(x5)

  mret
