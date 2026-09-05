.section .text, "ax"

.equ T6,      72
.equ T5,      68
.equ T4,      64
.equ T3,      60
.equ A7,      56
.equ A6,      52
.equ A5,      48
.equ A4,      44
.equ A3,      40
.equ A2,      36
.equ A1,      32
.equ A0,      28
.equ T2,      24
.equ T1,      20
.equ T0,      16
.equ RA,      12
.equ GP,      8
.equ SP,      4
.equ MEPC,    0

.equ CPUID_ADDRESS, 0xD0000000

.macro SAVE_REGS
  sw   t6, T6(sp)
  sw   t5, T5(sp)
  sw   t4, T4(sp)
  sw   t3, T3(sp)
  sw   a7, A7(sp)
  sw   a6, A6(sp)
  sw   a5, A5(sp)
  sw   a4, A4(sp)
  sw   a3, A3(sp)
  sw   a2, A2(sp)
  sw   a1, A1(sp)
  sw   a0, A0(sp)
  sw   t2, T2(sp)
  sw   t1, T1(sp)
  sw   t0, T0(sp)
  sw   ra, RA(sp)
  sw   gp, GP(sp)
  la   gp, __global_pointer$
  csrr t0, mepc
  sw   t0, MEPC(sp)
.endm

.macro RESTORE_REGS
  csrci mstatus, 8 // disable interrupts
  lw   t0, MEPC(sp)
  csrw mepc, t0
  lw   t6, T6(sp)
  lw   t5, T5(sp)
  lw   t4, T4(sp)
  lw   t3, T3(sp)
  lw   a7, A7(sp)
  lw   a6, A6(sp)
  lw   a5, A5(sp)
  lw   a4, A4(sp)
  lw   a3, A3(sp)
  lw   a2, A2(sp)
  lw   a1, A1(sp)
  lw   a0, A0(sp)
  lw   t2, T2(sp)
  lw   t1, T1(sp)
  lw   t0, T0(sp)
  lw   ra, RA(sp)
  lw   gp, GP(sp)
.endm

.macro IRQ_HANDLER func
  csrrw t0, mscratch, t0
  beqz  t0, .set_sp\@
  csrrw t0, mscratch, t0

  addi sp, sp, -T6-4

  SAVE_REGS

  jal  \func

  RESTORE_REGS

  addi sp, sp, T6+4
  mret

.set_sp\@:
  csrr t0, mscratch
  csrw mscratch, sp
  la sp, CPUID_ADDRESS
  lw sp, (sp)
  bnez sp, .cpu1\@
  la sp, __CORE0_EXCEPTION_STACK_TOP - T0 - 4
  j .save\@
.cpu1\@:
  la sp, __CORE1_EXCEPTION_STACK_TOP - T0 - 4
.save\@:

  SAVE_REGS

  csrr t0, mscratch
  sw   t0, SP(sp)

  jal  \func

  RESTORE_REGS

  lw   sp, SP(sp)
  csrw mscratch, zero
  mret
.endm

.type UndefinedInterruptHandler, @function
.type Exception, @function
.type MachineSoftwareInterrupt, @function
.type MachineTimerInterrupt, @function
.type MachineExternalInterrupt, @function

.align 2
UndefinedInterruptHandler:
    j UndefinedInterruptHandler

.align 2
Exception:
    j Exception

.align 2
MachineSoftwareInterrupt:
    IRQ_HANDLER MachineSoftwareInterruptHandler

.align 2
MachineTimerInterrupt:
    IRQ_HANDLER MachineTimerInterruptHandler

.align 2
MachineExternalInterrupt:
    IRQ_HANDLER MachineExternalInterruptHandler

.section .riscv_intvect, "ax"
.globl _VectoredInterruptVectorTable
//The pointer written to mtvec must be word-aligned (4 bytes).
//Additionally, when vectoring is enabled, it must be aligned to the size of the table, rounded up to a power of two. This works out to 64-byte alignment.
.align 6
.type _VectoredInterruptVectorTable, @function

_VectoredInterruptVectorTable:
                                j Exception
                                j UndefinedInterruptHandler
                                j UndefinedInterruptHandler
                                j MachineSoftwareInterrupt
                                j UndefinedInterruptHandler
                                j UndefinedInterruptHandler
                                j UndefinedInterruptHandler
                                j MachineTimerInterrupt
                                j UndefinedInterruptHandler
                                j UndefinedInterruptHandler
                                j UndefinedInterruptHandler
                                j MachineExternalInterrupt
                                j UndefinedInterruptHandler
                                j UndefinedInterruptHandler
                                j UndefinedInterruptHandler
                                j UndefinedInterruptHandler
