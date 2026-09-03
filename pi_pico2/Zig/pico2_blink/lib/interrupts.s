.section .text, "ax"

.equ T0,      16
.equ RA,      12
.equ GP,      8
.equ SP,      4
.equ MEPC,    0

.macro IRQ_HANDLER func
    csrrw t0, mscratch, t0
    beqz  t0, .set_sp\@
    csrrw t0, mscratch, t0

    addi sp, sp, -T0-4
  sw   t0, T0(sp)
  sw   ra, RA(sp)
  sw   gp, GP(sp)
  la   gp, __global_pointer$
  csrr t0, mepc
  sw   t0, MEPC(sp)

  csrsi mstatus, 8 // enable interrupts

  jal  MachineExternalInterruptHandler

  csrci mstatus, 8 // disable interrupts
  lw   t0, MEPC(sp)
  csrw mepc, x5
  lw   t0, T0(sp)
  lw   ra, RA(sp)
  lw   gp, GP(sp)
  addi sp, sp, T0+4
  mret

.set_sp\@:
    csrr t0, mscratch
    csrw mscratch, sp
    la sp, __EXCEPTION_STACK_TOP - T0 - 4
/*stack:
  t0
  ra
  gp
  old_sp
  mepc
*/
  sw   t0, T0(sp)
  sw   ra, RA(sp)
  sw   gp, GP(sp)
  la   gp, __global_pointer$
  csrr t0, mscratch
  sw   t0, SP(sp)
  csrr t0, mepc
  sw   t0, MEPC(sp)

  csrsi mstatus, 8 // enable interrupts

  jal  \func

  csrci mstatus, 8 // disable interrupts
  lw   t0, MEPC(sp)
  csrw mepc, t0
  lw   t0, T0(sp)
  lw   ra, RA(sp)
  lw   gp, GP(sp)
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
