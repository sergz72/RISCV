.syntax unified

.cpu cortex-m33

.thumb_func
.section .riscv_intvect, "ax"
.globl _VectoredInterruptVectorTable

_VectoredInterruptVectorTable:
