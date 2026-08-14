.global osExit,osDelay,osLeds

.section .text

osExit:
  li a7, 0
  ecall
loop:
  j loop

osDelay:
  li a7, 1
  ecall
  ret

osLeds:
  li a7, 2
  ecall
  ret
