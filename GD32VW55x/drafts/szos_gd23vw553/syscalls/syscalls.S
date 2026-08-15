.global osTaskSwitch,osExit,osDelay,osLeds

.section .text

osTaskSwitch:
  li a7, 0
  ecall
  ret

osExit:
  li a7, 1
  ecall
loop:
  j loop

osDelay:
  li a7, 2
  ecall
  ret

osLeds:
  li a7, 3
  ecall
  ret
