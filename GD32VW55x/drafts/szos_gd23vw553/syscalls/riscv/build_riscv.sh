#! /bin/sh

riscv64-linux-gnu-gcc -march=rv32imacfd -mabi=ilp32d -shared -nostdlib -o syscalls.so syscalls.S
