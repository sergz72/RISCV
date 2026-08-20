#! /bin/sh

gcc -shared -o libsyscalls_x86.so -nostdlib syscalls_x86.S
