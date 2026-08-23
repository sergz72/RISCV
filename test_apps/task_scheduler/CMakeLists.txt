cmake_minimum_required(VERSION 4.3)
project(task_scheduler C ASM)

set(CMAKE_C_STANDARD 23)

add_executable(task_scheduler main.c os.c os.S)
