SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.14)

set(CMAKE_C_STANDARD 23)

# specify the cross compiler
SET(CMAKE_C_COMPILER_WORKS 1)

SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(LINKER_SCRIPT ../../Link_192_128.ld)
#SET(LINKER_SCRIPT ../../Link.ld)

SET(COMMON_FLAGS " -march=rv32imacfxw -mabi=ilp32f -msmall-data-limit=8 -msave-restore -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -specs=nosys.specs -specs=nano.specs -nostartfiles")

SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")

project(freertos_test C ASM)

add_definitions(-DCH32V30x_D8C)

file(GLOB SOURCES
        "src/main.c"
        "src/hal.c"
        "../lib/delay.c"
        "../lib/system_ch32v30x.c"
        "../lib/src/ch32v30x_misc.c"
        "../lib/src/ch32v30x_rcc.c"
        "../lib/src/ch32v30x_gpio.c"
        "../FreeRTOS/startup_ch32v30x_D8C.S"
        "../../../FreeRTOS/tasks.c"
        "../../../FreeRTOS/list.c"
        "../../../FreeRTOS/timers.c"
        "../../../FreeRTOS/queue.c"
        "../../../FreeRTOS/portable/MemMang/heap_4.c"
        "../FreeRTOS/portable/GCC/RISC-V/port.c"
        "../FreeRTOS/portable/GCC/RISC-V/portASM.S"
)

include_directories(src ../lib ../lib/inc ../../../FreeRTOS/include ../FreeRTOS/portable/GCC/RISC-V
        ../FreeRTOS/portable/GCC/RISC-V/chip_specific_extensions/RV32I_PFIC_no_extensions)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")
