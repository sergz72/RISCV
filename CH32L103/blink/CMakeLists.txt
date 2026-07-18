SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.14)

set(CMAKE_C_STANDARD 23)

# specify the cross compiler
SET(CMAKE_C_COMPILER_WORKS 1)

SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(LINKER_SCRIPT ../../Link_lp.ld)

SET(COMMON_FLAGS "-march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -specs=nosys.specs -specs=nano.specs -nostartfiles")

SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")

project(blink C ASM)

file(GLOB SOURCES
        "src/main.c"
        "../lib/delay.c"
        "../lib/ch32l103_it.c"
        "../lib/src/ch32l103_misc.c"
        "../lib/src/ch32l103_rcc.c"
        "../lib/src/ch32l103_pwr.c"
        "../lib/src/ch32l103_gpio.c"
        "../lib/src/ch32l103_dbgmcu.c"
        "../startup_ch32l103_lp.S"
)

include_directories(src ../lib ../lib/inc)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")
