SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.14)

set(CMAKE_C_STANDARD 23)

# specify the cross compiler
SET(CMAKE_C_COMPILER_WORKS 1)

SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(LINKER_SCRIPT ../../GD32VW553xM.lds)

SET(COMMON_FLAGS "-march=rv32imafdc_zba_zbb_zbc_zbs_xxldspn1x -mtune=nuclei-300-series -mcmodel=medany -msmall-data-limit=8 -mdiv -mabi=ilp32d -msave-restore -fmessage-length=0 -fsigned-char -fno-common -ffunction-sections -fdata-sections -Wunused -Wuninitialized -specs=nosys.specs -specs=nano.specs -nostartfiles")

SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")

project(szos_gd32vw553 C ASM)

file(GLOB SOURCES
        "src/main.c"
        "src/hal.c"
        "src/exceptions.c"
        "src/pmp.c"
        "src/sys_timer.c"
        "src/system_commands.c"
        "src/entry.S"
        "../../../common_lib/shell/shell.c"
        "../../../common_lib/fixed_queue.c"
        "../../../common_lib/getstring.c"
        "../../../common_lib/common_printf.c"
        "../start.S"
        "../entry.S"
        "../init.c"
        "../lib/system_gd32vw55x.c"
        "../lib/Source/gd32vw55x_eclic.c"
        "../lib/Source/gd32vw55x_gpio.c"
        "../lib/Source/gd32vw55x_rcu.c"
        "../lib/Source/gd32vw55x_usart.c"
)

include_directories(src .. ../lib ../lib/Include ../lib/NMSIS/Core/Include ../../../common_lib ../../../common_lib/shell)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")
