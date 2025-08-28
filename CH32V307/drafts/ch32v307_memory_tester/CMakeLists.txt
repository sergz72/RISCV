SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.14)

# specify the cross compiler
SET(CMAKE_C_COMPILER_WORKS 1)

SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(LINKER_SCRIPT ../../Link.ld)

if (MOUNRIVER)
    SET(COMMON_FLAGS " -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -specs=nosys.specs -specs=nano.specs -nostartfiles")
else()
    SET(COMMON_FLAGS " -march=rv32imac_zicsr -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -specs=nosys.specs -specs=nano.specs -nostartfiles")
endif()

SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")

project(ch32v307_memory_tester C ASM)

add_definitions(-DCH32V30x_D8C)

file(GLOB SOURCES
        "src/main.c"
        "src/hal.c"
        "src/i2c_commands.c"
        "src/spi_commands.c"
        "../lib/delay.c"
        "../lib/i2c_func.c"
        "../lib/system_ch32v30x.c"
        "../lib/src/ch32v30x_misc.c"
        "../lib/src/ch32v30x_rcc.c"
        "../lib/src/ch32v30x_gpio.c"
        "../lib/src/ch32v30x_spi.c"
        "../lib/src/ch32v30x_i2c.c"
        "../startup_ch32v30x_D8C.S"
        "../lib/USBLIBHS/cdc/ch32v30x_usbhs_device.c"
        "../lib/USBLIBHS/cdc/usb_desc.c"
        "../lib/USBLIBHS/cdc/usb_cdc.c"
        "../../../ARM_CLION/common_lib/shell/shell.c"
        "../../../ARM_CLION/common_lib/memory/spi_memory.c"
        "../../../ARM_CLION/common_lib/getstring.c"
        "../../../ARM_CLION/common_lib/fixed_queue.c"
        "../../../ARM_CLION/common_lib/crypto/chacha.c"
)

include_directories(src ../lib ../lib/inc ../lib/USBLIBHS/cdc ../../../ARM_CLION/common_lib
        ../../../ARM_CLION/common_lib/shell ../../../ARM_CLION/common_lib/crypto ../../../ARM_CLION/common_lib/memory)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")
