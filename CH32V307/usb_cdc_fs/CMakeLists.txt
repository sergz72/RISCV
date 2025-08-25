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

project(usb_cdc_fs C ASM)

add_definitions(-DCH32V30x_D8C)

file(GLOB SOURCES
        "src/*.c"
        "../lib/debug.c"
        "../lib/delay.c"
        "../lib/system_ch32v30x.c"
        "../lib/src/ch32v30x_misc.c"
        "../lib/src/ch32v30x_rcc.c"
        "../lib/src/ch32v30x_gpio.c"
        "../lib/src/ch32v30x_exti.c"
        "../startup_ch32v30x_D8C.S"
        "../lib/USBLIBFS/cdc/ch32v30x_usbfs_device.c"
        "../lib/USBLIBFS/cdc/usb_desc.c"
        "../lib/USBLIBFS/cdc/usb_cdc.c"
)

include_directories(src ../lib ../lib/inc ../lib/USBLIBFS/cdc)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")
