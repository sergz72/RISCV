SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.14)

# specify the cross compiler
SET(CMAKE_C_COMPILER_WORKS 1)

SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(LINKER_SCRIPT ../../Link_192_128.ld)

if (MOUNRIVER)
    SET(COMMON_FLAGS " -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -specs=nosys.specs -specs=nano.specs -nostartfiles")
else()
    SET(COMMON_FLAGS " -march=rv32imac_zicsr -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -specs=nosys.specs -specs=nano.specs -nostartfiles")
endif()

SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")

project(ethernet_test C ASM)

add_definitions(-DCH32V30x_D8C)

file(GLOB SOURCES
        "src/main.c"
        "src/hal.c"
        "src/ethernet_commands.c"
        "../lib/delay.c"
        "../lib/system_ch32v30x.c"
        "../lib/src/ch32v30x_misc.c"
        "../lib/src/ch32v30x_rcc.c"
        "../lib/src/ch32v30x_gpio.c"
        "../lib/src/ch32v30x_exti.c"
        "../lib/src/ch32v30x_tim.c"
        "../lib/src/ch32v30x_eth.c"
        "../lib/src/ch32v30x_dbgmcu.c"
        "../lib/src/ch32v30x_flash.c"
        "../startup_ch32v30x_D8C.S"
        "../lib/USBLIBFS/cdc/ch32v30x_usbfs_device.c"
        "../lib/USBLIBFS/cdc/usb_desc.c"
        "../lib/USBLIBFS/cdc/usb_cdc.c"
        "../../../common_lib/shell/shell.c"
        "../../../common_lib/getstring.c"
        "../../../common_lib/fixed_queue.c"
        "../lib/netlib/eth_driver_10M.c"
)

include_directories(src ../lib ../lib/inc ../lib/USBLIBFS/cdc ../../../common_lib ../../../common_lib/shell ../lib/netlib)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

target_link_libraries(${PROJECT_NAME}.elf PRIVATE ${CMAKE_SOURCE_DIR}/../lib/netlib/libwchnet.a)

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")
