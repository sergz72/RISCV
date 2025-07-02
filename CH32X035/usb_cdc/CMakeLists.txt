SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.14)

set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 23)

# specify the cross compiler
SET(CMAKE_C_COMPILER_WORKS 1)
SET(CMAKE_CXX_COMPILER_WORKS 1)

SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(LINKER_SCRIPT ../../Link.ld)

if (MOUNRIVER)
    SET(COMMON_FLAGS " -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -specs=nosys.specs -specs=nano.specs -nostartfiles")
else()
    SET(COMMON_FLAGS " -march=rv32imac_zicsr -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -specs=nosys.specs -specs=nano.specs -nostartfiles")
endif()

SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")

if (CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DDEBUG)
endif()

project(usb_cdc C CXX ASM)

file(GLOB SOURCES
        "src/main.cpp"
        "src/hal.c"
        "../lib/delay.c"
        "../lib/core_riscv.c"
        "../lib/system_ch32x035.c"
        "../lib/src/ch32x035_misc.c"
        "../lib/src/ch32x035_rcc.c"
        "../lib/src/ch32x035_gpio.c"
        "../startup_ch32x035.S"
        "../../../usblib/usb_device.cpp"
        "../../../usblib/usb_device_ch32x035.cpp"
        "../../../usblib/cdc_class.cpp"
)

include_directories(src src/usblib ../lib ../lib/inc ../../../usblib)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")
