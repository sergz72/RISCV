SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.14)

# specify the cross compiler
SET(CMAKE_C_COMPILER_WORKS 1)

SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(LINKER_SCRIPT ../../Link.ld)

if (MOUNRIVER)
    SET(COMMON_FLAGS " -march=rv32imafcxw -mabi=ilp32f -msmall-data-limit=8 -msave-restore -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -specs=nosys.specs -specs=nano.specs -nostartfiles")
else()
    SET(COMMON_FLAGS " -march=rv32imafc_zifencei_zicsr -mabi=ilp32f -msmall-data-limit=8 -msave-restore -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -specs=nosys.specs -specs=nano.specs -nostartfiles")
endif()

SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")

project(audio_headphone C ASM)

add_definitions(-DCH32V30x_D8C)

file(GLOB SOURCES
        "src/main.c"
        "src/uac10_headphone.c"
        "src/hal.c"
        "src/tlv.c"
        "../lib/i2c_func.c"
        "../../../ARM_CLION/common_lib/audio/tlv320dac3100.c"
        "../lib/USBLIB/audio/ch32v30x_usbfs_device.c"
        "../lib/USBLIB/audio/usb_desc.c"
        "../lib/delay.c"
        "../lib/system_ch32v30x.c"
        "../lib/src/ch32v30x_misc.c"
        "../lib/src/ch32v30x_rcc.c"
        "../lib/src/ch32v30x_gpio.c"
        "../lib/src/ch32v30x_i2c.c"
        "../lib/src/ch32v30x_spi.c"
        "../lib/src/ch32v30x_dma.c"
        "../startup_ch32v30x_D8C.S"
)

include_directories(src ../lib ../lib/inc ../lib/USBLIB/audio ../../../ARM_CLION/common_lib/audio)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
