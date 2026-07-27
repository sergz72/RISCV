SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.14)

# specify the cross compiler
SET(CMAKE_C_COMPILER_WORKS 1)

SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(LINKER_SCRIPT ../../Link.ld)

SET(COMMON_FLAGS "-march=rv32imac_zicsr -ffunction-sections -fdata-sections -specs=nosys.specs -specs=nano.specs -nostartfiles")

SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")

project(usb_cdc C ASM)

file(GLOB SOURCES
        "src/*.c"
        "../lib/debug.c"
        "../lib/delay.c"
        "../lib/system_ch32v20x.c"
        "../lib/src/ch32v20x_misc.c"
        "../lib/src/ch32v20x_rcc.c"
        "../lib/src/ch32v20x_gpio.c"
        "../lib/src/ch32v20x_exti.c"
        "../startup_ch32v20x_D6.S"
        "../lib/USBLIB/USB-Driver/src/usb_core.c"
        "../lib/USBLIB/USB-Driver/src/usb_init.c"
        "../lib/USBLIB/USB-Driver/src/usb_int.c"
        "../lib/USBLIB/USB-Driver/src/usb_mem.c"
        "../lib/USBLIB/USB-Driver/src/usb_regs.c"
        "../lib/USBLIB/USB-Driver/src/usb_sil.c"
        "../lib/USBLIB/CONFIG/hw_config.c"
        "../lib/USBLIB/CONFIG/usb_istr.c"
        "../lib/USBLIB/CONFIG/usb_pwr.c"
        "../lib/USBLIB/USB_CDC/usb_desc.c"
        "../lib/USBLIB/USB_CDC/usb_endp.c"
        "../lib/USBLIB/USB_CDC/usb_prop.c"
)

include_directories(src ../lib ../lib/inc ../lib/USBLIB/USB-Driver/inc ../lib/USBLIB/CONFIG ../lib/USBLIB/USB_CDC)

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
