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

project(CH32V203_meter C ASM)

file(GLOB SOURCES
        "src/hal.c"
        "src/main.c"
        "src/internal_devices/dev_internal_pwm.c"
        "../lib/debug.c"
        "../lib/delay.c"
        "../lib/system_ch32v20x.c"
        "../lib/src/ch32v20x_misc.c"
        "../lib/src/ch32v20x_rcc.c"
        "../lib/src/ch32v20x_gpio.c"
        "../lib/src/ch32v20x_exti.c"
        "../lib/src/ch32v20x_tim.c"
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
        ../../../ARM_CLION/meter_core/core_main.c
        ../../../ARM_CLION/meter_core/device_config.c
        ../../../ARM_CLION/meter_core/device_list.c
        ../../../ARM_CLION/meter_core/devices.c
        ../../../ARM_CLION/meter_core/i2c_soft_hal.c
        ../../../ARM_CLION/meter_core/devices/dev_ads1115.c
        ../../../ARM_CLION/meter_core/devices/dev_dds.c
        ../../../ARM_CLION/meter_core/devices/dev_ina226.c
        ../../../ARM_CLION/meter_core/devices/dev_ina3221.c
        ../../../ARM_CLION/meter_core/devices/dev_mcp3421.c
        ../../../ARM_CLION/meter_core/devices/dev_mcp9600.c
        ../../../ARM_CLION/meter_core/devices/dev_si5351.c
        ../../../ARM_CLION/meter_core/devices/dev_pwm.c
        ../../../ARM_CLION/meter_core/devices/dev_meter.c
        ../../../ARM_CLION/common_lib/i2c_soft.c
        ../../../ARM_CLION/common_lib/spi_soft.c
        ../../../ARM_CLION/common_lib/sensor/ads1115.c
        ../../../ARM_CLION/common_lib/sensor/mcp3421.c
        ../../../ARM_CLION/common_lib/sensor/ina226.c
        ../../../ARM_CLION/common_lib/sensor/ina3221.c
        ../../../ARM_CLION/common_lib/sensor/mcp9600.c
        ../../../ARM_CLION/common_lib/rf/si5351.c
        ../../../ARM_CLION/common_lib/memory/24c01_16.c
)

include_directories(
        src
        src/internal_devices
        ../lib
        ../lib/inc
        ../lib/USBLIB/USB-Driver/inc
        ../lib/USBLIB/CONFIG
        ../lib/USBLIB/USB_CDC
        ../../../ARM_CLION/common_lib
        ../../../ARM_CLION/common_lib/sensor
        ../../../ARM_CLION/common_lib/rf
        ../../../ARM_CLION/common_lib/memory
        ../../../ARM_CLION/meter_core
        ../../../ARM_CLION/meter_core/devices
)

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
