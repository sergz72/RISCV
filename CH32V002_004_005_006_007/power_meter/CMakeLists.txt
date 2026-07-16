SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.14)

set(CMAKE_C_STANDARD 23)

# specify the cross compiler
SET(CMAKE_C_COMPILER_WORKS 1)

SET(CMAKE_VERBOSE_MAKEFILE ON)

if (CH32V002)
    SET(LINKER_SCRIPT ../../Link_002.ld)
    add_definitions(-DCH32V002)
else()
    SET(LINKER_SCRIPT ../../Link_006_007.ld)
endif()

SET(COMMON_FLAGS "-march=rv32ecxw_zmmul -mabi=ilp32e -msmall-data-limit=8 -mno-save-restore -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -specs=nosys.specs -specs=nano.specs -nostartfiles")

SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")

project(power_meter C ASM)

file(GLOB SOURCES
        "src/main.c"
        "src/hal.c"
        "src/usart_handler.c"
        "src/ina_func.c"
        "src/flash_func.c"
        "src/ui.c"
        "../lib/delay.c"
        "../lib/i2c.c"
        "../lib/usart.c"
        "../lib/system_ch32v00X.c"
        "../lib/src/ch32v00X_misc.c"
        "../lib/src/ch32v00X_rcc.c"
        "../lib/src/ch32v00X_gpio.c"
        "../lib/src/ch32v00X_tim.c"
        "../lib/src/ch32v00X_usart.c"
        "../lib/src/ch32v00X_i2c.c"
        "../lib/src/ch32v00X_spi.c"
        "../startup_ch32v00X.S"
        "../../../common_lib/common_printf.c"
        "../../../common_lib/myprintf.c"
        "../../../common_lib/ultoa.c"
        "../../../common_lib/spi_soft.c"
        "../../../common_lib/adc/ina228.c"
        "../../../common_lib/display/lcd.c"
        "../../../common_lib/display/lcd_ssd1306.c"
        "../../../common_lib/display/font.c"
        "../../../common_lib/display/fonts/font12.c"
        "../../../common_lib/display/display2.c"
        "../../../common_lib/memory/spi_memory.c"
)

include_directories(src ../lib ../lib/inc ../../../common_lib
        ../../../common_lib/adc ../../../common_lib/display ../../../common_lib/memory)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")
