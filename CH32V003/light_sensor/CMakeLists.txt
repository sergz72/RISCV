SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.14)

set(CMAKE_C_STANDARD 23)

# specify the cross compiler
SET(CMAKE_C_COMPILER_WORKS 1)

SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(LINKER_SCRIPT ../../Link.ld)

SET(COMMON_FLAGS "-march=rv32ecxw -mabi=ilp32e -msmall-data-limit=8 -mno-save-restore -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -specs=nosys.specs -specs=nano.specs -nostartfiles")

SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")

project(light_sensor C ASM)

file(GLOB SOURCES
        "src/*.c"
        "../lib/delay.c"
        "../lib/core_riscv.c"
        "../lib/system_ch32v00x.c"
        "../lib/src/ch32v00x_misc.c"
        "../lib/src/ch32v00x_rcc.c"
        "../lib/src/ch32v00x_gpio.c"
        "../lib/src/ch32v00x_i2c.c"
        "../startup_ch32v00x.S"
        "../../../ARM_CLION/common_lib/myprintf.c"
        "../../../ARM_CLION/common_lib/ultoa.c"
        "../../../ARM_CLION/common_lib/display/lcd.c"
        "../../../ARM_CLION/common_lib/display/lcd_ssd1306.c"
        "../../../ARM_CLION/common_lib/display/font.c"
        "../../../ARM_CLION/common_lib/display/fonts/font18.c"
        "../../../ARM_CLION/common_lib/sensor/veml7700.c"
)

include_directories(src ../lib ../lib/inc ../../../ARM_CLION/common_lib ../../../ARM_CLION/common_lib/sensor ../../../ARM_CLION/common_lib/display)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")
