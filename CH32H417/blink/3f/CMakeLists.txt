SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.14)

set(CMAKE_C_STANDARD 23)

# specify the cross compiler
SET(CMAKE_C_COMPILER_WORKS 1)

SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(LINKER_SCRIPT ../../../Link_v3f.ld)

SET(COMMON_FLAGS "-march=rv32imafcbxw -mabi=ilp32f -msmall-data-limit=8 -msave-restore -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -specs=nosys.specs -specs=nano.specs -nostartfiles")

SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")
add_definitions(-DCore_V3F)

project(blink_3f C ASM)

file(GLOB SOURCES
        "src/main.c"
        "../../lib/delay.c"
        "../../lib/system_ch32vh417.c"
        "../../lib/src/ch32h417_rcc.c"
        "../../lib/src/ch32h417_gpio.c"
        "../../lib/src/ch32h417_flash.c"
        "../../startup_ch32h417_v3f.S"
        "../../lib/system_ch32h417.c"
)

include_directories(src ../../lib ../../lib/inc)

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
