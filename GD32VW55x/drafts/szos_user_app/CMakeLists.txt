SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.14)

set(CMAKE_C_STANDARD 23)

# specify the cross compiler
SET(CMAKE_C_COMPILER_WORKS 1)

SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(LINKER_SCRIPT ../../szos_gd23vw553/syscalls/ldscript2.ld)

SET (FLASH_SIZE 1024)
SET (RAM_SIZE 1024)

SET(COMMON_FLAGS "-mno-relax -mno-explicit-relocs -march=rv32imafdc_zba_zbb_zbc_zbs_xxldspn1x -mtune=nuclei-300-series -mcmodel=medlow -msmall-data-limit=8 -mdiv -mabi=ilp32d -msave-restore -fmessage-length=0 -fsigned-char -fno-common -ffunction-sections -fdata-sections -Wunused -Wuninitialized -specs=nosys.specs -specs=nano.specs -nostartfiles")
add_definitions(-DFLASH_SIZE=${FLASH_SIZE} -DRAM_SIZE=${RAM_SIZE})

SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage,--defsym=FLASH_SIZE=${FLASH_SIZE},--defsym=RAM_SIZE=${RAM_SIZE} -T ${LINKER_SCRIPT}")

project(szos_user_app C ASM)

file(GLOB SOURCES
        "src/main.c"
)

include_directories(src ../szos_gd23vw553/syscalls)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

add_library(syscalls SHARED IMPORTED)
set_target_properties(syscalls PROPERTIES
        IMPORTED_LOCATION "../../szos_gd23vw553/syscalls/syscalls.so"
)

target_link_libraries(${PROJECT_NAME}.elf PRIVATE syscalls)

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")
