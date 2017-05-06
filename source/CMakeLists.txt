cmake_minimum_required(VERSION 2.8)

project(dds_dac)

set(COMPILER_OPTIONS "-Wall -Werror")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${COMPILER_OPTIONS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMPILER_OPTIONS} -std=c++11")

# shared libraries
add_subdirectory(libs)

# force stm builds to debug.  This is a hack as when release is set, it automatically 
# sets the invalid -O3 flag on the assembler.
set(CMAKE_BUILD_TYPE Debug)

# enable asm for stm startup.s file
enable_language(ASM)

# override toolchain flags
set(COMPILER_OPTIONS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 ")

# -fno-builtin -fno-strict-aliasing -std=c99 
SET(CMAKE_C_FLAGS "${COMPILER_OPTIONS} -fmessage-length=0 -ffunction-sections " CACHE INTERNAL "c compiler flags")
SET(CMAKE_CXX_FLAGS "${COMPILER_OPTIONS} -fmessage-length=0 -ffunction-sections " CACHE INTERNAL "cxx compiler flags")
SET(CMAKE_ASM_FLAGS "${COMPILER_OPTIONS}" CACHE INTERNAL "asm compiler flags")
# -mthumb -mcpu=cortex-m3 -mfix-cortex-m3-ldrd   -Wl,-Map=linker.map -Wl,-cref  -Wl,--gc-sections
SET(CMAKE_EXE_LINKER_FLAGS "${COMPILER_OPTIONS} -Wl,-Map=output.map -Wl,--gc-sections -fno-exceptions -fno-rtti -lm " CACHE INTERNAL "exe link flags")

# set compiler optimisations
set(COMPILER_OPTIMISATION "-g3 -O2")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${COMPILER_OPTIMISATION}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMPILER_OPTIMISATION}")

# add startup and linker file
set(STARTUP_ASM_FILE "${libs_SOURCE_DIR}/startup/startup_stm32.s")
set_property(SOURCE ${STARTUP_ASM_FILE} PROPERTY LANGUAGE ASM)
set(LINKER_FILE "${CMAKE_CURRENT_SOURCE_DIR}/LinkerScript.ld")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${COMPILER_OPTIONS} -T${LINKER_FILE}")

# add the source code directory
add_subdirectory(src)

message(STATUS "System Processor      : ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS 
    "BUILD FLAGS:\n"
    "   c flags     = ${CMAKE_C_FLAGS}\n"
    "   c++ flags   = ${CMAKE_CXX_FLAGS}\n"
    "   asm flags   = ${CMAKE_ASM_FLAGS}\n"
    "   ld flags    = ${CMAKE_EXE_LINKER_FLAGS}\n"
    "   linker file = ${LINKER_FILE}\n"
    "   startup file= ${STARTUP_ASM_FILE}\n"
    "   library path= ${CMAKE_LIBRARY_PATH}\n"
    "   prefix path = ${CMAKE_PREFIX_PATH}\n"
)
