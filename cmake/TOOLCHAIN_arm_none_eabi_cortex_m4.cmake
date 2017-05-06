# CMAKE toolchain for the gcc arm-none-eabi
#
set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_VERSION   1)
set(CMAKE_SYSTEM_PROCESSOR arm-none-eabi)

# set arm-none-eabi toolchain paths
set(TOOLCHAIN_DIR C:/opt/gcc-arm-none-eabi-4_9-2015q3-20150921-win32) #/opt/arm-gcc/gcc-arm-none-eabi-4_8-2014q1 )
set(TOOL_CHAIN_PREFIX arm-none-eabi)
set(TOOLCHAIN_BIN_DIR ${TOOLCHAIN_DIR}/bin)
set(TOOLCHAIN_LIB_DIR ${TOOLCHAIN_DIR}/lib)

if(WIN32)
set(EXE .exe)
endif(WIN32)

# which compilers to use for C and C++
#
set(CMAKE_C_COMPILER       ${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX}-gcc${EXE})
set(CMAKE_CXX_COMPILER     ${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX}-g++${EXE})
set(CMAKE_ASM_COMPILER     ${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX}-as${EXE})
set(CMAKE_OBJCOPY     	   ${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX}-objcopy${EXE} CACHE INTERNAL "objcopy command")
set(CMAKE_OBJDUMP     	   ${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX}-objdump${EXE} CACHE INTERNAL "objdump command")
set(CMAKE_GDB              ${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX}-gdb${EXE})

set(COMPILER_OPTIONS "-mthumb -mcpu=cortex-m4 -mlittle-endian -mthumb-interwork ")

SET(CMAKE_C_FLAGS "${COMPILER_OPTIONS} -lm -lc -lnosys --specs=nosys.specs " CACHE INTERNAL "c compiler flags")
SET(CMAKE_CXX_FLAGS "${COMPILER_OPTIONS} -lm -lc -lnosys --specs=nosys.specs " CACHE INTERNAL "cxx compiler flags")
SET(CMAKE_ASM_FLAGS "${COMPILER_OPTIONS}" CACHE INTERNAL "asm compiler flags")

SET(CMAKE_EXE_LINKER_FLAGS "${COMPILER_OPTIONS} -Wl,-Map=linker.map -Wl,-cref -Wl,--gc-sections" CACHE INTERNAL "exe link flags")

# adjust the default behaviour of the FIND_XXX() commands:
# search headers and libraries in the target environment,
# search programs in the host environment
#
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

SET(CMAKE_ASM_COMPILE_OBJECT "<CMAKE_ASM_COMPILER> <FLAGS> -o <OBJECT> <SOURCE>")
