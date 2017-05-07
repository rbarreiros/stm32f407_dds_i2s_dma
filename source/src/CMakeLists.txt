cmake_minimum_required(VERSION 2.8)

project(dds_i2s_dma)

set(PP_DEFINES " -DSTM32 -DSTM32F4 -DSTM32F407ZETx -DSTM32F40_41xxx -DSTM32F40XX -DUSE_STDPERIPH_DRIVER")


include_directories(
    ${libs_SOURCE_DIR}/CMSIS/device
    ${libs_SOURCE_DIR}/CMSIS/Include
    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/inc
    inc
)

# uncomment/ comment as required
file(GLOB STM_STD_PERIPH_LIB_SRC
    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/misc.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_can.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cec.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_crc.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_aes.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_des.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_tdes.c
#     ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dac.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dbgmcu.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dfsdm.c
    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma2d.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c
    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flas_ramfunc.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fmc.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fmpi2c.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fsmc.c
    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_md5.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_sha1.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_iwdg.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_lptim.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_ltdc.c
    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_qspi.c
    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rng.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rtc.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sai.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sdio.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spdifrx.c
    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c
    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c
    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c
#    ${libs_SOURCE_DIR}/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_wwdg.c
)


file(GLOB C_SOURCE
	dds.c
	debug_uart.c
    hw_config.c
	i2s_dma.c
    main.c
    stm32f4xx_it.c
    syscalls.c
    system_stm32f4xx.c
	
    ${STM_STD_PERIPH_LIB_SRC}
)

set_source_files_properties(${C_SOURCE}
    PROPERTIES COMPILE_FLAGS ${PP_DEFINES}
)

add_executable(${PROJECT_NAME}.elf
    ${STARTUP_ASM_FILE}
    ${C_SOURCE}
)

add_custom_target ( ${PROJECT_NAME}.hex ALL
    DEPENDS ${PROJECT_NAME}.elf 
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Generating ${PROJECT_NAME}.hex"
)
add_custom_target ( ${PROJECT_NAME}.bin ALL
    DEPENDS ${PROJECT_NAME}.elf 
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMENT "Generating ${PROJECT_NAME}.bin"
)

# debug target
add_custom_target ( debug-${PROJECT_NAME}
    DEPENDS ${PROJECT_NAME}.elf 
    COMMAND ${CMAKE_GDB} ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Starting gdb for ${PROJECT_NAME}"
)
