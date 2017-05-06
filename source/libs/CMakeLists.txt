cmake_minimum_required(VERSION 2.8)

project(libs)

add_subdirectory(CMSIS)

# Standard peripheral driver for stm32f4xxx
add_subdirectory(STM32F4xx_StdPeriph_Driver)

# project start-up file
add_subdirectory(startup)

