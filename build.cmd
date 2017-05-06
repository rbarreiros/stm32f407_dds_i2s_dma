@echo off
rmdir /s /q build-stm32
cd source
cmake -G"Eclipse CDT4 - MinGW Makefiles" -D_ECLIPSE_VERSION=4.5 -H. -B"..\build-stm32" -DCMAKE_TOOLCHAIN_FILE=..\cmake\TOOLCHAIN_arm_none_eabi_cortex_m4.cmake
cd ..
make -C build-stm32
@echo on