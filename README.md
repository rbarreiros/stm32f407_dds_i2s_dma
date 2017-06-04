Sine generator using the STM32f4xxx I2S and PCM5102A
---

### Description

This project is a 2-channel (Left/Right stereo) sine signal generator using an STM32F407ZET6 board. The I2S interface on the STM is running in DMA mode with double buffering and FIFO enabled. The sample rate is 48KHz (16/48) and for the sine generation the phase accumulator method is used. You can control the sine frequency using the stm's UART1 port. The board I've used is the following one and you can find on e-bay for about $15.

<img src="http://www.stupid-projects.com/wp-content/uploads/2017/05/stm32f407vet6_.jpg" width="200"/>

FYI, there are two quite similar dev boards for stm32f407. The first one is the STM32F407ZET6 board and the other one is STM32F407VET6 board. Their difference is that the ZET6 micro-processor is an LQFP144 package and has 114 gpios and the VET6 is LQFP100 package with 82 GPIOs. Both have 512KB flash and 192KB ram and have exactly the same peripherals on exactly the same ports and pins; therefore the code should be compatible for both boards.

This code is part from a web article, so look for more info in this link:

http://www.stupid-projects.com/sine-generator-using-stmf407-internal-dac-and-pcm5102a/

### How to compile and flash
You need cmake to build this project either on Windows or Linux. To setup the cmake properly follow the instructions from [here](https://github.com/dimtass/cmake_toolchains/blob/master/README.md). Then edit the cmake/TOOLCHAIN_arm_none_eabi_cortex_m3.cmake file and point TOOLCHAIN_DIR to the correct GCC path.
> e.g. on Windows
> set(TOOLCHAIN_DIR C:/opt/gcc-arm-none-eabi-4_9-2015q3-20150921-win32)
> or on Linux
> set(TOOLCHAIN_DIR /opt/gcc-arm-none-eabi-4_9-2015q3)

Then on Windows run ```build.cmd``` or on Linux run ```./build.sh``` and the .bin and .hex files should be created in the ```build-stm32/src``` folder. Also, a .cproject and .project files are created if you want to edit the source code.

To flash the HEX file in windows use st-link utility like this:
```"C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe" -c SWD -p build-stm32\src\stm32f103_wifi_usb_psu.hex -Rst```

If instead the st-link you have J-Link then flash the binary/hex with J-Flash Lite gui tool.

To flash the bin in Linux:
```st-flash --reset write build-stm32/src/stm32f103_wifi_usb_psu.bin 0x8000000```

### Control commands
To change the sine frequency for each channel then connect a USB to UART converter on UART1 using the following connection:
RS232 TX -> PA9
RS232 RX -> PA10
RS232 GND -> STM32 dev board GND

Then open an rs-232 terminal (e.g. Bray's Terminal) with the following settings, 115200,8,N,1 and check 'CR=LF' and '+CR' and use the following commands:

```FREQ=<CH>,<FREQUENCY>```
    where CH is L or R
    This sets the desired frequncy in Hz on the selected channel.

E.g.:
```FREQ=L,5000```
    Sets the left I2S channel output frequncy to 5 KHz.

```FREQ=R,18459.23```
    Sets the right I2S sine output frequency to 18.45923 KHz.