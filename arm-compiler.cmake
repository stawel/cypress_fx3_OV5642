
#set(TOOLCHAIN arm-elf)
#please see for ubuntu: https://launchpad.net/~terry.guo/+archive/ubuntu/gcc-arm-embedded
set(TOOLCHAIN arm-none-eabi)
#set(TOOLCHAIN arm-linux-gnueabi)

SET(CMAKE_C_COMPILER ${TOOLCHAIN}-gcc)
SET(CMAKE_CXX_COMPILER ${TOOLCHAIN}-g++)

#cmake compiler test bypass
INCLUDE(CMakeForceCompiler)
CMAKE_FORCE_C_COMPILER(${CMAKE_C_COMPILER} GNU)
CMAKE_FORCE_CXX_COMPILER(${CMAKE_CXX_COMPILER} GNU)

set(FX3_PATH $ENV{FX3_INSTALL_PATH})

#eclipse Assembler:
#arm-none-eabi-gcc -mcpu=arm926ej-s -marm -mthumb-interwork -O0 -fmessage-length=0 
# -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 
# -x assembler-with-cpp -MMD -MP -MF"cyfx_gcc_startup.d" -MT"cyfx_gcc_startup.o" -c -o "cyfx_gcc_startup.o" "../cyfx_gcc_startup.S"

#eclipse C:
#arm-none-eabi-gcc -mcpu=arm926ej-s -marm -mthumb-interwork -O0 -fmessage-length=0 
# -fsigned-char -ffunction-sections -fdata-sections -Wall  
# -g3 -I"/home/stawel/Cypress/cyfx3sdk/boot_lib/1_3_3/include" -std=gnu11 -MMD -MP -MF"main.d" -MT"main.o" -c -o "main.o" "../main.c"

include_directories("${FX3_PATH}/boot_lib/1_3_3/include")

SET(CTUNING "-fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections")
SET(CPU_FLAGS "-mcpu=arm926ej-s -marm -mthumb-interwork")
SET(OTHER_FLAGS "-O0 -Wall -g3")

SET(CFLAGS "${CPU_FLAGS}  ${CTUNING} ${OTHER_FLAGS} -std=c11")
SET(CXXFLAGS "${CPU_FLAGS}  ${CTUNING} ${OTHER_FLAGS} -fno-rtti -fno-exceptions -std=c++11")
SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${CFLAGS}   -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXXFLAGS} -ffunction-sections -fdata-sections")

#eclipse Linker:
#arm-none-eabi-gcc -mcpu=arm926ej-s -marm -mthumb-interwork -O0 
# -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections 
# -Wall  -g3 -T "/home/stawel/Cypress/cyfx3sdk/fw_build/boot_fw/cyfx3.ld" 
# -nostartfiles -Xlinker --gc-sections 
# -L"/home/stawel/Cypress/cyfx3sdk/boot_lib/1_3_3/lib" 
# -Wl,-Map,"BootLedBlink.map" -Wl,-d -Wl,-elf -Wl,--no-wchar-size-warning 
# -Wl,--entry,Reset_Handler -o "BootLedBlink.elf"  ./cyfx_gcc_startup.o ./main.o   -lcyfx3boot -lc -lgcc

link_directories(${FX3_PATH}/boot_lib/1_3_3/lib)
set(LINKER_FLAGS "-T ${FX3_PATH}/fw_build/boot_fw/cyfx3.ld -nostartfiles -Xlinker --gc-sections  -Wl,-d -Wl,-elf -Wl,--no-wchar-size-warning  -Wl,--entry,Reset_Handler")
set(CMAKE_EXE_LINKER_FLAGS "${LINKER_FLAGS}")
