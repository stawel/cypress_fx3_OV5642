set(FX3_PATH                    $ENV{FX3_INSTALL_PATH})
set(ARMGCC_INSTALL_PATH         $ENV{ARMGCC_INSTALL_PATH})

#please see for ubuntu: https://launchpad.net/~terry.guo/+archive/ubuntu/gcc-arm-embedded
#set(TOOLCHAIN arm-none-eabi)

set(TOOLCHAIN ${ARMGCC_INSTALL_PATH}/bin/arm-none-eabi)


INCLUDE(CMakeForceCompiler)
SET(CMAKE_SYSTEM_NAME Generic)


SET(CMAKE_C_COMPILER ${TOOLCHAIN}-gcc)
SET(CMAKE_CXX_COMPILER ${TOOLCHAIN}-g++)

CMAKE_FORCE_C_COMPILER(${CMAKE_C_COMPILER} GNU)
CMAKE_FORCE_CXX_COMPILER(${CMAKE_CXX_COMPILER} GNU)
