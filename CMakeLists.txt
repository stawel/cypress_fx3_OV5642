cmake_minimum_required(VERSION 2.8.11)

SET(CMAKE_SYSTEM_NAME Generic)

include(arm-compiler.cmake)


Project(cypress_fx3_camera C ASM)


add_subdirectory(src)
