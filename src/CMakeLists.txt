
set(SRC   cyfx_gcc_startup.S  cyfxtx.c  cyfxusbenumdscr.c  cyfxusbi2cregmode.c  cyfxusbi2cregmode.h)

set_source_files_properties(cyfx_gcc_startup.S PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

include_directories("${FX3_PATH}/boot_lib/1_3_3/include")
include_directories("${FX3_PATH}/fw_lib/1_3_3/inc")

link_directories(${FX3_PATH}/boot_lib/1_3_3/lib)
link_directories(${FX3_PATH}/fw_lib/1_3_3/fx3_debug)
add_definitions(-D__CYU3P_TX__=1)

add_executable(camera ${SRC})
TARGET_LINK_LIBRARIES(camera cyu3lpp cyfxapi cyu3threadx c gcc)
