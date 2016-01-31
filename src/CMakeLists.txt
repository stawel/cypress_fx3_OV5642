
set(SRC   cyfx_gcc_startup.S  cyfxtx.c  cyfxusbenumdscr.c  cyfxusbi2cregmode.c  cyfxusbi2cregmode.h)

set_source_files_properties(cyfx_gcc_startup.S PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

add_executable(camera ${SRC})
TARGET_LINK_LIBRARIES(camera c gcc)
