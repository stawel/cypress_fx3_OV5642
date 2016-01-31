
set(SRC   defines.h  main.c cyfx_gcc_startup.S)

set_source_files_properties(cyfx_gcc_startup.S PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

add_executable(led ${SRC})
TARGET_LINK_LIBRARIES(led cyfx3boot c gcc)
