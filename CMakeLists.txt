cmake_minimum_required(VERSION 3.30)
project(mos6502 LANGUAGES C)

# We require C11 for the testing harness.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

include(cmake/Warnings.cmake)

add_library(mos6502 STATIC mos6502.c)
target_compile_options(mos6502 PRIVATE ${MOS6502_WARNING_FLAGS_C})

enable_testing()
add_subdirectory(tests)
