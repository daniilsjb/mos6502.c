add_executable(mos6502-test main.c harness.c suite_programs.c suite_opcodes.c)

add_test(NAME "Emulator Tests" COMMAND mos6502-test)
target_link_libraries(mos6502-test PRIVATE mos6502)
target_compile_options(mos6502-test PRIVATE ${MOS6502_WARNING_FLAGS_C})

add_custom_command(TARGET mos6502-test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/resources
        ${CMAKE_CURRENT_BINARY_DIR}/resources
)
