include_guard(GLOBAL)

set(MOS6502_WARNING_FLAGS_C)
set(MOS6502_WARNING_FLAGS_CXX)

if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID MATCHES ".*Clang")
    list(APPEND MOS6502_WARNING_FLAGS_C
            -Wall
            -Wextra
            -Werror
            -Wpedantic
            -Wcast-align
            -Wcast-qual
            -Wconversion
            -Wformat
            -Wsign-conversion
            -Wshadow
            -Wswitch-enum
            -Wundef
            -Wunused
            -Wdouble-promotion
            -Wstrict-aliasing
            -Wuninitialized
            -Wimplicit-fallthrough
            -Wnull-dereference
    )

    if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
        list(APPEND MOS6502_WARNING_FLAGS_C
                -Wmisleading-indentation
                -Wduplicated-cond
                -Wduplicated-branches
                -Wlogical-op
        )
    endif()

    list(APPEND MOS6502_WARNING_FLAGS_CXX
            ${MOS6502_WARNING_FLAGS_C}
            -Wnon-virtual-dtor
            -Wold-style-cast
            -Woverloaded-virtual
            -Wno-missing-field-initializers
    )
elseif(CMAKE_C_COMPILER_ID STREQUAL "MSVC")
    list(APPEND MOS6502_WARNING_FLAGS_C
            /W4          # Baseline warnings.
            /WX          # Treat warnings as errors.
            /w14242      # 'identifier': conversion from 'type1' to 'type2', possible loss of data.
            /w14287      # 'operator': unsigned/negative constant mismatch.
            /w14296      # 'operator': expression is always false.
            /w14311      # 'variable': pointer truncation from 'type' to 'type'.
            /w14826      # Conversion from 'type1' to 'type2' is sign-extended.
            /w14062      # Enumerator 'identifier' in a switch of enum 'enumeration' is not handled.
            /w14254      # A larger bit field was assigned to a smaller bit field, possible loss of data.
            /w14289      # Loop control variable is used outside the for-loop scope.
            /w14545      # Expression before comma evaluates to a function which is missing an argument list.
            /w14546      # Function call before comma missing argument list.
            /w14547      # Operator before comma has no effect; expected operator with side-effect.
            /w14549      # Operator before comma has no effect; did you intend 'operator2'?
            /w14555      # Expression has no effect; expected expression with side-effect.
            /w14619      # #pragma warning: there is no warning number 'number'.
            /permissive- # Use standards conformance mode.
            /D_CRT_SECURE_NO_WARNINGS
    )

    list(APPEND MOS6502_WARNING_FLAGS_CXX
            ${MOS6502_WARNING_FLAGS_C}
            /w14263      # Member function does not override any base class virtual member function.
            /w14265      # 'class': class has virtual functions, but destructor is not virtual.
            /w14640      # 'instance': construction of local static object is not thread-safe.
            /w14928      # Illegal copy-initialization; applied more than one user-defined conversion.
    )
else()
    message(AUTHOR_WARNING "No warning flags could be set for the '${CMAKE_C_COMPILER_ID}' compiler.")
endif()
