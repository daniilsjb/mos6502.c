#include "harness.h"
#include "suite_programs.h"
#include "suite_opcodes.h"

int main(void) {
    harness_begin();

    run_suite_programs();
    run_suite_opcodes_mos6502();
    run_suite_opcodes_nes6502();

    harness_end();
    return harness_status();
}
