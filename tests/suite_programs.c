#include "harness.h"
#include "suite_programs.h"
#include "../mos6502.h"

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>

static void on_cpu_write(uint16_t address, uint8_t data, void *memory) {
    ((uint8_t *)memory)[address] = data;
}

static uint8_t on_cpu_read(uint16_t address, void *memory) {
    return ((uint8_t *)memory)[address];
}

static mos6502_t create_cpu(uint8_t *memory) {
    return mos6502_create((mos6502_desc_t) {
        .write = on_cpu_write,
        .read  = on_cpu_read,
        .bus   = memory,
    });
}

static void load(const char *path, uint8_t *memory, size_t offset) {
    FILE *file = fopen(path, "rb");
    if (!file) {
        fprintf(stderr, "Could not open '%s'\n", path);
        exit(EXIT_FAILURE);
    }

    if (!fread(&memory[offset], sizeof(uint8_t), 0x10000 - offset, file)) {
        fprintf(stderr, "Could not read data from '%s'\n", path);
        exit(EXIT_FAILURE);
    }

    fclose(file);
}

/*
 * This test was obtained from the following thread on Reddit:
 *   https://www.reddit.com/r/EmuDev/comments/9s755i/comment/e8qi80f
 *
 * Source: https://github.com/koute/pinky/tree/master/mos6502/roms/sources
 * Binary: https://github.com/koute/pinky/tree/master/mos6502/roms
 */
static void run_ttl6502(void) {
    const char *test_path = "resources/ttl-6502/TTL6502.bin";
    test_begin(test_path);

    uint8_t memory[0x10000] = { 0x00 };
    load(test_path, memory, 0xE000);

    mos6502_t cpu = create_cpu(memory);
    mos6502_reset(&cpu);

    while (cpu.pc != 0xF5B6) {
        if (mos6502_step(&cpu) == MOS6502_LOOP) {
            test_fail("Stuck in an infinite loop at $%04" PRIx16, cpu.pc);
            return;
        }
    }

    test_pass();
}

/*
 * Tests obtained from the hmc-6502 project by cminter.
 *
 * Source: https://github.com/cminter/hmc-6502
 * Binary: https://codegolf.stackexchange.com/questions/12844/emulate-a-mos-6502-cpu
 */
static void run_all_suite_a(void) {
    const char *test_path = "resources/all-suite-a/AllSuiteA.bin";
    test_begin(test_path);

    uint8_t memory[0x10000] = { 0x00 };
    load(test_path, memory, 0x4000);

    mos6502_t cpu = create_cpu(memory);
    mos6502_reset(&cpu);

    while (cpu.pc != 0x45C0) {
        mos6502_step(&cpu);
    }

    uint8_t code = memory[0x0210];
    if (code != 0xFF) {
        test_fail("Expected $ff at $0210, but got $%02 instead" PRIx8, code);
        return;
    }

    test_pass();
}

/*
 * Comprehensive test suite for the 6502 processor by Klaus Dormann.
 *
 * Source: https://github.com/amb5l/6502_65C02_functional_tests/tree/master/ca65
 * Binary: https://github.com/amb5l/6502_65C02_functional_tests/tree/master/bin_files
 */
static void run_6502_functional_test(void) {
    const char *test_path = "resources/6502-functional-test/6502_functional_test.bin";
    test_begin(test_path);

    uint8_t memory[0x10000] = { 0x00 };
    load(test_path, memory, 0x0000);

    mos6502_t cpu = create_cpu(memory);
    cpu.pc = 0x0400;

    while (mos6502_step(&cpu) != MOS6502_LOOP) {
        /* Keep executing... */
    }

    if (cpu.pc != 0x3469) {
        /* For details, see the trap descriptions in the test's assembly listing. */
        test_fail("Stuck in an infinite loop at $%04" PRIx16, cpu.pc);
        return;
    }

    test_pass();
}

/*
 * Test of all possible BCD encodings, originally written by Bruce Clark.
 *
 * Source: https://github.com/amb5l/6502_65C02_functional_tests/tree/master/ca65
 * Binary: https://github.com/amb5l/6502_65C02_functional_tests/tree/master/bin_files
 */
static void run_6502_decimal_test(void) {
    const char *test_path = "resources/6502-decimal-test/6502_decimal_test.bin";
    test_begin(test_path);

    uint8_t memory[0x10000] = { 0x00 };
    load(test_path, memory, 0x0000);

    mos6502_t cpu = create_cpu(memory);
    cpu.pc = 0x0200;

    while (cpu.pc != 0x024B) {
        mos6502_step(&cpu);
    }

    uint8_t code = memory[0x000B];
    if (code != 0x00) {
        test_fail("Expected $00 at $000B, but got $%02" PRIx8, code);
        return;
    }

    test_pass();
}

void run_suite_programs(void) {
    suite_begin("Programs");

    run_ttl6502();
    run_all_suite_a();
    run_6502_functional_test();
    run_6502_decimal_test();

    suite_end();
}
