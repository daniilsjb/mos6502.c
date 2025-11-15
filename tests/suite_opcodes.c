/*
 * This suite is based on Tom Harte's processor tests, available at:
 *   https://github.com/SingleStepTests/65x02
 *
 * The repository contains 10,000 randomly-generated tests for each opcode of
 * the 6502 CPU. Each test is described by a set of initial register and memory
 * values, followed by a cycle-by-cycle breakdown of opcode execution verified
 * using other emulators. Thus, unlike other programs we're using for testing,
 * this suite also verifies cycle accuracy.
 *
 * Note that the original tests are encoded in JSON. While it is certainly a
 * convenient format for higher-level languages, parsing it in C would be rather
 * awkward. Besides, the original files end up occupying quite a bit of space
 * due to all the extra characters required by JSON syntax. To alleviate both
 * issues, I converted each test into an equivalent binary file while omitting
 * some fields that we don't really need (e.g., test names).
 *
 * Furthermore, I excluded the expected RAM contents from each test and ensured
 * that the initial memory contents only contain addresses that the CPU actually
 * interacts with. Consequently, the correctness of memory reads and writes is
 * verified exclusively through the bus activity of the opcode.
 *
 * The resulting file format can be inferred from the code below, but in summary,
 * each test is structured as follows (assuming little-endian order and ASCII
 * character encodings):
 *
 *   (1) Test Count: u16
 *
 *   (2) CPU Initial:             (3) CPU Final:
 *
 *     u16 u8 u8 u8 u8 u8         u16 u8 u8 u8 u8 u8
 *      ^  ^  ^  ^  ^  ^           ^  ^  ^  ^  ^  ^
 *     PC  A  X  Y  S  P          PC  A  X  Y  S  P
 *
 *   (4) RAM Initial:
 *
 *     u8    | u16      u8                                   |
 *     ^     | ^        ^                                    |
 *     Size  | Address  Value  ...repeated 'Size' times      |
 *
 *   (5) Cycles:
 *
 *     u8    | u16      u8     char                          |
 *     ^     | ^        ^      ^                             |
 *     Size  | Address  Value  r|w  ...repeated 'Size' times |
 *
 * where:
 *   (1) - The number of tests contained within the file.
 *   (2) - The register values that the CPU starts the test with.
 *   (3) - The register values that the CPU should end the test with.
 *   (4) - A set of address/value pairs comprising initial RAM contents.
 *   (5) - An ordered sequence of cycles comprising the test's bus activity.
 *
 * To generate new test files, use the `scripts/convert.py` tool:
 *   $ py convert.py <path-to-json>                  # Keep all tests
 *   $ py convert.py <path-to-json> --keep <number>  # Keep only a subset of tests
 */

#include "harness.h"
#include "suite_opcodes.h"
#include "../mos6502.h"

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>

#define MAX_CYCLES 8
#define MAX_MEMORY 16

typedef struct {
    uint16_t addr;
    uint8_t data;
} opc_memory_t;

typedef struct {
    uint16_t addr;
    uint8_t data;
    uint8_t type;
} opc_cycle_t;

typedef struct {
    uint16_t pc;
    uint8_t a;
    uint8_t x;
    uint8_t y;
    uint8_t s;
    uint8_t p;
} opc_regs_t;

typedef struct {
    opc_regs_t regs_starting;
    opc_regs_t regs_expected;

    uint8_t num_memory;
    uint8_t num_cycles;

    opc_memory_t memory[MAX_MEMORY];
    opc_cycle_t cycles[MAX_CYCLES];
} opc_test_t;

typedef struct {
    mos6502_t cpu;

    uint8_t num_memory;
    uint8_t num_cycles;

    opc_memory_t memory[MAX_MEMORY];
    opc_cycle_t cycles[MAX_CYCLES];
} opc_state_t;

static uint8_t read_u8(uint8_t **cursor) {
    return *(*cursor)++;
}

static uint16_t read_u16(uint8_t **cursor) {
    uint8_t lo = read_u8(cursor);
    uint8_t hi = read_u8(cursor);
    return (uint16_t)((hi << 8) | lo);
}

static opc_regs_t read_regs(uint8_t **cursor) {
    opc_regs_t regs;
    regs.pc = read_u16(cursor);
    regs.a = read_u8(cursor);
    regs.x = read_u8(cursor);
    regs.y = read_u8(cursor);
    regs.s = read_u8(cursor);
    regs.p = read_u8(cursor);
    return regs;
}

static opc_memory_t read_memory(uint8_t **cursor) {
    opc_memory_t memory;
    memory.addr = read_u16(cursor);
    memory.data = read_u8(cursor);
    return memory;
}

static opc_cycle_t read_cycle(uint8_t **cursor) {
    opc_cycle_t cycle;
    cycle.addr = read_u16(cursor);
    cycle.data = read_u8(cursor);
    cycle.type = read_u8(cursor);
    return cycle;
}

static opc_test_t read_test(uint8_t **cursor) {
    opc_test_t test;

    test.regs_starting = read_regs(cursor);
    test.regs_expected = read_regs(cursor);

    test.num_memory = read_u8(cursor);
    for (uint8_t i = 0; i < test.num_memory; ++i) {
        test.memory[i] = read_memory(cursor);
    }

    test.num_cycles = read_u8(cursor);
    for (uint8_t i = 0; i < test.num_cycles; ++i) {
        test.cycles[i] = read_cycle(cursor);
    }

    return test;
}

static void on_cpu_write(uint16_t address, uint8_t value, void *bus) {
    opc_state_t *state = bus;

    state->cycles[state->num_cycles++] = (opc_cycle_t) {
        .addr = address,
        .data = value,
        .type = 'w'
    };

    for (uint8_t i = 0; i < state->num_memory; ++i) {
        if (state->memory[i].addr == address) {
            state->memory[i].data = value;
            break;
        }
    }

    state->memory[state->num_memory++] = (opc_memory_t) {
        .addr = address,
        .data = value,
    };
}

static uint8_t on_cpu_read(uint16_t address, void *bus) {
    opc_state_t *state = bus;

    uint8_t value = 0x00;
    for (uint8_t i = 0; i < state->num_memory; ++i) {
        if (state->memory[i].addr == address) {
            value = state->memory[i].data;
            break;
        }
    }

    state->cycles[state->num_cycles++] = (opc_cycle_t) {
        .addr = address,
        .data = value,
        .type = 'r'
    };

    return value;
}

static bool run_test(opc_test_t *test, bool bcd_disabled) {
    opc_state_t state = {
        .cpu = mos6502_create((mos6502_desc_t) {
            .bcd_disabled = bcd_disabled,
            .write = on_cpu_write,
            .read  = on_cpu_read,
            .bus   = &state,
        }),
    };

    state.cpu.pc = test->regs_starting.pc;
    state.cpu.a = test->regs_starting.a;
    state.cpu.x = test->regs_starting.x;
    state.cpu.y = test->regs_starting.y;
    state.cpu.s = test->regs_starting.s;
    state.cpu.p = test->regs_starting.p;

    for (uint8_t i = 0; i < test->num_memory; ++i) {
        state.memory[state.num_memory++] = (opc_memory_t) {
            .addr = test->memory[i].addr,
            .data = test->memory[i].data,
        };
    }

    /* Each test consists of only a single instruction. */
    mos6502_step(&state.cpu);

    if (state.cpu.pc != test->regs_expected.pc) return false;
    if (state.cpu.a != test->regs_expected.a) return false;
    if (state.cpu.x != test->regs_expected.x) return false;
    if (state.cpu.y != test->regs_expected.y) return false;
    if (state.cpu.s != test->regs_expected.s) return false;
    if (state.cpu.p != test->regs_expected.p) return false;

    if (state.num_cycles != test->num_cycles) return false;
    for (uint8_t i = 0; i < test->num_cycles; ++i) {
        if (state.cycles[i].addr != test->cycles[i].addr) return false;
        if (state.cycles[i].data != test->cycles[i].data) return false;
        if (state.cycles[i].type != test->cycles[i].type) return false;
    }

    return true;
}

static void run_tests(const char *suite, int opcode, bool bcd_disabled) {
     char test_path[32] = {0};
     sprintf(test_path, "resources/%s/%02x.bin", suite, opcode);

     FILE *file = fopen(test_path, "rb");
     if (!file) return;

     fseek(file, 0, SEEK_END);
     size_t size = (size_t)ftell(file);
     rewind(file);

     uint8_t *test_buffer = malloc(size);
     uint8_t *test_cursor = test_buffer;

     if (fread(test_buffer, sizeof(uint8_t), size, file) != size) {
        fprintf(stderr, "Could not read data from '%s'\n", test_path);
        exit(EXIT_FAILURE);
     }

     fclose(file);

     test_begin(test_path);

     uint16_t test_count = read_u16(&test_cursor);
     for (uint16_t i = 0; i < test_count; ++i) {
         opc_test_t test = read_test(&test_cursor);

         if (!run_test(&test, bcd_disabled)) {
             test_fail("State mismatch on step %" PRId16, i);
             goto done;
         }
     }

     test_pass();

done:
     free(test_buffer);
}

static const bool supported[0x100] = {
    [0x00] = 1, [0x01] = 1, [0x02] = 0, [0x03] = 0, [0x04] = 0, [0x05] = 1, [0x06] = 1, [0x07] = 0,
    [0x08] = 1, [0x09] = 1, [0x0A] = 1, [0x0B] = 0, [0x0C] = 0, [0x0D] = 1, [0x0E] = 1, [0x0F] = 0,
    [0x10] = 1, [0x11] = 1, [0x12] = 0, [0x13] = 0, [0x14] = 0, [0x15] = 1, [0x16] = 1, [0x17] = 0,
    [0x18] = 1, [0x19] = 1, [0x1A] = 0, [0x1B] = 0, [0x1C] = 0, [0x1D] = 1, [0x1E] = 1, [0x1F] = 0,
    [0x20] = 1, [0x21] = 1, [0x22] = 0, [0x23] = 0, [0x24] = 1, [0x25] = 1, [0x26] = 1, [0x27] = 0,
    [0x28] = 1, [0x29] = 1, [0x2A] = 1, [0x2B] = 0, [0x2C] = 1, [0x2D] = 1, [0x2E] = 1, [0x2F] = 0,
    [0x30] = 1, [0x31] = 1, [0x32] = 0, [0x33] = 0, [0x34] = 0, [0x35] = 1, [0x36] = 1, [0x37] = 0,
    [0x38] = 1, [0x39] = 1, [0x3A] = 0, [0x3B] = 0, [0x3C] = 0, [0x3D] = 1, [0x3E] = 1, [0x3F] = 0,
    [0x40] = 1, [0x41] = 1, [0x42] = 0, [0x43] = 0, [0x44] = 0, [0x45] = 1, [0x46] = 1, [0x47] = 0,
    [0x48] = 1, [0x49] = 1, [0x4A] = 1, [0x4B] = 0, [0x4C] = 1, [0x4D] = 1, [0x4E] = 1, [0x4F] = 0,
    [0x50] = 1, [0x51] = 1, [0x52] = 0, [0x53] = 0, [0x54] = 0, [0x55] = 1, [0x56] = 1, [0x57] = 0,
    [0x58] = 1, [0x59] = 1, [0x5A] = 0, [0x5B] = 0, [0x5C] = 0, [0x5D] = 1, [0x5E] = 1, [0x5F] = 0,
    [0x60] = 1, [0x61] = 1, [0x62] = 0, [0x63] = 0, [0x64] = 0, [0x65] = 1, [0x66] = 1, [0x67] = 0,
    [0x68] = 1, [0x69] = 1, [0x6A] = 1, [0x6B] = 0, [0x6C] = 1, [0x6D] = 1, [0x6E] = 1, [0x6F] = 0,
    [0x70] = 1, [0x71] = 1, [0x72] = 0, [0x73] = 0, [0x74] = 0, [0x75] = 1, [0x76] = 1, [0x77] = 0,
    [0x78] = 1, [0x79] = 1, [0x7A] = 0, [0x7B] = 0, [0x7C] = 0, [0x7D] = 1, [0x7E] = 1, [0x7F] = 0,
    [0x80] = 0, [0x81] = 1, [0x82] = 0, [0x83] = 0, [0x84] = 1, [0x85] = 1, [0x86] = 1, [0x87] = 0,
    [0x88] = 1, [0x89] = 0, [0x8A] = 1, [0x8B] = 0, [0x8C] = 1, [0x8D] = 1, [0x8E] = 1, [0x8F] = 0,
    [0x90] = 1, [0x91] = 1, [0x92] = 0, [0x93] = 0, [0x94] = 1, [0x95] = 1, [0x96] = 1, [0x97] = 0,
    [0x98] = 1, [0x99] = 1, [0x9A] = 1, [0x9B] = 0, [0x9C] = 0, [0x9D] = 1, [0x9E] = 0, [0x9F] = 0,
    [0xA0] = 1, [0xA1] = 1, [0xA2] = 1, [0xA3] = 0, [0xA4] = 1, [0xA5] = 1, [0xA6] = 1, [0xA7] = 0,
    [0xA8] = 1, [0xA9] = 1, [0xAA] = 1, [0xAB] = 0, [0xAC] = 1, [0xAD] = 1, [0xAE] = 1, [0xAF] = 0,
    [0xB0] = 1, [0xB1] = 1, [0xB2] = 0, [0xB3] = 0, [0xB4] = 1, [0xB5] = 1, [0xB6] = 1, [0xB7] = 0,
    [0xB8] = 1, [0xB9] = 1, [0xBA] = 1, [0xBB] = 0, [0xBC] = 1, [0xBD] = 1, [0xBE] = 1, [0xBF] = 0,
    [0xC0] = 1, [0xC1] = 1, [0xC2] = 0, [0xC3] = 0, [0xC4] = 1, [0xC5] = 1, [0xC6] = 1, [0xC7] = 0,
    [0xC8] = 1, [0xC9] = 1, [0xCA] = 1, [0xCB] = 0, [0xCC] = 1, [0xCD] = 1, [0xCE] = 1, [0xCF] = 0,
    [0xD0] = 1, [0xD1] = 1, [0xD2] = 0, [0xD3] = 0, [0xD4] = 0, [0xD5] = 1, [0xD6] = 1, [0xD7] = 0,
    [0xD8] = 1, [0xD9] = 1, [0xDA] = 0, [0xDB] = 0, [0xDC] = 0, [0xDD] = 1, [0xDE] = 1, [0xDF] = 0,
    [0xE0] = 1, [0xE1] = 1, [0xE2] = 0, [0xE3] = 0, [0xE4] = 1, [0xE5] = 1, [0xE6] = 1, [0xE7] = 0,
    [0xE8] = 1, [0xE9] = 1, [0xEA] = 1, [0xEB] = 0, [0xEC] = 1, [0xED] = 1, [0xEE] = 1, [0xEF] = 0,
    [0xF0] = 1, [0xF1] = 1, [0xF2] = 0, [0xF3] = 0, [0xF4] = 0, [0xF5] = 1, [0xF6] = 1, [0xF7] = 0,
    [0xF8] = 1, [0xF9] = 1, [0xFA] = 0, [0xFB] = 0, [0xFC] = 0, [0xFD] = 1, [0xFE] = 1, [0xFF] = 0,
};


void run_suite_opcodes_mos6502(void) {
    suite_begin("Opcodes (MOS 6502)");
    for (int opcode = 0; opcode <= 0xFF; ++opcode) {
        if (supported[opcode]) {
            run_tests("opcode-mos6502", opcode, false);
        }
    }
    suite_end();
}

void run_suite_opcodes_nes6502(void) {
    suite_begin("Opcodes (NES 6502)");
    for (int opcode = 0; opcode <= 0xFF; ++opcode) {
        if (supported[opcode]) {
            run_tests("opcode-nes6502", opcode, true);
        }
    }
    suite_end();
}
