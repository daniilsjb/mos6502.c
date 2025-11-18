/*
 * mos6502.c - see license at the bottom, no warranty implied, use at your own risk;
 *   made by daniilsjb (https://github.com/daniilsjb/mos6502.c)
 *
 * An emulator of the MOS 6502 Technology CPU in standard C99. This code is written
 * as a standalone module that can be used in any system that must emulate the 6502,
 * such as Apple I, BBC Micro, Commodore 64, or NES. It has no external dependencies
 * aside from several type definitions from the standard library.
 *
 * Features:
 *
 * ~ Implements all 151 of the official opcodes for the MOS 6502 CPU.
 * ~ Provides cycle-accuracy by emulating every single memory read and write.
 * ~ Optionally supports binary-coded decimal arithmetic for platforms that allow it.
 * ~ Correctness extensively verified through tens of thousands of automated tests.
 * ~ Standalone and cross-platform library relying only on type definitions of C99.
 * ~ Easily pluggable into any project - simply copy two files, and you're done!
 *
 * Getting Started:
 *
 * Simply copy `mos6502.h` and `mos6502.c` to any location in your project. You may
 * either compile the source code along with the rest of the project, or compile it
 * into a static or dynamic library linked against your target using whichever build
 * system you prefer. If necessary, adjust include paths in the implementation file.
 * Then simply include the header in your own code.
 *
 * Usage:
 *
 * To start using the emulator, you first need to define the structure of the system
 * you're emulating. Remember that the CPU is only responsible for executing program
 * instructions, which is useless if there is no I/O involved. Typically, this means
 * that a system will have other components beside the CPU. As a very basic example,
 * suppose that the target system also has 64KB of RAM:
 *
 *   #include "mos6502.h"
 *
 *   typedef struct {
 *       mos6502_t cpu;
 *       uint8_t   mem[0x10000];
 *   } system_state_t;
 *
 * In practice, your system may include components such as a graphical processing
 * unit, an audio processing unit, a keyboard, an LCD screen, a disk drive, etc.
 * However, since 6502 uses a memory-mapped architecture, there are no dedicated
 * instructions for communicating with external devices. Instead, communication
 * occurs through ordinary memory read and write operations.
 *
 * To facilitate this, the registers of every device are mapped to specific addresses
 * in the system's 16-bit address space, and the CPU interacts with them by reading
 * from or writing to these addresses. In a sense, an "address" is merely a port to
 * an arbitrary device, not necessarily RAM.
 *
 * Thus, to connect the CPU to the emulated system, you only need to provide callbacks
 * for reading and writing memory. Both callbacks accept a 16-bit address, which can be
 * used to identify the device that the CPU is talking to. The system state may also be
 * optionally passed via a pointer to avoid global variables.
 *
 * Note that this emulation assumes the CPU to be the driver of the system, meaning
 * that each time the CPU makes a read or write, the state of the whole system must
 * be advanced forward by exactly one CPU cycle. Thus, the CPU moves the emulation,
 * while all other devices are constantly catching up with it. This means that the
 * memory callbacks may also involve updates to other components.
 *
 * For our basic example, however, we will simply map most addresses to RAM and use
 * one port for serial I/O operations: writing to it will print a single character
 * to stdout, while reading from it will poll a single character from stdin:
 *
 *   #include <stdio.h>
 *   #include <string.h>
 *
 *   #define IO_PORT_W 0xF000
 *   #define IO_PORT_R 0xF000
 *
 *   static void on_cpu_write(uint16_t address, uint8_t data, void *bus) {
 *       system_state_t *system = (system_state_t *)bus;
 *       if (address == IO_PORT_W) {
 *           putchar((char)data);
 *       } else {
 *           system->mem[address] = data;
 *       }
 *   }
 *
 *   static uint8_t on_cpu_read(uint16_t address, void *bus) {
 *       system_state_t *system = (system_state_t *)bus;
 *       if (address == IO_PORT_R) {
 *           return (uint8_t)getchar();
 *       } else {
 *           return system->mem[address];
 *       }
 *   }
 *
 * Now we can initialize the system. First, create the system state and configure
 * the CPU with the above callbacks. This will perform basic initialization of the
 * emulator, such as setting up registers and interrupt lines:
 *
 *   system_state_t system = {0};
 *   system.cpu = mos6502_create((mos6502_desc_t) {
 *       .write = &on_cpu_write,
 *       .read  = &on_cpu_read,
 *       .bus   = &system,
 *   });
 *
 * Next, the emulated program itself must be made available to the CPU. Since bus
 * operations in this example read from RAM, we must copy the program instructions
 * into it. Typically, they will come from a ROM file, but the program can also be
 * embedded statically into the source code. As a demonstration, this program will
 * simply print "Hello!" to the standard output:
 *
 *   uint8_t program[] = {
 *       0xA2, 0x00,        // $0600  LDX #$00    ; Initialize index to 0.
 *       0xBD, 0x11, 0x06,  // $0602  LDA $0611,X ; Load character from string.
 *       0xF0, 0x07,        // $0605  BEQ $07     ; If zero, jump to end.
 *       0x8D, 0x00, 0xF0,  // $0607  STA $F000   ; Write the character.
 *       0xE8,              // $060A  INX         ; Increment index.
 *       0x4C, 0x02, 0x06,  // $060B  JMP $0602   ; Loop back.
 *       0x4C, 0x0E, 0x06,  // $060E  JMP $060E   ; Jump to self (halt).
 *       'H', 'e', 'l', 'l', 'o', '!', 0x00
 *   };
 *
 *   memcpy(&system.mem[0x0600], program, sizeof(program));
 *
 * However, the CPU is not yet aware of where to start program execution, as it
 * must first execute a reset sequence to enter a well-defined state. To do so,
 * we must set the program's entry point in the interrupt vector table (IVT)
 * and then trigger the RESET interrupt:
 *
 *   system.mem[MOS6502_RES + 0] = 0x00; // Lo-byte
 *   system.mem[MOS6502_RES + 1] = 0x06; // Hi-byte
 *
 *   mos6502_reset(&system.cpu);
 *
 * Lastly, we can execute the program by repeatedly calling `mos6502_step()`, which
 * performs one fetch-decode-execute cycle per call. We should also remember to
 * handle the different status codes that can be returned:
 *
 *   mos6502_status_t status;
 *   while ((status = mos6502_step(&system.cpu)) != MOS6502_LOOP) {
 *       if (status == MOS6502_UNKNOWN) {
 *           printf("\nUnknown opcode 0x%02X at address 0x%04X\n",
 *               system.cpu.ir,
 *               system.cpu.iar
 *           );
 *       }
 *   }
 *
 * In more advanced situations, you may also need to trigger interrupts by calling
 * the `mos6502_set_irq()` and `mos6502_set_nmi()` procedures within the body of
 * the loop. These are typically controlled by other devices in the system, so
 * their usage depends highly on the specifics of the architecture.
 *
 * And that's it! We have ourselves a simple but functional 6502 system. If we add
 * in a multimedia library and reserve addresses for graphical, audio, and input
 * devices, we can easily emulate much more sophisticated systems, from gaming
 * consoles to operating systems and programming language interpreters.
 */

#ifndef MOS6502_H
#define MOS6502_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MOS6502_API
    #ifdef __EMSCRIPTEN__
        #include <emscripten.h>
        #define MOS6502_API EMSCRIPTEN_KEEPALIVE
    #elif defined(_MSC_VER)
        #define MOS6502_API __declspec(dllexport)
    #elif defined(__GNUC__)
        #define MOS6502_API __attribute__((visibility("default")))
    #elif defined(__clang__)
        #define MOS6502_API __attribute__((visibility("default")))
    #else
        #define MOS6502_API extern
    #endif
#endif

#include <stdint.h>
#include <stdbool.h>

#define MOS6502_C (1 << 0) /* Carry       */
#define MOS6502_Z (1 << 1) /* Zero        */
#define MOS6502_I (1 << 2) /* IRQ Disable */
#define MOS6502_D (1 << 3) /* Decimal     */
#define MOS6502_B (1 << 4) /* Break       */
#define MOS6502_U (1 << 5) /* Unused      */
#define MOS6502_V (1 << 6) /* Overflow    */
#define MOS6502_N (1 << 7) /* Negative    */

#define MOS6502_NMI 0xFFFA /* Non-Maskable Interrupt */
#define MOS6502_RES 0xFFFC /* Reset Interrupt        */
#define MOS6502_IRQ 0xFFFE /* Interrupt Request      */

/*
 * Procedure representing a write operation through the system bus.
 *
 * The `address` corresponds to the port that the CPU is writing to, which may be
 * a memory location or a device register, while `data` is the value being written.
 * The `bus` is represented by a pointer to arbitrary user data, which is provided
 * during emulator initialization and passed as-is to this procedure. Decoding of
 * the supplied address is entirely up to the client code.
 */
typedef void (*mos6502_write_fn)(uint16_t address, uint8_t data, void *bus);

/*
 * Procedure representing a read operation through the system bus.
 *
 * The `address` corresponds to the port that the CPU is reading from, which may be
 * a memory location or a device register, while the return value is the response.
 * The `bus` is represented by a pointer to arbitrary user data, which is provided
 * during emulator initialization and passed as-is to this procedure. Decoding of
 * the supplied address is entirely up to the client code.
 */
typedef uint8_t (*mos6502_read_fn)(uint16_t address, void *bus);

/*
 * Allowed options for constructing an emulator instance.
 *
 * To use the CPU from client code (metaphorically, connecting it to the system bus),
 * it is necessary to provide two callbacks - one for writing to memory and one for
 * reading from memory. Optionally, these callbacks may be provided with a pointer
 * to arbitrary user data to avoid relying on global variables.
 *
 * Aside from `write` and `read`, all other fields are optional and can be omitted,
 * in which case they will either be ignored or set to defaults. It is assumed that
 * omitted fields are always zero-initialized. To simplify this, it is recommended
 * to use the C99 struct initialization syntax.
 */
typedef struct mos6502_desc {
    /*
     * (Required)
     * Callback used for writing to the system bus, called on every write cycle.
     *
     * The CPU writes to the system by placing a 16-bit address and an 8-bit payload
     * on to the system bus. The meaning of this operation depends on the underlying
     * system architecture, but is typically intended to produce side effects. Every
     * invocation of this callback corresponds to a single write cycle.
     *
     * See `mos6502_write_fn` typedef for details.
     */
    mos6502_write_fn write;

    /*
     * (Required)
     * Callback used for reading from the system bus, called on every read cycle.
     *
     * The CPU reads from the system by placing a 16-bit address to the system bus and
     * receiving an 8-bit response. The meaning of this operation depends on underlying
     * system architecture, but is usually intended for polling state. Every invocation
     * of this callback corresponds to a single read cycle.
     *
     * See `mos6502_read_fn` typedef for details.
     */
    mos6502_read_fn read;

    /*
     * Pointer to arbitrary user data, passed as-is to all callbacks.
     *
     * Metaphorically, this pointer is a reference to the system bus that the CPU is
     * connected to. Since reading and writing memory typically involves interacting
     * with other devices in the system (including RAM and I/O), this will typically
     * point to a struct containing other components of the system being emulated.
     *
     * If the callbacks rely on global variables, this may be omitted and then simply
     * ignored in the callback implementations. Otherwise, the callbacks must cast
     * this pointer to the actual underlying data type. In other libraries, this
     * would be commonly called `userdata` or similar.
     */
    void *bus;

    /*
     * Disables binary-coded decimal arithmetic.
     *
     * The 6502 supports arithmetic in binary-coded decimal (BCD) mode, where each
     * byte corresponds to two 4-bit decimal digits. However, due to the feature
     * being proprietary, derived CPUs had to either omit or disable this mode.
     *
     * For compatibility with these systems, setting this option to true will make
     * the emulator ignore the decimal flag when executing instructions and instead
     * always perform arithmetic in binary mode.
     */
    bool bcd_disabled;
} mos6502_desc_t;

/*
 * Status of the CPU after executing an instruction.
 *
 * See `mos6502_step()` for details.
 */
typedef enum mos6502_status {
    MOS6502_OK,      /* Successfully executed an instruction. */
    MOS6502_LOOP,    /* Jumped back to the same address, CPU is looping. */
    MOS6502_UNKNOWN, /* Encountered an unsupported opcode. */
} mos6502_status_t;

/*
 * The entire state of a CPU instance.
 *
 * Visible registers (PC, A, X, Y, S, P) are accessible through CPU instructions.
 * Internal registers (IR, MAR, IAR) are used by the emulator during execution of
 * the current instruction and are updated in each `mos6502_step()` call.
 *
 * Although not strictly part of the official 6502 documentation, IR refers to the
 * opcode of the currently executing instruction, IAR is the address of the opcode,
 * and MAR is the effective address determined by its addressing mode.
 *
 * Instances of this structure should be created using `mos6502_create()` rather
 * than being initialized manually, as proper setup requires configuration of the
 * bus interface, status flags, interrupt signals, and so on.
 */
typedef struct mos6502 {
    /* Visible registers */
    uint16_t pc;  /* Program Counter  */
    uint8_t a;    /* Accumulator      */
    uint8_t x;    /* Index X Register */
    uint8_t y;    /* Index Y Register */
    uint8_t s;    /* Stack Pointer    */
    uint8_t p;    /* Status Register  */

    /* Internal register */
    uint8_t ir;   /* Instruction Register         */
    uint16_t iar; /* Instruction Address Register */
    uint16_t mar; /* Memory Address Register      */

    /* Interrupt signals */
    uint8_t irq; /* Interrupt Request      */
    uint8_t nmi; /* Non-Maskable Interrupt */
    uint8_t nmi_latch; /* Output of NMI edge detector */

    /* Miscellaneous */
    uint8_t has_bcd; /* Support binary-coded decimals? */

    /* Bus interface */
    mos6502_write_fn write;
    mos6502_read_fn read;
    void *bus;
} mos6502_t;

/*
 * Sets up a new emulator instance based on the provided configuration.
 *
 * This procedure creates and initializes a new instance of the emulator state,
 * making it ready to execute instructions. However, after being returned from
 * this function, the virtual CPU itself is not yet aware of where the program
 * itself is located in the address space.
 *
 * Initially, the program counter of the returned CPU is set to zero from this
 * function, meaning the emulator will be executing instructions from the very
 * beginning of the address space by default. However, this is not correct for
 * many 6502-based systems.
 *
 * To ensure that the CPU is in a well-defined state when it starts executing
 * a given program, it must first perform a reset interrupt sequence, which is
 * conceptually equivalent to calling the `main()` function. For details, see
 * the `mos6502_reset()` procedure.
 *
 * While it is possible to directly set the emulator's program counter to the
 * desired address, this will cause the CPU's status flags to be initialized
 * incorrectly and will skip several read cycles. This is appropriate mostly
 * for testing and simple programs.
 */
MOS6502_API mos6502_t mos6502_create(mos6502_desc_t);

/*
 * Initiates the RESET sequence from the interrupt vector table.
 *
 * This procedure triggers the RESET interrupt, which is handled by the routine
 * specified in the interrupt vector table (addresses 0xFFFC-0xFFFD). In effect,
 * this will jump to the RESET handler while disabling IRQs.
 *
 * The actual execution of the interrupt handler occurs during subsequent calls
 * to the `mos6502_step()` procedure. Unlike other interrupts, the RESET serves
 * as the entry point to the program's main execution flow, meaning its handler
 * effectively represents the entirety of the program.
 *
 * Note that this procedure will perform exactly 7 read cycles using the `read`
 * callback provided during creation of the emulator, of which only the last 2
 * will actually determine the jump address.
 */
MOS6502_API void mos6502_reset(mos6502_t *);

/*
 * Performs the fetch-decode-execute sequence for the next instruction.
 *
 * Calling this procedure will make the emulator execute the next instruction in
 * the program, as determined by the program counter's current value. Depending
 * on its complexity and address mode, running a single instruction may require
 * from 2 to 7 memory cycles altogether.
 *
 * If the next instruction uses an unofficial opcode that is not supported, this
 * procedure will return the `MOS6502_UNKNOWN` status, and the offending opcode
 * may be retrieved from the `.ir` attribute of the CPU. In this case, the only
 * effect is that a single byte of memory is fetched, causing one read cycle and
 * advancing the program counter to the next address.
 *
 * Otherwise, the emulator has successfully executed the instruction. Normally,
 * this would result in the `MOS6502_OK` status being returned. However, if the
 * instruction caused a jump back to itself, the `MOS6502_LOOP` status will be
 * returned instead to indicate that the emulator has entered an infinite loop.
 * Subsequent calls to `mos6502_step()` will repeat the instruction.
 *
 * In certain cases, this may be used to signal to the emulator that the program
 * has halted (i.e., finished executing). However, this could also indicate that
 * the CPU is busy-waiting for an I/O process to complete, and will break out of
 * the loop via an interrupt handler. Thus, this status may be ignored entirely
 * if irrelevant to the target system.
 */
MOS6502_API mos6502_status_t mos6502_step(mos6502_t *);

/* 
 * Sets the current signal of the IRQ line.
 *
 * The IRQ is active-low and level-detected, meaning that the interrupt will be
 * asserted for as long as the IRQ signal is held at zero while the IRQ Disable
 * flag is clear (see the status register).
 *
 * In other words, calls to this procedure are interpreted as follows
 * - If `new_irq_state == 0`, the IRQ will be pending until cleared.
 * - If `new_irq_state != 0`, the IRQ will be cleared.
 *
 * Note that this implementation asserts the state of the interrupt lines right
 * before executing the next instruction. This is not accurate for cycle-level
 * precision, but it should not matter for most use cases.
 */
MOS6502_API void mos6502_set_irq(mos6502_t *, uint8_t new_irq_state);

/* 
 * Sets the current signal of the NMI line.
 *
 * The NMI is falling-edge-detected, meaning that the interrupt will be asserted
 * only once after the NMI signal goes from high to low. It is assumed that the
 * edge detector operates without delay.
 *
 * Initially, the NMI signal is held high after emulator initialization. Whether
 * the interrupt is asserted after calling this procedure depends on the previous
 * state of the NMI signal. Thus, a reliable way to trigger an NMI is to pull the
 * signal low and then back high:
 *
 *   mos6502_set_nmi(&cpu, 0);
 *   mos6502_set_nmi(&cpu, 1);
 *
 * Note that this implementation asserts the state of the interrupt lines right
 * before executing the next instruction. This is not accurate for cycle-level
 * precision, but it should not matter for most use cases.
 */
MOS6502_API void mos6502_set_nmi(mos6502_t *, uint8_t new_nmi_state);

#ifdef __cplusplus
}
#endif

#endif

/*
MIT License

Copyright (c) 2021-2025 Daniils Buts

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
