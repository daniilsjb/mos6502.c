/*
 * mos6502.c - see license at the bottom, no warranty implied, use at your own risk;
 *   made by daniilsjb (https://github.com/daniilsjb/mos6502.c)
 *
 * The following is an implementation of a MOS 6502 Technology CPU in standard C99.
 * Since the code is fairly long, it is organized into sections to ease navigation,
 * but is otherwise intended to be read from top to bottom.
 *
 * Arguably, the most interesting aspect of emulating a 6502 has to do with encoding
 * all the different combinations of operations and addressing modes for each opcode.
 * There is a wide variety of interesting ways to accomplish that, from preprocessing
 * and code generation to breaking each instruction down into a "microcode" sequence
 * that can be encoded and decoded similarly to a PLA.
 *
 * Instead, the approach I chose for this implementation is to treat addressing modes
 * and operations as separate units that can be composed together to produce all the
 * combinations of opcodes that we need. This works because addressing modes are only
 * responsible for determining the address of the operand without any concern for how
 * it is used, while operations typically use their operands without any knowledge of
 * where they came from. This makes it possible to implement each addressing mode and
 * operation once and make them interact via a common interface.
 *
 * In this implementation, the internal Memory Address Register (MAR) is used to store
 * the address produced by the addressing mode. Operation can then read their operands
 * from this address, if applicable, during execution. This works well for most of the
 * opcodes except for those using accumulator addressing, since it is the only one that
 * does not produce an actual address. As there are only four such opcodes, their logic
 * is simply duplicated for accumulator addressing.
 *
 * Interrupts are implemented completely separately from opcodes. In total, there are
 * only three interrupt sequences: reset interrupts, hardware interrupts (IRQ/NMI),
 * and software interrupts (BRK). While these sequences may seem nearly identical,
 * they are conceptually independent and have significant differences, so they are
 * implemented separately without concern for duplication.
 *
 * Thus, the main fetch-decode-execute cycle works in a rather straightforward way.
 * First, the CPU checks whether there are any pending hardware interrupts. If so,
 * it executes the interrupt. Otherwise, it fetches the next opcode, executes its
 * addressing mode logic, and then executes its operational logic. To communicate
 * the state of execution, each operation returns a status to indicate whether it
 * was performed successfully, halted the CPU, etc.
 */

#include "mos6502.h"

mos6502_t mos6502_create(mos6502_desc_t desc) {
    /* Avoid designated initializer in case we're on a pre-C++20 compiler. */
    mos6502_t cpu = {0};
    cpu.write = desc.write;
    cpu.read = desc.read;
    cpu.bus = desc.bus;

    /* Since BCD mode is standard, we assume that it is enabled by default. */
    cpu.has_bcd = !desc.bcd_disabled;

    /* All interrupt lines are active-low, so they are held high initially. */
    cpu.irq = 1;
    cpu.nmi = 1;
    cpu.nmi_latch = 1;

    return cpu;
}

/* Utilities =============================================================== */

static void write(mos6502_t *cpu, uint16_t address, uint8_t data) {
    cpu->write(address, data, cpu->bus);
}

static uint8_t read(mos6502_t *cpu, uint16_t address) {
    return cpu->read(address, cpu->bus);
}

static uint8_t fetch(mos6502_t *cpu) {
    return read(cpu, cpu->pc++);
}

static uint16_t on_stack(uint8_t offset) {
    return 0x0100 | offset;
}

static uint8_t pull(mos6502_t *cpu) {
    return read(cpu, on_stack(++cpu->s));
}

static void push(mos6502_t *cpu, uint8_t data) {
    write(cpu, on_stack(cpu->s--), data);
}

static uint8_t get_flags(mos6502_t *cpu, uint8_t flags) {
    return cpu->p & flags;
}

static void set_flags(mos6502_t *cpu, uint8_t flags, bool condition) {
    if (condition) {
        cpu->p |= flags;
    } else {
        cpu->p &= ~flags;
    }
}

static void set_flags_zn(mos6502_t *cpu, uint8_t value) {
    set_flags(cpu, MOS6502_Z, value == 0);
    set_flags(cpu, MOS6502_N, value & 0x80);
}

/* Interrupts ============================================================== */

void mos6502_reset(mos6502_t *cpu) {
    /* Cycles 1, 2: Pre-fetch the two instruction bytes and discard their values.
     * PC increments are suppressed, therefore we end up reading twice from the same address. */
    read(cpu, cpu->pc);
    read(cpu, cpu->pc);

    /* Cycles 3, 4, 5: During reset, memory writes are suppressed, and are instead wired to
     * perform reads. Therefore, the stack pointer is decremented thrice without modifying memory. */
    read(cpu, on_stack(cpu->s--));
    read(cpu, on_stack(cpu->s--));
    read(cpu, on_stack(cpu->s--));

    /* Cycle 6: Fetch PCL from the reset vector. */
    cpu->pc = 0x0000;
    cpu->pc |= read(cpu, MOS6502_RES + 0) << 0;

    /* Between cycles 6 and 7 the CPU sets I flag. */
    set_flags(cpu, MOS6502_I, 1);

    /* Cycle 7: Fetch PCH from the reset vector. */
    cpu->pc |= read(cpu, MOS6502_RES + 1) << 8;
}

void mos6502_set_irq(mos6502_t *cpu, uint8_t new_irq_state) {
    /* IRQ is asserted continuously based on its level. */
    cpu->irq = new_irq_state;
}

void mos6502_set_nmi(mos6502_t *cpu, uint8_t new_nmi_state) {
    /* NMI is asserted once after every falling edge. */
    if (cpu->nmi && !new_nmi_state) {
        cpu->nmi_latch = new_nmi_state;
    }

    cpu->nmi = new_nmi_state;
}

static bool has_pending_interrupts(mos6502_t *cpu) {
    bool has_pending_nmi = !cpu->nmi_latch;
    bool has_pending_irq = !cpu->irq && !get_flags(cpu, MOS6502_I);
    return has_pending_nmi || has_pending_irq;
}

static uint16_t select_interrupt_vector(mos6502_t *cpu) {
    /* The CPU does not "remember" which kind of interrupt was asserted during polling.
     * This means that the appropriate interrupt vector is only selected once the BRK
     * sequence has begun, and a higher priority interrupt (NMI) may still be selected
     * even if a lower priority interrupt (IRQ) was initially asserted. This is known
     * as interrupt hijacking. */
    if (cpu->nmi_latch == 0) {
        cpu->nmi_latch = 1;
        return MOS6502_NMI;
    } else {
        return MOS6502_IRQ;
    }
}

static mos6502_status_t handle_hardware_interrupt(mos6502_t *cpu) {
    /* Cycles 1, 2: Pre-fetch the two instruction bytes and discard their values.
     * PC increments are suppressed, so we end up reading twice from the same address. */
    read(cpu, cpu->pc);
    read(cpu, cpu->pc);

    /* Cycles 3, 4: Push PC onto the stack. */
    push(cpu, (uint8_t)((cpu->pc >> 8) & 0x00FF));
    push(cpu, (uint8_t)((cpu->pc >> 0) & 0x00FF));

    /* Between cycles 4 and 5 the CPU determines which interrupt vector is used. */
    uint16_t vector = select_interrupt_vector(cpu);

    /* Cycle 5: Push P onto the stack (B flag is clear, U flag appears set). */
    push(cpu, (uint8_t)((cpu->p & ~MOS6502_B) | MOS6502_U));

    /* Cycle 6: Fetch PCL from the interrupt vector. */
    cpu->pc = 0x0000;
    cpu->pc |= read(cpu, vector + 0) << 0;

    /* Between cycles 6 and 7 the CPU sets I flag. */
    set_flags(cpu, MOS6502_I, 1);

    /* Cycle 7: Fetch PCH from the interrupt vector. */
    cpu->pc |= read(cpu, vector + 1) << 8;
    return MOS6502_OK;
}

static mos6502_status_t handle_software_interrupt(mos6502_t *cpu) {
    /* Cycle 2: The padding byte has already been pre-fetched, simply discard it. */
    cpu->pc++;

    /* Cycles 3, 4: Push PC onto the stack. */
    push(cpu, (uint8_t)((cpu->pc >> 8) & 0x00FF));
    push(cpu, (uint8_t)((cpu->pc >> 0) & 0x00FF));

    /* Between cycles 4 and 5 the CPU determines which interrupt vector is used. */
    uint16_t vector = select_interrupt_vector(cpu);

    /* Cycle 5: Push P onto the stack (B flag is set, U flag appears set). */
    push(cpu, cpu->p | MOS6502_B | MOS6502_U);

    /* Cycle 6: Fetch PCL from the interrupt vector. */
    cpu->pc = 0x0000;
    cpu->pc |= read(cpu, vector + 0) << 0;

    /* Between cycles 6 and 7 the CPU sets I flag. */
    set_flags(cpu, MOS6502_I, 1);

    /* Cycle 7: Fetch PCH from the interrupt vector. */
    cpu->pc |= read(cpu, vector + 1) << 8;
    return MOS6502_OK;
}

/* Addressing Modes ======================================================== */

/* Pre-fetching */
static void am_pre(mos6502_t *cpu) {
    /* The 6502 always fetches at least two bytes per instruction, even if it has no operands.
     * Pre-fetch occurs immediately after fetching the opcode, but does not increment the PC.
     * This behavior is important primarily for implied and accumulator addressing modes. */
    cpu->mar = read(cpu, cpu->pc);
}

/* Implied addressing */
static void am_imp(mos6502_t *cpu) {
    am_pre(cpu);
}

/* Accumulator addressing */
static void am_acc(mos6502_t *cpu) {
    am_pre(cpu);
}

/* Immediate addressing */
static void am_imm(mos6502_t *cpu) {
    cpu->mar = cpu->pc++;
}

/* Zero-page addressing */
static void am_zpg(mos6502_t *cpu) {
    cpu->mar = fetch(cpu);
}

static void am_zpg_indexed(mos6502_t *cpu, uint8_t index) {
    uint16_t address = fetch(cpu);
    read(cpu, address);
    cpu->mar = (address + index) & 0x00FF;
}

/* Zero-page,X addressing */
static void am_zpx(mos6502_t *cpu) {
    am_zpg_indexed(cpu, cpu->x);
}

/* Zero-page,Y addressing */
static void am_zpy(mos6502_t *cpu) {
    am_zpg_indexed(cpu, cpu->y);
}

/* Absolute addressing */
static void am_abs(mos6502_t *cpu) {
    uint16_t address_lo = (uint16_t)(fetch(cpu) << 0);
    uint16_t address_hi = (uint16_t)(fetch(cpu) << 8);
    cpu->mar = (uint16_t)(address_hi | address_lo);
}

static void am_abs_indexed(mos6502_t *cpu, uint8_t index, bool force_extra_cycle) {
    uint16_t address_lo = (uint16_t)(fetch(cpu) << 0);
    uint16_t address_hi = (uint16_t)(fetch(cpu) << 8);

    uint16_t effective = (uint16_t)((address_hi | address_lo) + index);
    uint16_t premature = (uint16_t)((address_hi | (effective & 0x00FF)));

    if (force_extra_cycle || premature != effective) {
        read(cpu, premature);
    }

    cpu->mar = effective;
}

/* Absolute,X addressing */
static void am_abx(mos6502_t *cpu) {
    am_abs_indexed(cpu, cpu->x, true);
}

/* Absolute,X addressing (specialized for read instructions) */
static void am_rbx(mos6502_t *cpu) {
    am_abs_indexed(cpu, cpu->x, false);
}

/* Absolute,Y addressing */
static void am_aby(mos6502_t *cpu) {
    am_abs_indexed(cpu, cpu->y, true);
}

/* Absolute,Y addressing (specialized for read instructions) */
static void am_rby(mos6502_t *cpu) {
    am_abs_indexed(cpu, cpu->y, false);
}

/* Indirect addressing */
static void am_ind(mos6502_t *cpu) {
    uint16_t pointer_lo = (uint16_t)(fetch(cpu) << 0);
    uint16_t pointer_hi = (uint16_t)(fetch(cpu) << 8);

    uint16_t address_lo = (uint16_t)(read(cpu, pointer_hi | ((pointer_lo + 0) & 0x00FF)) << 0);
    uint16_t address_hi = (uint16_t)(read(cpu, pointer_hi | ((pointer_lo + 1) & 0x00FF)) << 8);

    cpu->mar = (uint16_t)(address_hi | address_lo);
}

/* Indexed indirect addressing */
static void am_idx(mos6502_t *cpu) {
    uint16_t pointer = fetch(cpu);

    read(cpu, pointer);
    pointer += cpu->x;

    uint16_t address_lo = (uint16_t)(read(cpu, (pointer + 0) & 0x00FF) << 0);
    uint16_t address_hi = (uint16_t)(read(cpu, (pointer + 1) & 0x00FF) << 8);

    cpu->mar = (uint16_t)(address_hi | address_lo);
}

static void am_ind_indexed(mos6502_t *cpu, bool force_extra_cycle) {
    uint16_t pointer = fetch(cpu);

    uint16_t address_lo = (uint16_t)(read(cpu, (pointer + 0) & 0x00FF) << 0);
    uint16_t address_hi = (uint16_t)(read(cpu, (pointer + 1) & 0x00FF) << 8);

    uint16_t effective = (uint16_t)((address_hi | address_lo) + cpu->y);
    uint16_t premature = (uint16_t)((address_hi | (effective & 0x00FF)));

    if (force_extra_cycle || premature != effective) {
        read(cpu, premature);
    }

    cpu->mar = effective;
}

/* Indirect indexed addressing */
static void am_idy(mos6502_t *cpu) {
    am_ind_indexed(cpu, true);
}

/* Indirect indexed addressing (specialized for read instructions) */
static void am_rdy(mos6502_t *cpu) {
    am_ind_indexed(cpu, false);
}

/* Relative addressing */
static void am_rel(mos6502_t *cpu) {
    int8_t offset = (int8_t)fetch(cpu);
    cpu->mar = (uint16_t)(cpu->pc + offset);
}

/* Operations ============================================================== */

static mos6502_status_t adc_bin(mos6502_t *cpu, uint8_t value) {
    uint16_t a = cpu->a;
    uint16_t b = value;
    uint16_t c = get_flags(cpu, MOS6502_C);

    uint16_t result = (uint16_t)(a + b + c);
    cpu->a = (uint8_t)result;

    set_flags(cpu, MOS6502_V, (a ^ result) & (b ^ result) & 0x80);
    set_flags(cpu, MOS6502_C, result & 0x100);
    set_flags_zn(cpu, cpu->a);
    return MOS6502_OK;
}

static mos6502_status_t adc_dec(mos6502_t *cpu, uint8_t value) {
    uint16_t a = cpu->a;
    uint16_t b = value;
    uint16_t c = get_flags(cpu, MOS6502_C);

    uint16_t result = (uint16_t)((a & 0x0F) + (b & 0x0F) + c);
    if (result > 0x09) {
        result = ((result + 0x06) & 0x0F) | 0x10;
    }

    result += (uint16_t)((a & 0xF0) + (b & 0xF0));

    set_flags(cpu, MOS6502_Z, ((a + b + c) & 0xFF) == 0);
    set_flags(cpu, MOS6502_V, (a ^ result) & (b ^ result) & 0x80);
    set_flags(cpu, MOS6502_N, result & 0x80);

    if (result > 0x9F) {
        result = ((result + 0x60) & 0xFF) | 0x100;
    }

    cpu->a = (uint8_t)result;

    set_flags(cpu, MOS6502_C, result & 0x100);
    return MOS6502_OK;
}

/* ADC - Add memory to accumulator with carry */
static mos6502_status_t op_adc(mos6502_t *cpu) {
    uint8_t value = read(cpu, cpu->mar);
    if (cpu->has_bcd && get_flags(cpu, MOS6502_D)) {
        return adc_dec(cpu, value);
    } else {
        return adc_bin(cpu, value);
    }
}

/* AND - Logical AND with accumulator */
static mos6502_status_t op_and(mos6502_t *cpu) {
    cpu->a &= read(cpu, cpu->mar);
    set_flags_zn(cpu, cpu->a);
    return MOS6502_OK;
}

/* ASL - Arithmetic shift left */
static mos6502_status_t op_asl(mos6502_t *cpu) {
    uint8_t value = read(cpu, cpu->mar);
    write(cpu, cpu->mar, value);
    set_flags(cpu, MOS6502_C, value >> 7);
    write(cpu, cpu->mar, value <<= 1);
    set_flags_zn(cpu, value);
    return MOS6502_OK;
}

/* ASL - Arithmetic shift left (on accumulator) */
static mos6502_status_t op_asl_acc(mos6502_t *cpu) {
    set_flags(cpu, MOS6502_C, cpu->a >> 7);
    cpu->a <<= 1;
    set_flags_zn(cpu, cpu->a);
    return MOS6502_OK;
}

static mos6502_status_t branch(mos6502_t *cpu, bool condition) {
    if (!condition) {
        return MOS6502_OK;
    }

    read(cpu, cpu->pc);

    uint16_t effective = cpu->mar;
    uint16_t premature = (cpu->pc & 0xFF00) | (effective & 0x00FF);

    if (effective != premature) {
        read(cpu, premature);
    }

    cpu->pc = effective;

    if (cpu->mar == cpu->iar) {
        return MOS6502_LOOP;
    } else {
        return MOS6502_OK;
    }
}

/* BCC - Branch on carry clear */
static mos6502_status_t op_bcc(mos6502_t *cpu) {
    return branch(cpu, !get_flags(cpu, MOS6502_C));
}

/* BCS - Branch on carry set */
static mos6502_status_t op_bcs(mos6502_t *cpu) {
    return branch(cpu, get_flags(cpu, MOS6502_C));
}

/* BEQ - Branch on result zero */
static mos6502_status_t op_beq(mos6502_t *cpu) {
    return branch(cpu, get_flags(cpu, MOS6502_Z));
}

/* BIT - Test bits in memory with accumulator */
static mos6502_status_t op_bit(mos6502_t *cpu) {
    uint8_t a = cpu->a;
    uint8_t b = read(cpu, cpu->mar);
    set_flags(cpu, MOS6502_Z, (a & b) == 0);
    set_flags(cpu, MOS6502_V, b & 0x40);
    set_flags(cpu, MOS6502_N, b & 0x80);
    return MOS6502_OK;
}

/* BMI - Branch on result minus */
static mos6502_status_t op_bmi(mos6502_t *cpu) {
    return branch(cpu, get_flags(cpu, MOS6502_N));
}

/* BNE - Branch on result not zero */
static mos6502_status_t op_bne(mos6502_t *cpu) {
    return branch(cpu, !get_flags(cpu, MOS6502_Z));
}

/* BPL - Branch on result plus */
static mos6502_status_t op_bpl(mos6502_t *cpu) {
    return branch(cpu, !get_flags(cpu, MOS6502_N));
}

/* BRK - Force interrupt */
static mos6502_status_t op_brk(mos6502_t *cpu) {
    return handle_software_interrupt(cpu);
}

/* BVC - Branch on overflow clear */
static mos6502_status_t op_bvc(mos6502_t *cpu) {
    return branch(cpu, !get_flags(cpu, MOS6502_V));
}

/* BVS - Branch on overflow set */
static mos6502_status_t op_bvs(mos6502_t *cpu) {
    return branch(cpu, get_flags(cpu, MOS6502_V));
}

/* CLC - Clear carry flag */
static mos6502_status_t op_clc(mos6502_t *cpu) {
    set_flags(cpu, MOS6502_C, 0);
    return MOS6502_OK;
}

/* CLD - Clear decimal mode */
static mos6502_status_t op_cld(mos6502_t *cpu) {
    set_flags(cpu, MOS6502_D, 0);
    return MOS6502_OK;
}

/* CLI - Clear interrupt disable */
static mos6502_status_t op_cli(mos6502_t *cpu) {
    set_flags(cpu, MOS6502_I, 0);
    return MOS6502_OK;
}

/* CLV - Clear overflow flag */
static mos6502_status_t op_clv(mos6502_t *cpu) {
    set_flags(cpu, MOS6502_V, 0);
    return MOS6502_OK;
}

static mos6502_status_t compare(mos6502_t *cpu, uint8_t lhs, uint8_t rhs) {
    set_flags(cpu, MOS6502_C, lhs >= rhs);
    set_flags_zn(cpu, lhs - rhs);
    return MOS6502_OK;
}

/* CMP - Compare accumulator */
static mos6502_status_t op_cmp(mos6502_t *cpu) {
    return compare(cpu, cpu->a, read(cpu, cpu->mar));
}

/* CPX - Compare X register */
static mos6502_status_t op_cpx(mos6502_t *cpu) {
    return compare(cpu, cpu->x, read(cpu, cpu->mar));
}

/* CPY - Compare Y register */
static mos6502_status_t op_cpy(mos6502_t *cpu) {
    return compare(cpu, cpu->y, read(cpu, cpu->mar));
}

/* DEC - Decrement memory */
static mos6502_status_t op_dec(mos6502_t *cpu) {
    uint8_t value = read(cpu, cpu->mar);
    write(cpu, cpu->mar, value);
    write(cpu, cpu->mar, --value);
    set_flags_zn(cpu, value);
    return MOS6502_OK;
}

/* DEX - Decrement X register */
static mos6502_status_t op_dex(mos6502_t *cpu) {
    --cpu->x;
    set_flags_zn(cpu, cpu->x);
    return MOS6502_OK;
}

/* DEY - Decrement Y register */
static mos6502_status_t op_dey(mos6502_t *cpu) {
    --cpu->y;
    set_flags_zn(cpu, cpu->y);
    return MOS6502_OK;
}

/* EOR - Exclusive OR with accumulator */
static mos6502_status_t op_eor(mos6502_t *cpu) {
    cpu->a ^= read(cpu, cpu->mar);
    set_flags_zn(cpu, cpu->a);
    return MOS6502_OK;
}

/* INC - Increment memory */
static mos6502_status_t op_inc(mos6502_t *cpu) {
    uint8_t value = read(cpu, cpu->mar);
    write(cpu, cpu->mar, value);
    write(cpu, cpu->mar, ++value);
    set_flags_zn(cpu, value);
    return MOS6502_OK;
}

/* INX - Increment X register */
static mos6502_status_t op_inx(mos6502_t *cpu) {
    ++cpu->x;
    set_flags_zn(cpu, cpu->x);
    return MOS6502_OK;
}

/* INY - Increment Y register */
static mos6502_status_t op_iny(mos6502_t *cpu) {
    ++cpu->y;
    set_flags_zn(cpu, cpu->y);
    return MOS6502_OK;
}

/* JMP - Jump to new location */
static mos6502_status_t op_jmp(mos6502_t *cpu) {
    cpu->pc = cpu->mar;
    if (cpu->mar == cpu->iar) {
        return MOS6502_LOOP;
    } else {
        return MOS6502_OK;
    }
}

/* JSR - Jump to subroutine */
static mos6502_status_t op_jsr(mos6502_t *cpu) {
    cpu->pc++; /* The low byte has already been pre-fetched. */

    uint16_t address_lo = cpu->mar;

    read(cpu, on_stack(cpu->s));

    push(cpu, (uint8_t)((cpu->pc >> 8) & 0x00FF));
    push(cpu, (uint8_t)((cpu->pc >> 0) & 0x00FF));

    uint16_t address_hi = (uint16_t)(fetch(cpu) << 8);

    cpu->pc = (uint16_t)(address_hi | address_lo);
    return MOS6502_OK;
}

/* LDA - Load accumulator */
static mos6502_status_t op_lda(mos6502_t *cpu) {
    cpu->a = read(cpu, cpu->mar);
    set_flags_zn(cpu, cpu->a);
    return MOS6502_OK;
}

/* LDX - Load X register */
static mos6502_status_t op_ldx(mos6502_t *cpu) {
    cpu->x = read(cpu, cpu->mar);
    set_flags_zn(cpu, cpu->x);
    return MOS6502_OK;
}

/* LDY - Load Y register */
static mos6502_status_t op_ldy(mos6502_t *cpu) {
    cpu->y = read(cpu, cpu->mar);
    set_flags_zn(cpu, cpu->y);
    return MOS6502_OK;
}

/* LSR - Logical shift right */
static mos6502_status_t op_lsr(mos6502_t *cpu) {
    uint8_t value = read(cpu, cpu->mar);
    write(cpu, cpu->mar, value);
    set_flags(cpu, MOS6502_C, value & 1);
    write(cpu, cpu->mar, value >>= 1);
    set_flags_zn(cpu, value);
    return MOS6502_OK;
}

/* LSR - Logical shift right (on accumulator) */
static mos6502_status_t op_lsr_acc(mos6502_t *cpu) {
    set_flags(cpu, MOS6502_C, cpu->a & 1);
    cpu->a >>= 1;
    set_flags_zn(cpu, cpu->a);
    return MOS6502_OK;
}

/* NOP - No operation */
static mos6502_status_t op_nop(mos6502_t *cpu) {
    (void)cpu;
    return MOS6502_OK;
}

/* ORA - Logical OR with accumulator */
static mos6502_status_t op_ora(mos6502_t *cpu) {
    cpu->a |= read(cpu, cpu->mar);
    set_flags_zn(cpu, cpu->a);
    return MOS6502_OK;
}

/* PHA - Push accumulator onto stack */
static mos6502_status_t op_pha(mos6502_t *cpu) {
    push(cpu, cpu->a);
    return MOS6502_OK;
}

/* PHP - Push processor status on stack */
static mos6502_status_t op_php(mos6502_t *cpu) {
    push(cpu, cpu->p | MOS6502_B | MOS6502_U);
    return MOS6502_OK;
}

/* PLA - Pull accumulator from stack */
static mos6502_status_t op_pla(mos6502_t *cpu) {
    read(cpu, on_stack(cpu->s));
    cpu->a = pull(cpu);
    set_flags_zn(cpu, cpu->a);
    return MOS6502_OK;
}

/* PLP - Pull processor status from stack */
static mos6502_status_t op_plp(mos6502_t *cpu) {
    read(cpu, on_stack(cpu->s));
    cpu->p = (uint8_t)((pull(cpu) & ~MOS6502_B) | MOS6502_U);
    return MOS6502_OK;
}

/* ROL - Rotate left */
static mos6502_status_t op_rol(mos6502_t *cpu) {
    uint8_t value = read(cpu, cpu->mar);
    write(cpu, cpu->mar, value);

    uint8_t c = get_flags(cpu, MOS6502_C);
    set_flags(cpu, MOS6502_C, value >> 7);

    value = (uint8_t)((value << 1) | c);
    write(cpu, cpu->mar, value);

    set_flags_zn(cpu, value);
    return MOS6502_OK;
}

/* ROL - Rotate left (on accumulator) */
static mos6502_status_t op_rol_acc(mos6502_t *cpu) {
    uint8_t c = get_flags(cpu, MOS6502_C);
    set_flags(cpu, MOS6502_C, cpu->a >> 7);

    cpu->a = (uint8_t)((cpu->a << 1) | c);

    set_flags_zn(cpu, cpu->a);
    return MOS6502_OK;
}

/* ROR - Rotate right */
static mos6502_status_t op_ror(mos6502_t *cpu) {
    uint8_t value = read(cpu, cpu->mar);
    write(cpu, cpu->mar, value);

    uint8_t c = get_flags(cpu, MOS6502_C);
    set_flags(cpu, MOS6502_C, value & 1);

    value = (uint8_t)((c << 7) | (value >> 1));
    write(cpu, cpu->mar, value);

    set_flags_zn(cpu, value);
    return MOS6502_OK;
}

/* ROR - Rotate right (on accumulator) */
static mos6502_status_t op_ror_acc(mos6502_t *cpu) {
    uint8_t c = get_flags(cpu, MOS6502_C);
    set_flags(cpu, MOS6502_C, cpu->a & 1);

    cpu->a = (uint8_t)((c << 7) | (cpu->a >> 1));

    set_flags_zn(cpu, cpu->a);
    return MOS6502_OK;
}

/* RTI - Return from interrupt */
static mos6502_status_t op_rti(mos6502_t *cpu) {
    read(cpu, on_stack(cpu->s));

    cpu->p = (uint8_t)((pull(cpu) & ~MOS6502_B) | MOS6502_U);

    cpu->pc = 0x0000;
    cpu->pc |= pull(cpu) << 0;
    cpu->pc |= pull(cpu) << 8;

    return MOS6502_OK;
}

/* RTS - Return from subroutine */
static mos6502_status_t op_rts(mos6502_t *cpu) {
    read(cpu, on_stack(cpu->s));

    cpu->pc = 0x0000;
    cpu->pc |= pull(cpu) << 0;
    cpu->pc |= pull(cpu) << 8;

    fetch(cpu);

    return MOS6502_OK;
}

static mos6502_status_t sbc_bin(mos6502_t *cpu, uint8_t value) {
    return adc_bin(cpu, (uint8_t)~value);
}

static mos6502_status_t sbc_dec(mos6502_t *cpu, uint8_t value) {
    uint16_t a = (uint16_t)cpu->a;
    uint16_t b = (uint16_t)value;
    uint16_t c = (uint16_t)!get_flags(cpu, MOS6502_C);

    uint16_t result = (uint16_t)((a & 0x0F) - (b & 0x0F) - c);
    if (result & 0x10) {
        result = (uint16_t)(((result - 0x06) & 0x0F) | ((a & 0xF0) - (b & 0xF0) - 0x10));
    } else {
        result = (result & 0x0F) | ((a & 0xF0) - (b & 0xF0));
    }

    if (result & 0x100) {
        result -= 0x60;
    }

    cpu->a = (uint8_t)result;

    uint16_t tmp = (uint16_t)(a - b - c);
    set_flags(cpu, MOS6502_Z, (tmp & 0xFF) == 0);
    set_flags(cpu, MOS6502_N, (tmp & 0x80));
    set_flags(cpu, MOS6502_V, (tmp ^ a) & (a ^ b) & 0x80);
    set_flags(cpu, MOS6502_C, (tmp < 0x100));

    return MOS6502_OK;
}

/* SBC - Subtract memory from accumulator with borrow */
static mos6502_status_t op_sbc(mos6502_t *cpu) {
    uint8_t value = read(cpu, cpu->mar);
    if (cpu->has_bcd && get_flags(cpu, MOS6502_D)) {
        return sbc_dec(cpu, value);
    } else {
        return sbc_bin(cpu, value);
    }
}

/* SEC - Set carry flag */
static mos6502_status_t op_sec(mos6502_t *cpu) {
    set_flags(cpu, MOS6502_C, 1);
    return MOS6502_OK;
}

/* SED - Set decimal flag */
static mos6502_status_t op_sed(mos6502_t *cpu) {
    set_flags(cpu, MOS6502_D, 1);
    return MOS6502_OK;
}

/* SEI - Set interrupt disable */
static mos6502_status_t op_sei(mos6502_t *cpu) {
    set_flags(cpu, MOS6502_I, 1);
    return MOS6502_OK;
}

/* STA - Store accumulator in memory */
static mos6502_status_t op_sta(mos6502_t *cpu) {
    write(cpu, cpu->mar, cpu->a);
    return MOS6502_OK;
}

/* STX - Store X register in memory */
static mos6502_status_t op_stx(mos6502_t *cpu) {
    write(cpu, cpu->mar, cpu->x);
    return MOS6502_OK;
}

/* STY - Store Y register in memory */
static mos6502_status_t op_sty(mos6502_t *cpu) {
    write(cpu, cpu->mar, cpu->y);
    return MOS6502_OK;
}

/* TAX - Transfer accumulator to X */
static mos6502_status_t op_tax(mos6502_t *cpu) {
    cpu->x = cpu->a;
    set_flags_zn(cpu, cpu->x);
    return MOS6502_OK;
}

/* TAY - Transfer accumulator to Y */
static mos6502_status_t op_tay(mos6502_t *cpu) {
    cpu->y = cpu->a;
    set_flags_zn(cpu, cpu->y);
    return MOS6502_OK;
}

/* TSX - Transfer stack pointer to X */
static mos6502_status_t op_tsx(mos6502_t *cpu) {
    cpu->x = cpu->s;
    set_flags_zn(cpu, cpu->x);
    return MOS6502_OK;
}

/* TXA - Transfer X to accumulator */
static mos6502_status_t op_txa(mos6502_t *cpu) {
    cpu->a = cpu->x;
    set_flags_zn(cpu, cpu->a);
    return MOS6502_OK;
}

/* TXS - Transfer X to stack pointer */
static mos6502_status_t op_txs(mos6502_t *cpu) {
    cpu->s = cpu->x;
    return MOS6502_OK;
}

/* TYA - Transfer Y to accumulator */
static mos6502_status_t op_tya(mos6502_t *cpu) {
    cpu->a = cpu->y;
    set_flags_zn(cpu, cpu->a);
    return MOS6502_OK;
}

/* Unknown instruction */
static mos6502_status_t op_unk(mos6502_t *cpu) {
    (void)cpu;
    return MOS6502_UNKNOWN;
}

/* Driver ================================================================== */

mos6502_status_t mos6502_step(mos6502_t *cpu) {
    /* Every step is an instruction or an interrupt. */
    if (has_pending_interrupts(cpu)) {
        return handle_hardware_interrupt(cpu);
    }

    /* Store the address of the current instruction. */
    cpu->iar = cpu->pc;

    /* Fetch, decode, and execute the current instruction. */
    switch (cpu->ir = fetch(cpu)) {
        case 0x00: am_imp(cpu); return op_brk(cpu);
        case 0x01: am_idx(cpu); return op_ora(cpu);
        case 0x02: am_imp(cpu); return op_unk(cpu);
        case 0x03: am_imp(cpu); return op_unk(cpu);
        case 0x04: am_imp(cpu); return op_unk(cpu);
        case 0x05: am_zpg(cpu); return op_ora(cpu);
        case 0x06: am_zpg(cpu); return op_asl(cpu);
        case 0x07: am_imp(cpu); return op_unk(cpu);
        case 0x08: am_imp(cpu); return op_php(cpu);
        case 0x09: am_imm(cpu); return op_ora(cpu);
        case 0x0A: am_acc(cpu); return op_asl_acc(cpu);
        case 0x0B: am_imp(cpu); return op_unk(cpu);
        case 0x0C: am_imp(cpu); return op_unk(cpu);
        case 0x0D: am_abs(cpu); return op_ora(cpu);
        case 0x0E: am_abs(cpu); return op_asl(cpu);
        case 0x0F: am_imp(cpu); return op_unk(cpu);
        case 0x10: am_rel(cpu); return op_bpl(cpu);
        case 0x11: am_rdy(cpu); return op_ora(cpu);
        case 0x12: am_imp(cpu); return op_unk(cpu);
        case 0x13: am_imp(cpu); return op_unk(cpu);
        case 0x14: am_imp(cpu); return op_unk(cpu);
        case 0x15: am_zpx(cpu); return op_ora(cpu);
        case 0x16: am_zpx(cpu); return op_asl(cpu);
        case 0x17: am_imp(cpu); return op_unk(cpu);
        case 0x18: am_imp(cpu); return op_clc(cpu);
        case 0x19: am_rby(cpu); return op_ora(cpu);
        case 0x1A: am_imp(cpu); return op_unk(cpu);
        case 0x1B: am_imp(cpu); return op_unk(cpu);
        case 0x1C: am_imp(cpu); return op_unk(cpu);
        case 0x1D: am_rbx(cpu); return op_ora(cpu);
        case 0x1E: am_abx(cpu); return op_asl(cpu);
        case 0x1F: am_imp(cpu); return op_unk(cpu);
        case 0x20: am_pre(cpu); return op_jsr(cpu);
        case 0x21: am_idx(cpu); return op_and(cpu);
        case 0x22: am_imp(cpu); return op_unk(cpu);
        case 0x23: am_imp(cpu); return op_unk(cpu);
        case 0x24: am_zpg(cpu); return op_bit(cpu);
        case 0x25: am_zpg(cpu); return op_and(cpu);
        case 0x26: am_zpg(cpu); return op_rol(cpu);
        case 0x27: am_imp(cpu); return op_unk(cpu);
        case 0x28: am_imp(cpu); return op_plp(cpu);
        case 0x29: am_imm(cpu); return op_and(cpu);
        case 0x2A: am_acc(cpu); return op_rol_acc(cpu);
        case 0x2B: am_imp(cpu); return op_unk(cpu);
        case 0x2C: am_abs(cpu); return op_bit(cpu);
        case 0x2D: am_abs(cpu); return op_and(cpu);
        case 0x2E: am_abs(cpu); return op_rol(cpu);
        case 0x2F: am_imp(cpu); return op_unk(cpu);
        case 0x30: am_rel(cpu); return op_bmi(cpu);
        case 0x31: am_rdy(cpu); return op_and(cpu);
        case 0x32: am_imp(cpu); return op_unk(cpu);
        case 0x33: am_imp(cpu); return op_unk(cpu);
        case 0x34: am_imp(cpu); return op_unk(cpu);
        case 0x35: am_zpx(cpu); return op_and(cpu);
        case 0x36: am_zpx(cpu); return op_rol(cpu);
        case 0x37: am_imp(cpu); return op_unk(cpu);
        case 0x38: am_imp(cpu); return op_sec(cpu);
        case 0x39: am_rby(cpu); return op_and(cpu);
        case 0x3A: am_imp(cpu); return op_unk(cpu);
        case 0x3B: am_imp(cpu); return op_unk(cpu);
        case 0x3C: am_imp(cpu); return op_unk(cpu);
        case 0x3D: am_rbx(cpu); return op_and(cpu);
        case 0x3E: am_abx(cpu); return op_rol(cpu);
        case 0x3F: am_imp(cpu); return op_unk(cpu);
        case 0x40: am_imp(cpu); return op_rti(cpu);
        case 0x41: am_idx(cpu); return op_eor(cpu);
        case 0x42: am_imp(cpu); return op_unk(cpu);
        case 0x43: am_imp(cpu); return op_unk(cpu);
        case 0x44: am_imp(cpu); return op_unk(cpu);
        case 0x45: am_zpg(cpu); return op_eor(cpu);
        case 0x46: am_zpg(cpu); return op_lsr(cpu);
        case 0x47: am_imp(cpu); return op_unk(cpu);
        case 0x48: am_imp(cpu); return op_pha(cpu);
        case 0x49: am_imm(cpu); return op_eor(cpu);
        case 0x4A: am_acc(cpu); return op_lsr_acc(cpu);
        case 0x4B: am_imp(cpu); return op_unk(cpu);
        case 0x4C: am_abs(cpu); return op_jmp(cpu);
        case 0x4D: am_abs(cpu); return op_eor(cpu);
        case 0x4E: am_abs(cpu); return op_lsr(cpu);
        case 0x4F: am_imp(cpu); return op_unk(cpu);
        case 0x50: am_rel(cpu); return op_bvc(cpu);
        case 0x51: am_rdy(cpu); return op_eor(cpu);
        case 0x52: am_imp(cpu); return op_unk(cpu);
        case 0x53: am_imp(cpu); return op_unk(cpu);
        case 0x54: am_imp(cpu); return op_unk(cpu);
        case 0x55: am_zpx(cpu); return op_eor(cpu);
        case 0x56: am_zpx(cpu); return op_lsr(cpu);
        case 0x57: am_imp(cpu); return op_unk(cpu);
        case 0x58: am_imp(cpu); return op_cli(cpu);
        case 0x59: am_rby(cpu); return op_eor(cpu);
        case 0x5A: am_imp(cpu); return op_unk(cpu);
        case 0x5B: am_imp(cpu); return op_unk(cpu);
        case 0x5C: am_imp(cpu); return op_unk(cpu);
        case 0x5D: am_rbx(cpu); return op_eor(cpu);
        case 0x5E: am_abx(cpu); return op_lsr(cpu);
        case 0x5F: am_imp(cpu); return op_unk(cpu);
        case 0x60: am_imp(cpu); return op_rts(cpu);
        case 0x61: am_idx(cpu); return op_adc(cpu);
        case 0x62: am_imp(cpu); return op_unk(cpu);
        case 0x63: am_imp(cpu); return op_unk(cpu);
        case 0x64: am_imp(cpu); return op_unk(cpu);
        case 0x65: am_zpg(cpu); return op_adc(cpu);
        case 0x66: am_zpg(cpu); return op_ror(cpu);
        case 0x67: am_imp(cpu); return op_unk(cpu);
        case 0x68: am_imp(cpu); return op_pla(cpu);
        case 0x69: am_imm(cpu); return op_adc(cpu);
        case 0x6A: am_acc(cpu); return op_ror_acc(cpu);
        case 0x6B: am_imp(cpu); return op_unk(cpu);
        case 0x6C: am_ind(cpu); return op_jmp(cpu);
        case 0x6D: am_abs(cpu); return op_adc(cpu);
        case 0x6E: am_abs(cpu); return op_ror(cpu);
        case 0x6F: am_imp(cpu); return op_unk(cpu);
        case 0x70: am_rel(cpu); return op_bvs(cpu);
        case 0x71: am_rdy(cpu); return op_adc(cpu);
        case 0x72: am_imp(cpu); return op_unk(cpu);
        case 0x73: am_imp(cpu); return op_unk(cpu);
        case 0x74: am_imp(cpu); return op_unk(cpu);
        case 0x75: am_zpx(cpu); return op_adc(cpu);
        case 0x76: am_zpx(cpu); return op_ror(cpu);
        case 0x77: am_imp(cpu); return op_unk(cpu);
        case 0x78: am_imp(cpu); return op_sei(cpu);
        case 0x79: am_rby(cpu); return op_adc(cpu);
        case 0x7A: am_imp(cpu); return op_unk(cpu);
        case 0x7B: am_imp(cpu); return op_unk(cpu);
        case 0x7C: am_imp(cpu); return op_unk(cpu);
        case 0x7D: am_rbx(cpu); return op_adc(cpu);
        case 0x7E: am_abx(cpu); return op_ror(cpu);
        case 0x7F: am_imp(cpu); return op_unk(cpu);
        case 0x80: am_imp(cpu); return op_unk(cpu);
        case 0x81: am_idx(cpu); return op_sta(cpu);
        case 0x82: am_imp(cpu); return op_unk(cpu);
        case 0x83: am_imp(cpu); return op_unk(cpu);
        case 0x84: am_zpg(cpu); return op_sty(cpu);
        case 0x85: am_zpg(cpu); return op_sta(cpu);
        case 0x86: am_zpg(cpu); return op_stx(cpu);
        case 0x87: am_imp(cpu); return op_unk(cpu);
        case 0x88: am_imp(cpu); return op_dey(cpu);
        case 0x89: am_imp(cpu); return op_unk(cpu);
        case 0x8A: am_imp(cpu); return op_txa(cpu);
        case 0x8B: am_imp(cpu); return op_unk(cpu);
        case 0x8C: am_abs(cpu); return op_sty(cpu);
        case 0x8D: am_abs(cpu); return op_sta(cpu);
        case 0x8E: am_abs(cpu); return op_stx(cpu);
        case 0x8F: am_imp(cpu); return op_unk(cpu);
        case 0x90: am_rel(cpu); return op_bcc(cpu);
        case 0x91: am_idy(cpu); return op_sta(cpu);
        case 0x92: am_imp(cpu); return op_unk(cpu);
        case 0x93: am_imp(cpu); return op_unk(cpu);
        case 0x94: am_zpx(cpu); return op_sty(cpu);
        case 0x95: am_zpx(cpu); return op_sta(cpu);
        case 0x96: am_zpy(cpu); return op_stx(cpu);
        case 0x97: am_imp(cpu); return op_unk(cpu);
        case 0x98: am_imp(cpu); return op_tya(cpu);
        case 0x99: am_aby(cpu); return op_sta(cpu);
        case 0x9A: am_imp(cpu); return op_txs(cpu);
        case 0x9B: am_imp(cpu); return op_unk(cpu);
        case 0x9C: am_imp(cpu); return op_unk(cpu);
        case 0x9D: am_abx(cpu); return op_sta(cpu);
        case 0x9E: am_imp(cpu); return op_unk(cpu);
        case 0x9F: am_imp(cpu); return op_unk(cpu);
        case 0xA0: am_imm(cpu); return op_ldy(cpu);
        case 0xA1: am_idx(cpu); return op_lda(cpu);
        case 0xA2: am_imm(cpu); return op_ldx(cpu);
        case 0xA3: am_imp(cpu); return op_unk(cpu);
        case 0xA4: am_zpg(cpu); return op_ldy(cpu);
        case 0xA5: am_zpg(cpu); return op_lda(cpu);
        case 0xA6: am_zpg(cpu); return op_ldx(cpu);
        case 0xA7: am_imp(cpu); return op_unk(cpu);
        case 0xA8: am_imp(cpu); return op_tay(cpu);
        case 0xA9: am_imm(cpu); return op_lda(cpu);
        case 0xAA: am_imp(cpu); return op_tax(cpu);
        case 0xAB: am_imp(cpu); return op_unk(cpu);
        case 0xAC: am_abs(cpu); return op_ldy(cpu);
        case 0xAD: am_abs(cpu); return op_lda(cpu);
        case 0xAE: am_abs(cpu); return op_ldx(cpu);
        case 0xAF: am_imp(cpu); return op_unk(cpu);
        case 0xB0: am_rel(cpu); return op_bcs(cpu);
        case 0xB1: am_rdy(cpu); return op_lda(cpu);
        case 0xB2: am_imp(cpu); return op_unk(cpu);
        case 0xB3: am_imp(cpu); return op_unk(cpu);
        case 0xB4: am_zpx(cpu); return op_ldy(cpu);
        case 0xB5: am_zpx(cpu); return op_lda(cpu);
        case 0xB6: am_zpy(cpu); return op_ldx(cpu);
        case 0xB7: am_imp(cpu); return op_unk(cpu);
        case 0xB8: am_imp(cpu); return op_clv(cpu);
        case 0xB9: am_rby(cpu); return op_lda(cpu);
        case 0xBA: am_imp(cpu); return op_tsx(cpu);
        case 0xBB: am_imp(cpu); return op_unk(cpu);
        case 0xBC: am_rbx(cpu); return op_ldy(cpu);
        case 0xBD: am_rbx(cpu); return op_lda(cpu);
        case 0xBE: am_rby(cpu); return op_ldx(cpu);
        case 0xBF: am_imp(cpu); return op_unk(cpu);
        case 0xC0: am_imm(cpu); return op_cpy(cpu);
        case 0xC1: am_idx(cpu); return op_cmp(cpu);
        case 0xC2: am_imp(cpu); return op_unk(cpu);
        case 0xC3: am_imp(cpu); return op_unk(cpu);
        case 0xC4: am_zpg(cpu); return op_cpy(cpu);
        case 0xC5: am_zpg(cpu); return op_cmp(cpu);
        case 0xC6: am_zpg(cpu); return op_dec(cpu);
        case 0xC7: am_imp(cpu); return op_unk(cpu);
        case 0xC8: am_imp(cpu); return op_iny(cpu);
        case 0xC9: am_imm(cpu); return op_cmp(cpu);
        case 0xCA: am_imp(cpu); return op_dex(cpu);
        case 0xCB: am_imp(cpu); return op_unk(cpu);
        case 0xCC: am_abs(cpu); return op_cpy(cpu);
        case 0xCD: am_abs(cpu); return op_cmp(cpu);
        case 0xCE: am_abs(cpu); return op_dec(cpu);
        case 0xCF: am_imp(cpu); return op_unk(cpu);
        case 0xD0: am_rel(cpu); return op_bne(cpu);
        case 0xD1: am_rdy(cpu); return op_cmp(cpu);
        case 0xD2: am_imp(cpu); return op_unk(cpu);
        case 0xD3: am_imp(cpu); return op_unk(cpu);
        case 0xD4: am_imp(cpu); return op_unk(cpu);
        case 0xD5: am_zpx(cpu); return op_cmp(cpu);
        case 0xD6: am_zpx(cpu); return op_dec(cpu);
        case 0xD7: am_imp(cpu); return op_unk(cpu);
        case 0xD8: am_imp(cpu); return op_cld(cpu);
        case 0xD9: am_rby(cpu); return op_cmp(cpu);
        case 0xDA: am_imp(cpu); return op_unk(cpu);
        case 0xDB: am_imp(cpu); return op_unk(cpu);
        case 0xDC: am_imp(cpu); return op_unk(cpu);
        case 0xDD: am_rbx(cpu); return op_cmp(cpu);
        case 0xDE: am_abx(cpu); return op_dec(cpu);
        case 0xDF: am_imp(cpu); return op_unk(cpu);
        case 0xE0: am_imm(cpu); return op_cpx(cpu);
        case 0xE1: am_idx(cpu); return op_sbc(cpu);
        case 0xE2: am_imp(cpu); return op_unk(cpu);
        case 0xE3: am_imp(cpu); return op_unk(cpu);
        case 0xE4: am_zpg(cpu); return op_cpx(cpu);
        case 0xE5: am_zpg(cpu); return op_sbc(cpu);
        case 0xE6: am_zpg(cpu); return op_inc(cpu);
        case 0xE7: am_imp(cpu); return op_unk(cpu);
        case 0xE8: am_imp(cpu); return op_inx(cpu);
        case 0xE9: am_imm(cpu); return op_sbc(cpu);
        case 0xEA: am_imp(cpu); return op_nop(cpu);
        case 0xEB: am_imp(cpu); return op_unk(cpu);
        case 0xEC: am_abs(cpu); return op_cpx(cpu);
        case 0xED: am_abs(cpu); return op_sbc(cpu);
        case 0xEE: am_abs(cpu); return op_inc(cpu);
        case 0xEF: am_imp(cpu); return op_unk(cpu);
        case 0xF0: am_rel(cpu); return op_beq(cpu);
        case 0xF1: am_rdy(cpu); return op_sbc(cpu);
        case 0xF2: am_imp(cpu); return op_unk(cpu);
        case 0xF3: am_imp(cpu); return op_unk(cpu);
        case 0xF4: am_imp(cpu); return op_unk(cpu);
        case 0xF5: am_zpx(cpu); return op_sbc(cpu);
        case 0xF6: am_zpx(cpu); return op_inc(cpu);
        case 0xF7: am_imp(cpu); return op_unk(cpu);
        case 0xF8: am_imp(cpu); return op_sed(cpu);
        case 0xF9: am_rby(cpu); return op_sbc(cpu);
        case 0xFA: am_imp(cpu); return op_unk(cpu);
        case 0xFB: am_imp(cpu); return op_unk(cpu);
        case 0xFC: am_imp(cpu); return op_unk(cpu);
        case 0xFD: am_rbx(cpu); return op_sbc(cpu);
        case 0xFE: am_abx(cpu); return op_inc(cpu);
        case 0xFF: am_imp(cpu); return op_unk(cpu);
    }

    return MOS6502_UNKNOWN;
}

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
