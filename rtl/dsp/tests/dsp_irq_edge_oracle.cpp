// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 external-interrupt edge-mode boundary oracle
//
// This instantiates the production core directly.  It supplies a tiny P-space
// program containing the architectural IRQA/IRQB long-vector slots and RTI
// handlers, then observes the core's real fetch bus.  No interrupt behavior is
// modeled in the harness.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint8_t kStateDecode = 2;
constexpr uint16_t kIrqaVector = 0x0008;
constexpr uint16_t kIrqbVector = 0x000a;

class IrqHarness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};
    uint64_t cycles = 0;
    unsigned irqa_vector_fetches = 0;
    unsigned irqa_second_fetches = 0;
    unsigned irqb_vector_fetches = 0;
    unsigned irqb_second_fetches = 0;
    unsigned handler20_fetches = 0;
    bool handler20_masked = false;
    std::array<uint16_t, 8> vector_order{};
    unsigned vector_order_count = 0;

    IrqHarness() {
        // Ordinary execution loops through NOP and JMP >$0000.
        p[0x00] = 0x000000;
        p[0x01] = 0x0af080;
        p[0x02] = 0x000000;

        // DSP56001 long-interrupt vector pairs.
        p[0x08] = 0x0bf080; // IRQA: JSR >$0020
        p[0x09] = 0x000020;
        p[0x0a] = 0x0bf080; // IRQB: JSR >$0030
        p[0x0b] = 0x000030;
        p[0x20] = 0x000004; // RTI
        p[0x30] = 0x000004; // RTI

        core.clk = 0;
        core.rst_n = 0;
        core.clock_enable = 1;
        core.halt = 0;
        core.run = 0;
        core.p_fetch_data = 0;
        core.p_fetch_ready = 1;
        core.x_rdata = 0;
        core.x_ready = 1;
        core.y_rdata = 0;
        core.y_ready = 1;
        core.p_data_rdata = 0;
        core.irq_a = 0;
        core.irq_b = 0;
        core.irq_ssi_tx = 0;
        core.irq_ssi_rx = 0;
        core.irq_ssi_tx_exception = 0;
        core.irq_ssi_rx_exception = 0;
        core.irq_sci_tx = 0;
        core.irq_sci_rx = 0;
        core.irq_sci_timer = 0;
        core.irq_sci_rx_exception = 0;
        core.irq_sci_idle = 0;
        core.irq_priority = 0;
        core.dbg_reg_wr_en = 0;
        core.dbg_reg_wr_addr = 0;
        core.dbg_reg_wr_data = 0;

        for (unsigned i = 0; i < 4; ++i)
            tick();
        core.rst_n = 1;

        // Reset SR masks all external interrupts.  Set it to zero before run
        // through the production debug register boundary used by other core
        // oracles; this does not alter interrupt-controller state.
        core.dbg_reg_wr_en = 1;
        core.dbg_reg_wr_addr = 0x39;
        core.dbg_reg_wr_data = 0;
        tick();
        core.dbg_reg_wr_en = 0;
        core.run = 1;
    }

    void tick() {
        // The product memory response is registered and remains stable until
        // the next read.  Capture the request before the rising edge, advance
        // the core, then present that response for the following cycle.
        core.clk = 0;
        core.eval();
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        if (fetch_en) {
            switch (fetch_addr) {
            case 0x0008:
                ++irqa_vector_fetches;
                if (vector_order_count < vector_order.size())
                    vector_order[vector_order_count++] = fetch_addr;
                break;
            case 0x0009: ++irqa_second_fetches; break;
            case 0x000a:
                ++irqb_vector_fetches;
                if (vector_order_count < vector_order.size())
                    vector_order[vector_order_count++] = fetch_addr;
                break;
            case 0x000b: ++irqb_second_fetches; break;
            case 0x0020: ++handler20_fetches; break;
            default: break;
            }
        }
        core.clk = 1;
        core.eval();
        if (fetch_en && fetch_addr == 0x0020 &&
            ((core.sr_out >> 8) & 0x3) == 0x3)
            handler20_masked = true;
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 0xff] & 0x00ffffff;
        ++cycles;
    }

    bool wait_state(uint8_t state, unsigned bound = 200) {
        for (unsigned i = 0; i < bound; ++i) {
            if (core.state_out == state)
                return true;
            tick();
        }
        return false;
    }

    void pulse_a_between_boundaries() {
        wait_state(kStateDecode);
        core.irq_a = 1;
        tick();
        core.irq_a = 0;
    }

    void pulse_b_between_boundaries() {
        wait_state(kStateDecode);
        core.irq_b = 1;
        tick();
        core.irq_b = 0;
    }

    void pulse_both_between_boundaries() {
        wait_state(kStateDecode);
        core.irq_a = 1;
        core.irq_b = 1;
        tick();
        core.irq_a = 0;
        core.irq_b = 0;
    }

    void run_cycles(unsigned count) {
        for (unsigned i = 0; i < count; ++i)
            tick();
    }
};

bool irqa_pulse_is_retained() {
    IrqHarness h;
    h.core.irq_priority = 0x07; // IRQA edge mode, level code 3
    h.pulse_a_between_boundaries();
    h.run_cycles(80);
    const bool pass = h.irqa_vector_fetches == 1 &&
                      h.irqa_second_fetches == 1;
    std::printf("%s IRQA edge pulse retained: vector=%u second=%u\n",
                pass ? "PASS" : "FAIL", h.irqa_vector_fetches,
                h.irqa_second_fetches);
    return pass;
}

bool irqb_pulse_is_retained() {
    IrqHarness h;
    h.core.irq_priority = 0x38; // IRQB edge mode, level code 3
    h.pulse_b_between_boundaries();
    h.run_cycles(80);
    const bool pass = h.irqb_vector_fetches == 1 &&
                      h.irqb_second_fetches == 1;
    std::printf("%s IRQB edge pulse retained: vector=%u second=%u\n",
                pass ? "PASS" : "FAIL", h.irqb_vector_fetches,
                h.irqb_second_fetches);
    return pass;
}

bool higher_ipl_wins_simultaneous_edges() {
    IrqHarness h;
    // IRQA is edge-triggered IPL0 (code 1); IRQB is edge-triggered IPL2
    // (code 3).  Motorola requires the highest unmasked IPL to win each
    // arbitration.  The losing edge must remain pending and run after RTI.
    h.core.irq_priority = 0x3d;
    h.pulse_both_between_boundaries();
    h.run_cycles(160);
    const bool pass = h.vector_order_count == 2 &&
                      h.vector_order[0] == kIrqbVector &&
                      h.vector_order[1] == kIrqaVector &&
                      h.irqb_vector_fetches == 1 &&
                      h.irqb_second_fetches == 1 &&
                      h.irqa_vector_fetches == 1 &&
                      h.irqa_second_fetches == 1;
    std::printf("%s simultaneous external IRQ priority: order=%04x,%04x "
                "A=%u/%u B=%u/%u\n", pass ? "PASS" : "FAIL",
                h.vector_order_count > 0 ? h.vector_order[0] : 0xffff,
                h.vector_order_count > 1 ? h.vector_order[1] : 0xffff,
                h.irqa_vector_fetches, h.irqa_second_fetches,
                h.irqb_vector_fetches, h.irqb_second_fetches);
    return pass;
}

bool fast_pair_executes_and_returns_implicitly() {
    IrqHarness h;

    // A fast interrupt is exactly two one-word vector instructions when
    // neither slot contains JSR.  The PC and SR are not stacked or changed,
    // the pair is uninterruptible, and ordinary fetching resumes implicitly
    // after slot two.  Use independently visible accumulator writes so a
    // mere vector-bus fetch cannot satisfy the oracle.
    h.p[kIrqaVector] = 0x2e1100;     // MOVE #$11,A
    h.p[kIrqaVector + 1] = 0x2f2200; // MOVE #$22,B
    h.core.irq_priority = 0x07;      // IRQA edge mode, IPL2
    h.pulse_a_between_boundaries();
    h.run_cycles(100);

    const uint64_t acc_a = h.core.acc_a_out;
    const uint64_t acc_b = h.core.acc_b_out;
    const bool pass = h.irqa_vector_fetches == 1 &&
                      h.irqa_second_fetches == 1 &&
                      acc_a == 0x00110000000000ULL &&
                      acc_b == 0x00220000000000ULL &&
                      h.core.sr_out == 0;
    std::printf("%s fast external IRQ pair: vector=%u second=%u "
                "A=%014llx B=%014llx SR=%06x\n",
                pass ? "PASS" : "FAIL", h.irqa_vector_fetches,
                h.irqa_second_fetches,
                static_cast<unsigned long long>(acc_a),
                static_cast<unsigned long long>(acc_b), h.core.sr_out);
    return pass;
}

bool single_two_word_fast_instruction_executes() {
    IrqHarness h;
    h.p[kIrqaVector] = 0x5ef400;     // MOVE #>$123456,A
    h.p[kIrqaVector + 1] = 0x123456; // extension occupies slot two
    h.core.irq_priority = 0x07;
    h.pulse_a_between_boundaries();
    h.run_cycles(100);

    const uint64_t acc_a = h.core.acc_a_out;
    const bool pass = h.irqa_vector_fetches == 1 &&
                      h.irqa_second_fetches == 1 &&
                      acc_a == 0x123456000000ULL &&
                      h.core.sr_out == 0;
    std::printf("%s two-word fast external IRQ: vector=%u second=%u "
                "A=%014llx SR=%06x\n", pass ? "PASS" : "FAIL",
                h.irqa_vector_fetches, h.irqa_second_fetches,
                static_cast<unsigned long long>(acc_a), h.core.sr_out);
    return pass;
}

bool jsr_in_second_slot_forms_long_interrupt() {
    IrqHarness h;
    // Seed R6 in ordinary code.  A MOVE to R6 in slot one followed directly
    // by JSR (R6) in slot two would architecturally use the old pointer value
    // under DSP56001UMA3 A.8.6, and therefore is not a valid long-interrupt
    // vector sequence.
    h.p[0x00] = 0x362000;            // MOVE #$20,R6
    h.p[0x01] = 0x0af080;            // JMP >$0001
    h.p[0x02] = 0x000001;
    h.p[kIrqaVector] = 0x000000;     // NOP
    h.p[kIrqaVector + 1] = 0x0be680; // JSR (R6)
    h.run_cycles(20);
    h.core.irq_priority = 0x07;
    h.pulse_a_between_boundaries();
    h.run_cycles(120);

    const bool pass = h.irqa_vector_fetches == 1 &&
                      h.irqa_second_fetches == 1 &&
                      h.handler20_fetches == 1 &&
                      h.handler20_masked && h.core.sr_out == 0;
    std::printf("%s fast slot-two JSR: vector=%u second=%u handler=%u "
                "masked=%u SR=%06x\n", pass ? "PASS" : "FAIL",
                h.irqa_vector_fetches, h.irqa_second_fetches,
                h.handler20_fetches, h.handler20_masked, h.core.sr_out);
    return pass;
}

bool jsr_in_first_slot_discards_second_instruction() {
    IrqHarness h;

    // Seed R6 from ordinary code before accepting the interrupt.  Slot one
    // then changes flow.  Slot two must still be fetched to complete/ack the
    // service, but its accumulator write must never execute.
    h.p[0x00] = 0x362000;            // MOVE #$20,R6
    h.p[0x01] = 0x0af080;            // JMP >$0001
    h.p[0x02] = 0x000001;
    h.p[kIrqaVector] = 0x0be680;     // JSR (R6)
    h.p[kIrqaVector + 1] = 0x2f2200; // MOVE #$22,B -- fetched, discarded
    h.run_cycles(20);
    h.core.irq_priority = 0x07;
    h.pulse_a_between_boundaries();
    h.run_cycles(120);

    const uint64_t acc_b = h.core.acc_b_out;
    const bool pass = h.irqa_vector_fetches == 1 &&
                      h.irqa_second_fetches == 1 &&
                      h.handler20_fetches == 1 &&
                      h.handler20_masked && acc_b == 0 &&
                      h.core.sr_out == 0;
    std::printf("%s fast slot-one JSR discard: vector=%u second=%u "
                "handler=%u masked=%u B=%014llx SR=%06x\n",
                pass ? "PASS" : "FAIL", h.irqa_vector_fetches,
                h.irqa_second_fetches, h.handler20_fetches,
                h.handler20_masked,
                static_cast<unsigned long long>(acc_b), h.core.sr_out);
    return pass;
}

bool fast_pair_is_uninterruptible() {
    IrqHarness h;
    h.p[kIrqaVector] = 0x2e1100;     // MOVE #$11,A
    h.p[kIrqaVector + 1] = 0x2f2200; // MOVE #$22,B
    h.core.irq_priority = 0x3d;      // A edge IPL0, B edge IPL2
    h.pulse_a_between_boundaries();
    for (unsigned i = 0; i < 40 && h.irqa_vector_fetches == 0; ++i)
        h.tick();

    // Assert higher-priority IRQB after slot one has been fetched.  It must
    // remain pending until both fast instructions finish.
    h.core.irq_b = 1;
    h.tick();
    h.core.irq_b = 0;
    h.run_cycles(160);

    const bool pass = h.vector_order_count == 2 &&
                      h.vector_order[0] == kIrqaVector &&
                      h.vector_order[1] == kIrqbVector &&
                      h.irqa_vector_fetches == 1 &&
                      h.irqa_second_fetches == 1 &&
                      h.irqb_vector_fetches == 1 &&
                      h.irqb_second_fetches == 1;
    std::printf("%s fast external IRQ uninterruptible: order=%04x,%04x "
                "A=%u/%u B=%u/%u\n", pass ? "PASS" : "FAIL",
                h.vector_order_count > 0 ? h.vector_order[0] : 0xffff,
                h.vector_order_count > 1 ? h.vector_order[1] : 0xffff,
                h.irqa_vector_fetches, h.irqa_second_fetches,
                h.irqb_vector_fetches, h.irqb_second_fetches);
    return pass;
}

bool held_edge_is_one_shot_and_rearms() {
    IrqHarness h;
    h.core.irq_priority = 0x07;
    h.wait_state(kStateDecode);
    h.core.irq_a = 1;
    h.run_cycles(100);
    const bool one_shot = h.irqa_vector_fetches == 1 &&
                          h.irqa_second_fetches == 1;

    // A sampled inactive interval re-arms the edge detector.  The next
    // assertion must produce exactly one new vector service.
    h.core.irq_a = 0;
    h.run_cycles(4);
    h.wait_state(kStateDecode);
    h.core.irq_a = 1;
    h.run_cycles(80);
    const bool rearmed = h.irqa_vector_fetches == 2 &&
                         h.irqa_second_fetches == 2;
    const bool pass = one_shot && rearmed;
    std::printf("%s held IRQA edge one-shot/re-arm: one_shot=%u "
                "vector=%u second=%u\n", pass ? "PASS" : "FAIL",
                one_shot, h.irqa_vector_fetches, h.irqa_second_fetches);
    return pass;
}

bool disabled_edge_is_discarded() {
    IrqHarness h;
    h.core.irq_priority = 0x04; // IRQA edge mode, disabled
    h.pulse_a_between_boundaries();
    h.run_cycles(10);
    h.core.irq_priority = 0x07;
    h.run_cycles(60);
    const bool pass = h.irqa_vector_fetches == 0 &&
                      h.irqa_second_fetches == 0;
    std::printf("%s disabled IRQA edge discarded: vector=%u second=%u\n",
                pass ? "PASS" : "FAIL", h.irqa_vector_fetches,
                h.irqa_second_fetches);
    return pass;
}

bool level_mode_remains_level_sensitive() {
    IrqHarness h;
    h.core.irq_priority = 0x03; // IRQA level mode, level code 3

    // A level that disappears before an instruction boundary is not latched.
    h.pulse_a_between_boundaries();
    h.run_cycles(40);
    const bool short_level_ignored = h.irqa_vector_fetches == 0;

    // A held level remains pending after RTI and therefore services again.
    h.wait_state(kStateDecode);
    h.core.irq_a = 1;
    h.run_cycles(100);
    const bool held_level_retriggers = h.irqa_vector_fetches >= 2;
    h.core.irq_a = 0;
    const bool pass = short_level_ignored && held_level_retriggers;
    std::printf("%s IRQA level control: short_ignored=%u held_vectors=%u\n",
                pass ? "PASS" : "FAIL", short_level_ignored,
                h.irqa_vector_fetches);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const bool a_pulse = irqa_pulse_is_retained();
    const bool b_pulse = irqb_pulse_is_retained();
    const bool simultaneous = higher_ipl_wins_simultaneous_edges();
    const bool fast_pair = fast_pair_executes_and_returns_implicitly();
    const bool fast_two_word = single_two_word_fast_instruction_executes();
    const bool jsr_second = jsr_in_second_slot_forms_long_interrupt();
    const bool jsr_first = jsr_in_first_slot_discards_second_instruction();
    const bool fast_atomic = fast_pair_is_uninterruptible();
    const bool held = held_edge_is_one_shot_and_rearms();
    const bool disabled = disabled_edge_is_discarded();
    const bool level = level_mode_remains_level_sensitive();
    return a_pulse && b_pulse && simultaneous && fast_pair && fast_two_word &&
           jsr_second && jsr_first && fast_atomic && held && disabled &&
           level ? 0 : 1;
}

