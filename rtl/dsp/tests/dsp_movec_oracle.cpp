// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 MOVEC short-immediate boundary oracle
//
// Motorola defines MOVE(C) #xx,D1 as a one-word instruction:
//
//     00000101 iiiiiiii 101ddddd
//
// The immediate byte is data, not an addressing-form selector.  This oracle
// instantiates the production core directly and proves both boundaries: no
// extension word is consumed for any immediate value, and the immediate is
// written to SR with reserved SR bit 7 forced to zero.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint8_t kStateDecode = 2;
constexpr uint8_t kStateWritebackFetch = 4;
constexpr uint8_t kStateLatchExtension = 6;
constexpr uint32_t kMovecShortImmediateSr = 0x0500b9;

class MovecHarness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 16> p{};
    unsigned x_reads = 0;
    unsigned x_writes = 0;
    unsigned y_reads = 0;
    unsigned y_writes = 0;

    explicit MovecHarness(uint8_t immediate) {
        p[0] = kMovecShortImmediateSr |
               (static_cast<uint32_t>(immediate) << 8);
        p[1] = 0x000000; // The next word must remain the next instruction.

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
        core.run = 1;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        x_reads += core.x_rd_en;
        x_writes += core.x_wr_en;
        y_reads += core.y_rd_en;
        y_writes += core.y_wr_en;
        core.clk = 1;
        core.eval();
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 0x0f] & 0x00ffffff;
    }

    void debug_write(uint8_t address, uint64_t data) {
        core.dbg_reg_wr_addr = address;
        core.dbg_reg_wr_data = data;
        core.dbg_reg_wr_en = 1;
        tick();
        core.dbg_reg_wr_en = 0;
    }

    void clear_bus_counts() {
        x_reads = x_writes = y_reads = y_writes = 0;
    }

    bool execute_first_instruction(bool &consumed_extension) {
        bool decoded = false;
        consumed_extension = false;
        for (unsigned i = 0; i < 20; ++i) {
            tick();
            if (core.state_out == kStateDecode)
                decoded = true;
            if (decoded && core.state_out == kStateLatchExtension)
                consumed_extension = true;
            if (decoded && core.state_out == kStateWritebackFetch) {
                // Architectural register updates occur on the rising edge
                // that leaves S_WB_FETCH, not on the edge that enters it.
                tick();
                return true;
            }
        }
        return false;
    }
};

bool check_immediate(uint8_t immediate) {
    MovecHarness h(immediate);
    bool consumed_extension = false;
    const bool completed = h.execute_first_instruction(consumed_extension);
    const uint32_t expected = immediate & 0x7f;
    const bool pass = completed && !consumed_extension &&
                      h.core.sr_out == expected;
    std::printf("%s MOVEC #$%02x,SR: SR=$%06x expected=$%06x "
                "extension=%u completed=%u\n",
                pass ? "PASS" : "FAIL", immediate, h.core.sr_out,
                expected, consumed_extension, completed);
    return pass;
}

bool check_full_accumulator(const char *name, uint32_t instruction,
                            uint32_t readback, uint8_t source_reg,
                            uint64_t source_value, uint64_t other_value,
                            uint32_t expected_readback, bool expect_limit) {
    MovecHarness h(0);
    h.core.run = 0;
    h.p[0] = instruction;
    h.p[1] = readback;
    h.p[2] = 0x0af080; // JMP >$0002.
    h.p[3] = 0x000002;

    constexpr uint32_t kInitialSr = 0x000001; // C=1, L=0, scaling=00.
    h.debug_write(0x39, kInitialSr);
    h.debug_write(0x0e, source_reg == 0x0f ? other_value : source_value);
    h.debug_write(0x0f, source_reg == 0x0f ? source_value : other_value);
    h.clear_bus_counts();
    h.core.run = 1;

    auto retire = [&](uint32_t opcode) {
        for (unsigned i = 0; i < 80; ++i) {
            if (h.core.state_out == kStateWritebackFetch &&
                h.core.cur_inst_out == opcode) {
                h.tick();
                return true;
            }
            h.tick();
        }
        return false;
    };

    const bool source_completed = retire(instruction);
    const uint32_t expected_sr = kInitialSr | (expect_limit ? 0x40 : 0);
    const uint64_t source_after = source_reg == 0x0f ? h.core.acc_b_out :
                                                        h.core.acc_a_out;
    const bool source_pass = source_completed &&
                             source_after == source_value &&
                             h.core.sr_out == expected_sr;
    const bool readback_completed = retire(readback);
    const uint32_t actual_readback = source_reg == 0x0f ?
        ((h.core.acc_a_out >> 24) & 0xffffff) :
        ((h.core.acc_b_out >> 24) & 0xffffff);
    const bool pass = source_pass && readback_completed &&
                      actual_readback == expected_readback &&
                      h.core.sr_out == expected_sr &&
                      h.x_reads == 0 && h.x_writes == 0 &&
                      h.y_reads == 0 && h.y_writes == 0;
    std::printf("%s MOVEC limit %s opcode=%06x readback-opcode=%06x "
                "value=%06x/%06x SR=%06x/%06x source=%014llx/%014llx "
                "completed=%u/%u bus=X%u/%u,Y%u/%u\n",
                pass ? "PASS" : "FAIL", name, instruction, readback,
                actual_readback, expected_readback, h.core.sr_out,
                expected_sr, static_cast<unsigned long long>(source_after),
                static_cast<unsigned long long>(source_value),
                source_completed, readback_completed, h.x_reads, h.x_writes,
                h.y_reads, h.y_writes);
    return pass;
}

bool check_control_to_register_bus_quiet() {
    constexpr uint32_t setup = 0x05f420;       // MOVEC #>imm,M0.
    constexpr uint32_t instruction = 0x04e08e; // MOVEC M0,A (W=1 form).
    constexpr uint32_t value = 0x001357;
    MovecHarness h(0);
    h.core.run = 0;
    h.p[0] = setup;
    h.p[1] = value;
    h.p[2] = instruction;
    h.p[3] = 0x0af080; // JMP >$0003.
    h.p[4] = 0x000003;
    h.core.run = 1;

    auto retire = [&](uint32_t opcode) {
        for (unsigned i = 0; i < 80; ++i) {
            if (h.core.state_out == kStateWritebackFetch &&
                h.core.cur_inst_out == opcode) {
                h.tick();
                return true;
            }
            h.tick();
        }
        return false;
    };

    const bool setup_completed = retire(setup);
    h.clear_bus_counts();
    const bool completed = retire(instruction);

    const uint32_t actual = (h.core.acc_a_out >> 24) & 0xffffff;
    const bool pass = setup_completed && completed && actual == value &&
                      h.x_reads == 0 && h.x_writes == 0 &&
                      h.y_reads == 0 && h.y_writes == 0;
    std::printf("%s MOVEC M0,A bus quiet: value=%06x/%06x "
                "completed=%u/%u bus=X%u/%u,Y%u/%u\n",
                pass ? "PASS" : "FAIL", actual, value, setup_completed,
                completed, h.x_reads, h.x_writes, h.y_reads, h.y_writes);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);

    // $30 has immediate bits 5:3=110, the effective-address code for an
    // extension word in other MOVEC forms.  $c3 and $ff prove immediate bit 7
    // is data while SR's architecturally reserved bit 7 still reads as zero.
    const bool zero = check_immediate(0x00);
    const bool ordinary = check_immediate(0x43);
    const bool ea_collision = check_immediate(0x30);
    const bool high = check_immediate(0xc3);
    const bool all = check_immediate(0xff);
    const bool acc_a = check_full_accumulator(
        "A-to-M0-positive", 0x04cea0, 0x044da0, 0x0e,
        0x01000000123456ULL, 0x0013579b2468acULL, 0x00ffff, true);
    const bool acc_b = check_full_accumulator(
        "B-to-M1-negative", 0x04cfa1, 0x044ca1, 0x0f,
        0xfeffffff654321ULL, 0x002468ac13579bULL, 0x000000, true);
    const bool a1 = check_full_accumulator(
        "A1-to-M2-control", 0x04cca2, 0x044da2, 0x0c,
        0x01123456abcdefULL, 0x0013579b2468acULL, 0x003456, false);
    const bool control_to_register = check_control_to_register_bus_quiet();
    return zero && ordinary && ea_collision && high && all &&
           acc_a && acc_b && a1 && control_to_register ? 0 : 1;
}

