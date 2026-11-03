// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 ANDI/ORI immediate-control legal-form oracle
//
// Motorola defines exactly three EE destinations for each instruction:
// MR, CCR, and OMR.  The reserved EE=3 boundary is covered by the illegal-
// instruction oracle; this companion matrix proves that tightening that
// boundary preserves all six legal forms and their architectural masks.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint8_t kStateWritebackFetch = 4;

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 128> p{};
    unsigned illegal_vector_fetches = 0;

    Harness(uint32_t opcode, uint32_t initial_sr, uint8_t initial_omr) {
        p[0] = opcode;
        p[1] = 0x0af080; // JMP >$0001 after the instruction retires.
        p[2] = 0x000001;

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
        debug_write(0x39, initial_sr);
        debug_write(0x3a, initial_omr);
        core.run = 1;
    }

    void debug_write(uint8_t address, uint32_t value) {
        core.dbg_reg_wr_addr = address;
        core.dbg_reg_wr_data = value;
        core.dbg_reg_wr_en = 1;
        tick();
        core.dbg_reg_wr_en = 0;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        if (fetch_en && (fetch_addr == 0x003e || fetch_addr == 0x003f))
            ++illegal_vector_fetches;
        core.clk = 1;
        core.eval();
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 0x7f] & 0x00ffffff;
    }

    bool retire(uint32_t opcode) {
        for (unsigned i = 0; i < 40; ++i) {
            if (core.state_out == kStateWritebackFetch &&
                core.cur_inst_out == opcode) {
                tick();
                return true;
            }
            tick();
        }
        return false;
    }
};

struct Case {
    const char *name;
    bool is_and;
    uint8_t target;
    uint8_t immediate;
    uint32_t initial_sr;
    uint8_t initial_omr;
    uint32_t expected_sr;
    bool expected_de;
};

bool run_case(const Case &test) {
    // DSP56001UMA3 instruction encoding table A-253:
    //   ANDI #xx,D = 00000000 iiiiiiii 101110EE
    //   ORI  #xx,D = 00000000 iiiiiiii 111110EE
    const uint32_t base = test.is_and ? 0x0000b8 : 0x0000f8;
    const uint32_t opcode = base | (uint32_t(test.immediate) << 8) |
                            test.target;
    Harness h(opcode, test.initial_sr, test.initial_omr);
    const bool retired = h.retire(opcode);
    const bool pass = retired && h.illegal_vector_fetches == 0 &&
                      h.core.sr_out == test.expected_sr &&
                      h.core.data_rom_enable == test.expected_de;
    std::printf("%s %s opcode=%06x SR=%06x/%06x DE=%u/%u III=%u retired=%u\n",
                pass ? "PASS" : "FAIL", test.name, opcode, h.core.sr_out,
                test.expected_sr, h.core.data_rom_enable, test.expected_de,
                h.illegal_vector_fetches, retired);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);

    constexpr uint32_t kInitialSr = 0x00a535;
    const std::array<Case, 6> cases{{
        {"ORI-MR",   false, 0, 0x0a, kInitialSr, 0x91, 0x00af35, false},
        {"ANDI-MR",  true,  0, 0xf0, kInitialSr, 0x91, 0x00a035, false},
        {"ORI-CCR",  false, 1, 0x4a, kInitialSr, 0x91, 0x00a57f, false},
        {"ANDI-CCR", true,  1, 0x2d, kInitialSr, 0x91, 0x00a525, false},
        {"ORI-OMR",  false, 2, 0x06, kInitialSr, 0x91, kInitialSr, true},
        {"ANDI-OMR", true,  2, 0xfb, kInitialSr, 0x95, kInitialSr, false},
    }};

    bool pass = true;
    for (const auto &test : cases)
        pass &= run_case(test);
    return pass ? 0 : 1;
}

