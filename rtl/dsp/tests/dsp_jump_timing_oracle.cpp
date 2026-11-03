// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 production-core jump and bit-jump timing oracle
//
// DSP56001UMA3 Tables A-6, A-10, A-11, and A-13 define JMP/Jcc/JSR/JScc
// as four clocks plus the effective-address term and JCLR/JSET/JSCLR/JSSET
// as six clocks plus that term.  In zero-wait memory, indexed and
// predecrement addressing add two clocks.  Every bit-jump carries its branch
// target in an extension word; its tested register or X/Y operand must remain
// otherwise unchanged.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "Vdsp56001_core___024root.h"
#include "verilated.h"

namespace {

constexpr uint32_t kTargetMarker = 0x366600;      // MOVE #<$66,R6.
constexpr uint32_t kFallthroughMarker = 0x375500; // MOVE #<$55,R7.

struct Case {
    const char *name;
    uint32_t instruction;
    bool two_word;
    bool bit_jump;
    bool memory;
    bool y_space;
    bool call;
    bool expect_taken;
    bool source_bit;
    uint32_t initial_sr;
    uint16_t source_address;
    uint16_t target;
    uint16_t expected_r0;
    unsigned expected_clocks;
};

constexpr std::array<Case, 20> kCases{{
    {"JMP-short",       0x0c0020, false, false, false, false, false, true,
     false, 0, 0,      0x0020, 0x0040, 4},
    {"JMP-direct",      0x0ae080, false, false, false, false, false, true,
     false, 0, 0,      0x0040, 0x0040, 4},
    {"JMP-indexed",     0x0ae880, false, false, false, false, false, true,
     false, 0, 0,      0x0043, 0x0040, 6},
    {"JMP-predecrement",0x0af880, false, false, false, false, false, true,
     false, 0, 0,      0x003f, 0x003f, 6},
    {"JMP-absolute",    0x0af080, true,  false, false, false, false, true,
     false, 0, 0,      0x0020, 0x0040, 6},
    {"JEQ-short-taken", 0x0ea020, false, false, false, false, false, true,
     false, 4, 0,      0x0020, 0x0040, 4},
    {"JEQ-short-skip",  0x0ea020, false, false, false, false, false, false,
     false, 0, 0,      0x0020, 0x0040, 4},
    {"JSR-direct",      0x0be080, false, false, false, false, true,  true,
     false, 0, 0,      0x0040, 0x0040, 4},
    {"JSR-absolute",    0x0bf080, true,  false, false, false, true,  true,
     false, 0, 0,      0x0020, 0x0040, 6},
    {"JSEQ-short-taken",0x0fa020, false, false, false, false, true,  true,
     false, 4, 0,      0x0020, 0x0040, 4},
    {"JSEQ-short-skip", 0x0fa020, false, false, false, false, true,  false,
     false, 0, 0,      0x0020, 0x0040, 4},

    {"JCLR-register-taken", 0x0ac400, true, true, false, false, false, true,
     false, 0, 0,      0x0020, 0x0040, 6},
    {"JCLR-register-skip",  0x0ac400, true, true, false, false, false, false,
     true,  0, 0,      0x0020, 0x0040, 6},
    {"JSSET-register-taken",0x0bc420, true, true, false, false, true,  true,
     true,  0, 0,      0x0020, 0x0040, 6},
    {"JSSET-register-skip", 0x0bc420, true, true, false, false, true,  false,
     false, 0, 0,      0x0020, 0x0040, 6},
    {"JCLR-short-X",        0x0a1280, true, true, true,  false, false, true,
     false, 0, 0x0012, 0x0020, 0x0040, 6},
    {"JSET-direct-X",       0x0a60a0, true, true, true,  false, false, true,
     true,  0, 0x0040, 0x0020, 0x0040, 6},
    {"JSCLR-indexed-X",     0x0b6880, true, true, true,  false, true,  true,
     false, 0, 0x0043, 0x0020, 0x0040, 8},
    {"JSSET-predecrement-X",0x0b78a0, true, true, true,  false, true,  true,
     true,  0, 0x003f, 0x0020, 0x003f, 8},
    {"JCLR-peripheral-X",   0x0a8280, true, true, true,  false, false, true,
     false, 0, 0xffc2, 0x0020, 0x0040, 6},
}};

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};
    std::array<uint32_t, 65536> x{};
    std::array<uint32_t, 65536> y{};
    uint32_t instruction;
    uint64_t steps = 0;
    int64_t decode_step = -1, writeback_step = -1, successor_step = -1;
    bool target_marker = false;
    unsigned x_reads = 0, x_writes = 0, y_reads = 0, y_writes = 0;
    uint16_t read_address = 0;

    explicit Harness(const Case &test) : instruction(test.instruction) {
        p[0] = 0x60f400; // MOVE #>$40,R0.
        p[1] = 0x000040;
        p[2] = 0x380300; // MOVE #<3,N0.
        p[3] = 0x000000; // Address-register pipeline spacing.
        p[4] = instruction;
        const uint16_t fallthrough = test.two_word ? 6 : 5;
        if (test.two_word) p[5] = test.target;
        p[fallthrough] = kFallthroughMarker;
        p[fallthrough + 1] = 0x0c0007;
        p[test.target] = kTargetMarker;
        p[test.target + 1] = 0x0c0021;
        x[test.source_address] = test.source_bit ? 1 : 0;
        y[test.source_address] = test.source_bit ? 1 : 0;

        core.clk = 0; core.rst_n = 0; core.clock_enable = 1;
        core.halt = 0; core.run = 0; core.p_fetch_data = 0;
        core.p_fetch_ready = 1; core.x_rdata = core.y_rdata = 0;
        core.p_data_rdata = 0; core.x_ready = core.y_ready = 1;
        core.irq_a = core.irq_b = 0;
        core.irq_ssi_tx = core.irq_ssi_rx = 0;
        core.irq_ssi_tx_exception = core.irq_ssi_rx_exception = 0;
        core.irq_sci_tx = core.irq_sci_rx = 0;
        core.irq_sci_rx_exception = core.irq_sci_idle = 0;
        core.irq_sci_timer = 0; core.irq_priority = 0;
        core.dbg_reg_wr_en = 0; core.dbg_reg_wr_addr = 0;
        core.dbg_reg_wr_data = 0;
        for (unsigned i = 0; i < 4; ++i) tick();
        core.rst_n = 1;

        core.dbg_reg_wr_en = 1;
        core.dbg_reg_wr_addr = 0x04; // X0 bit-jump source.
        core.dbg_reg_wr_data = test.source_bit ? 1 : 0;
        tick();
        core.dbg_reg_wr_addr = 0x39; // SR, including Jcc condition inputs.
        core.dbg_reg_wr_data = test.initial_sr;
        tick();
        core.dbg_reg_wr_en = 0;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch = core.p_fetch_en;
        const uint16_t fetch_address = core.p_fetch_addr;
        const bool xr = core.x_rd_en, xw = core.x_wr_en;
        const bool yr = core.y_rd_en, yw = core.y_wr_en;
        const uint16_t xa = core.x_addr, ya = core.y_addr;
        if (core.state_out == 2 &&
            (core.p_fetch_data & 0x00ffffff) == instruction && decode_step < 0)
            decode_step = static_cast<int64_t>(steps);
        if (core.state_out == 4 && core.cur_inst_out == instruction &&
            writeback_step < 0)
            writeback_step = static_cast<int64_t>(steps);
        if (core.state_out == 4 &&
            (core.cur_inst_out == kTargetMarker ||
             core.cur_inst_out == kFallthroughMarker) && successor_step < 0) {
            successor_step = static_cast<int64_t>(steps);
            target_marker = core.cur_inst_out == kTargetMarker;
        }
        core.clk = 1;
        core.eval();
        if (fetch) core.p_fetch_data = p[fetch_address & 0xff] & 0x00ffffff;
        if (xr) { ++x_reads; read_address = xa; core.x_rdata = x[xa]; }
        if (xw) { ++x_writes; x[xa] = core.x_wdata & 0x00ffffff; }
        if (yr) { ++y_reads; read_address = ya; core.y_rdata = y[ya]; }
        if (yw) { ++y_writes; y[ya] = core.y_wdata & 0x00ffffff; }
        ++steps;
    }
};

bool run_case(const Case &test) {
    Harness h(test);
    h.core.run = 1;
    for (unsigned i = 0; i < 160 && h.successor_step < 0; ++i) h.tick();

    const int64_t clocks = h.decode_step >= 0 && h.writeback_step >= 0 ?
        h.writeback_step - h.decode_step + 1 : -1;
    const int64_t gap = h.writeback_step >= 0 && h.successor_step >= 0 ?
        h.successor_step - h.writeback_step : -1;
    const auto *root = h.core.rootp;
    const uint64_t ss0 = root->dsp56001_core__DOT__ss[0];
    const uint16_t return_pc = test.two_word ? 6 : 5;
    const bool stack_expected = test.call && test.expect_taken;
    const bool stack_ok = root->dsp56001_core__DOT__sp ==
                              (stack_expected ? 1 : 0) &&
        (!stack_expected ||
         (((ss0 >> 24) & 0xffff) == return_pc &&
          (ss0 & 0x00ffffff) == test.initial_sr));
    const bool memory_ok = test.memory ?
        ((test.y_space ? h.y_reads : h.x_reads) == 1 &&
         (test.y_space ? h.x_reads : h.y_reads) == 0 &&
         h.x_writes == 0 && h.y_writes == 0 &&
         h.read_address == test.source_address) :
        (h.x_reads == 0 && h.x_writes == 0 &&
         h.y_reads == 0 && h.y_writes == 0);
    const bool effect_ok = h.target_marker == test.expect_taken && stack_ok &&
        memory_ok && root->dsp56001_core__DOT__r_reg[0] == test.expected_r0 &&
        h.core.sr_out == test.initial_sr &&
        (!test.bit_jump ||
         (root->dsp56001_core__DOT__x0_reg & 1) == test.source_bit);
    const bool pass = clocks == test.expected_clocks && gap == 2 && effect_ok;
    std::printf("%s JUMP timing %-23s clocks=%lld/%u gap=%lld/2 "
                "taken=%u/%u SP=%u SS0=%012llx R0=%04x/%04x "
                "XY=%u/%u/%u/%u read=%04x SR=%06x effect=%u\n",
                pass ? "PASS" : "FAIL", test.name,
                static_cast<long long>(clocks), test.expected_clocks,
                static_cast<long long>(gap), h.target_marker,
                test.expect_taken, root->dsp56001_core__DOT__sp,
                static_cast<unsigned long long>(ss0),
                root->dsp56001_core__DOT__r_reg[0], test.expected_r0,
                h.x_reads, h.x_writes, h.y_reads, h.y_writes,
                h.read_address, h.core.sr_out, effect_ok);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    unsigned failures = 0;
    for (const auto &test : kCases) failures += !run_case(test);
    std::printf("JUMP TIMING SUMMARY cases=%zu failures=%u\n",
                kCases.size(), failures);
    return failures ? 1 : 0;
}

