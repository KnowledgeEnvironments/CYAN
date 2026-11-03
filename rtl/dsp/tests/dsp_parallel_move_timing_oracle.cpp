// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 parallel-MOVE production-core timing oracle
//
// DSP56001UMA3 Tables A-6, A-7, A-13, and A-14 define the zero-wait-state
// timing used here.  A parallel ALU/MOVE instruction has a two-clock base.
// Ordinary/short X or Y moves add no clocks; indexed and predecrement modes
// add two; immediate and absolute extension forms add two.  L: or X:Y with
// both external memories adds two more clocks for the shared external bus.
//
// Every memory-store case also requires the exact single X/Y write count so
// a timing-only no-op cannot pass.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint32_t kSuccessor = 0x375500; // MOVE #<$55,R7.

struct Case {
    const char *name;
    uint32_t instruction;
    uint32_t extension;
    bool two_word;
    unsigned expected_clocks;
    unsigned expected_x_writes;
    unsigned expected_y_writes;
};

constexpr std::array<Case, 16> kCases{{
    {"no-parallel-move",       0x200000, 0,        false, 2, 0, 0},
    {"Y-short-store",          0x6f3200, 0,        false, 2, 0, 1},
    {"X:R-direct-store",       0x102000, 0,        false, 2, 1, 0},
    {"X:R-postinc-N-store",    0x100800, 0,        false, 2, 1, 0},
    {"X:R-indexed-store",      0x102800, 0,        false, 4, 1, 0},
    {"X:R-predecrement-store", 0x103800, 0,        false, 4, 1, 0},
    {"X:R-absolute-store",     0x103000, 0x000040, true,  4, 1, 0},
    {"X:R-immediate-load",     0x10b400, 0x654321, true,  4, 0, 0},
    {"R:Y-direct-store",       0x106000, 0,        false, 2, 0, 1},
    {"R:Y-indexed-store",      0x106800, 0,        false, 4, 0, 1},
    {"Class-II-X-direct",      0x082000, 0,        false, 2, 1, 0},
    {"L-short-internal",       0x422000, 0,        false, 2, 1, 1},
    {"L-indexed-internal",     0x426800, 0,        false, 4, 1, 1},
    {"L-absolute-internal",    0x427000, 0x000040, true,  4, 1, 1},
    {"L-absolute-external",    0x427000, 0x00b002, true,  6, 1, 1},
    {"X:Y-internal",           0xc080e1, 0,        false, 2, 1, 1},
}};

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 8> p{};
    uint32_t instruction;
    uint16_t successor_address;
    uint64_t steps = 0;
    int64_t decode_step = -1;
    int64_t writeback_step = -1;
    int64_t successor_step = -1;
    unsigned x_writes = 0;
    unsigned y_writes = 0;

    explicit Harness(const Case &test)
        : instruction(test.instruction), successor_address(test.two_word ? 2 : 1) {
        p[0] = instruction;
        if (test.two_word)
            p[1] = test.extension;
        p[successor_address] = kSuccessor;
        p[successor_address + 1] = 0x0c0003; // Stable terminal JMP.

        core.clk = 0;
        core.rst_n = 0;
        core.clock_enable = 1;
        core.halt = 0;
        core.run = 0;
        core.p_fetch_data = 0;
        core.p_fetch_ready = 1;
        core.x_rdata = 0x13579b;
        core.y_rdata = 0x2468ac;
        core.p_data_rdata = 0;
        core.x_ready = core.y_ready = 1;
        core.irq_a = core.irq_b = 0;
        core.irq_ssi_tx = core.irq_ssi_rx = 0;
        core.irq_ssi_tx_exception = core.irq_ssi_rx_exception = 0;
        core.irq_sci_tx = core.irq_sci_rx = 0;
        core.irq_sci_rx_exception = core.irq_sci_idle = 0;
        core.irq_sci_timer = 0;
        core.irq_priority = 0;
        core.dbg_reg_wr_en = 0;
        core.dbg_reg_wr_addr = 0;
        core.dbg_reg_wr_data = 0;
        for (unsigned i = 0; i < 4; ++i)
            tick();
        core.rst_n = 1;

        debug_write(0x04, 0x123456); // X0.
        debug_write(0x06, 0x654321); // Y0.
        debug_write(0x0e, 0x00123456000000ULL); // A.
    }

    void debug_write(uint8_t address, uint64_t data) {
        core.dbg_reg_wr_addr = address;
        core.dbg_reg_wr_data = data;
        core.dbg_reg_wr_en = 1;
        tick();
        core.dbg_reg_wr_en = 0;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch = core.p_fetch_en;
        const uint16_t fetch_address = core.p_fetch_addr;
        if (core.state_out == 2 &&
            (core.p_fetch_data & 0x00ffffff) == instruction &&
            decode_step < 0)
            decode_step = static_cast<int64_t>(steps);
        if (core.state_out == 4 && core.cur_inst_out == instruction &&
            writeback_step < 0)
            writeback_step = static_cast<int64_t>(steps);
        if (core.state_out == 4 && core.cur_inst_out == kSuccessor &&
            successor_step < 0)
            successor_step = static_cast<int64_t>(steps);
        x_writes += core.x_wr_en;
        y_writes += core.y_wr_en;
        core.clk = 1;
        core.eval();
        if (fetch)
            core.p_fetch_data = p[fetch_address & 7] & 0x00ffffff;
        ++steps;
    }
};

bool run_case(const Case &test) {
    Harness h(test);
    h.core.run = 1;
    for (unsigned i = 0; i < 80 && h.successor_step < 0; ++i)
        h.tick();

    const int64_t clocks = h.decode_step >= 0 && h.writeback_step >= 0 ?
        h.writeback_step - h.decode_step + 1 : -1;
    const int64_t successor_gap = h.writeback_step >= 0 &&
                                  h.successor_step >= 0 ?
        h.successor_step - h.writeback_step : -1;
    const bool event_ok = h.x_writes == test.expected_x_writes &&
                          h.y_writes == test.expected_y_writes;
    const bool pass = clocks == test.expected_clocks &&
                      successor_gap == 2 && event_ok;
    std::printf("%s PARALLEL MOVE timing %-23s clocks=%lld/%u gap=%lld/2 "
                "XW=%u/%u YW=%u/%u event=%u decode=%lld wb=%lld successor=%lld\n",
                pass ? "PASS" : "FAIL", test.name,
                static_cast<long long>(clocks), test.expected_clocks,
                static_cast<long long>(successor_gap), h.x_writes,
                test.expected_x_writes, h.y_writes,
                test.expected_y_writes, event_ok,
                static_cast<long long>(h.decode_step),
                static_cast<long long>(h.writeback_step),
                static_cast<long long>(h.successor_step));
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    unsigned failures = 0;
    for (const auto &test : kCases)
        failures += !run_case(test);
    std::printf("PARALLEL MOVE TIMING SUMMARY cases=%zu failures=%u\n",
                kCases.size(), failures);
    return failures ? 1 : 0;
}
