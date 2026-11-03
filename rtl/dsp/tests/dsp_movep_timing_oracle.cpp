// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 MOVEP production-core timing oracle
//
// DSP56001UMA3 Tables A-6, A-9, A-13 and A-14 define the zero-wait-state
// timing used here:
//   register <-> internal peripheral       4 clocks
//   X/Y ordinary EA <-> peripheral         4 clocks
//   X/Y indexed/predecrement/absolute      6 clocks
//   P ordinary EA <-> peripheral           6 clocks
//   P indexed/predecrement/absolute        8 clocks
// The matrix measures decode through retirement and retains one exact
// register-to-peripheral event check so a timing-only no-op cannot pass.
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
    bool two_word;
    unsigned expected_clocks;
    bool check_x_write;
};

constexpr std::array<Case, 16> kCases{{
    {"reg-to-Xpp",       0x08c421, false, 4, true},
    {"Xpp-to-reg",       0x084722, false, 4, false},
    {"X-direct-to-Xpp",  0x08e0aa, false, 4, false},
    {"Xpp-to-X-direct",  0x0860ab, false, 4, false},
    {"X-index-to-Xpp",   0x08e8aa, false, 6, false},
    {"Xpp-to-X-index",   0x0868ab, false, 6, false},
    {"X-predec-to-Xpp",  0x08f8aa, false, 6, false},
    {"Xpp-to-X-predec",  0x0878ab, false, 6, false},
    {"X-abs-to-Xpp",     0x08f0a5, true,  6, false},
    {"Xpp-to-X-abs",     0x0870a6, true,  6, false},
    {"P-direct-to-Xpp",  0x08e06a, false, 6, false},
    {"Xpp-to-P-direct",  0x08606b, false, 6, false},
    {"P-index-to-Xpp",   0x08e86a, false, 8, false},
    {"Xpp-to-P-index",   0x08686b, false, 8, false},
    {"P-abs-to-Xpp",     0x08f063, true,  8, false},
    {"Xpp-to-P-abs",     0x087064, true,  8, false},
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
    uint16_t x_write_address = 0;

    explicit Harness(const Case &test)
        : instruction(test.instruction),
          successor_address(test.two_word ? 2 : 1) {
        p[0] = instruction;
        if (test.two_word)
            p[1] = 0x000040; // Internal absolute data address.
        p[successor_address] = kSuccessor;
        p[successor_address + 1] = 0x0c0003; // Stable terminal JMP.

        core.clk = 0;
        core.rst_n = 0;
        core.clock_enable = 1;
        core.halt = 0;
        core.run = 0;
        core.p_fetch_data = 0;
        core.p_fetch_ready = 1;
        core.x_rdata = core.y_rdata = core.p_data_rdata = 0x13579b;
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

        core.dbg_reg_wr_en = 1;
        core.dbg_reg_wr_addr = 0x04; // X0.
        core.dbg_reg_wr_data = 0x123456;
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
        if (core.x_wr_en) {
            ++x_writes;
            x_write_address = core.x_addr;
        }
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
    const bool event_ok = !test.check_x_write ||
                          (h.x_writes == 1 && h.x_write_address == 0xffe1);
    const bool pass = clocks == test.expected_clocks &&
                      successor_gap == 2 && event_ok;
    std::printf("%s MOVEP timing %-20s clocks=%lld/%u gap=%lld/2 "
                "XW=%u@%04x event=%u decode=%lld wb=%lld successor=%lld\n",
                pass ? "PASS" : "FAIL", test.name,
                static_cast<long long>(clocks), test.expected_clocks,
                static_cast<long long>(successor_gap), h.x_writes,
                h.x_write_address, event_ok,
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
    std::printf("MOVEP TIMING SUMMARY cases=%zu failures=%u\n",
                kCases.size(), failures);
    return failures ? 1 : 0;
}
