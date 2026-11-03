// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 production-core bit-operation timing oracle
//
// DSP56001UMA3 Tables A-6, A-10, A-13 and A-14 define BCLR/BSET/BCHG/BTST
// as four clocks plus the effective-address and memory penalties.  With
// internal zero-wait X/Y and peripheral memory, register/short/ordinary/
// peripheral forms take four clocks; indexed/predecrement/absolute take six.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "Vdsp56001_core___024root.h"
#include "verilated.h"

namespace {

constexpr uint32_t kSuccessor = 0x375500; // MOVE #<$55,R7.

struct Case {
    const char *name;
    uint32_t instruction;
    bool two_word;
    bool memory;
    bool y_space;
    bool modifies;
    uint16_t address;
    unsigned expected_clocks;
};

constexpr std::array<Case, 14> kCases{{
    {"register-BSET",   0x0ac460, false, false, false, true,  0x0000, 4},
    {"register-BTST",   0x0bc460, false, false, false, false, 0x0000, 4},
    {"short-X-BSET",    0x0a1220, false, true,  false, true,  0x0012, 4},
    {"short-X-BTST",    0x0b1220, false, true,  false, false, 0x0012, 4},
    {"direct-X-BSET",   0x0a6020, false, true,  false, true,  0x0040, 4},
    {"direct-X-BTST",   0x0b6020, false, true,  false, false, 0x0040, 4},
    {"index-X-BSET",    0x0a6820, false, true,  false, true,  0x0043, 6},
    {"index-X-BTST",    0x0b6820, false, true,  false, false, 0x0043, 6},
    {"predec-X-BSET",   0x0a7820, false, true,  false, true,  0x003f, 6},
    {"predec-X-BTST",   0x0b7820, false, true,  false, false, 0x003f, 6},
    {"absolute-Y-BSET", 0x0a7060, true,  true,  true,  true,  0x0044, 6},
    {"absolute-Y-BTST", 0x0b7060, true,  true,  true,  false, 0x0044, 6},
    {"periph-X-BSET",   0x0a8220, false, true,  false, true,  0xffc2, 4},
    {"periph-X-BTST",   0x0b8220, false, true,  false, false, 0xffc2, 4},
}};

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};
    std::array<uint32_t, 65536> x{};
    std::array<uint32_t, 65536> y{};
    uint32_t instruction;
    uint16_t successor_address;
    uint64_t steps = 0;
    int64_t decode_step = -1, writeback_step = -1, successor_step = -1;
    unsigned x_reads = 0, x_writes = 0, y_reads = 0, y_writes = 0;
    uint16_t read_address = 0, write_address = 0;
    uint32_t write_data = 0;

    explicit Harness(const Case &test)
        : instruction(test.instruction), successor_address(test.two_word ? 6 : 5) {
        p[0] = 0x60f400; // MOVE #>$40,R0.
        p[1] = 0x000040;
        p[2] = 0x380300; // MOVE #<3,N0.
        p[3] = 0x000000; // Address-register pipeline spacing.
        p[4] = instruction;
        if (test.two_word)
            p[5] = test.address;
        p[successor_address] = kSuccessor;
        p[successor_address + 1] = 0x0c0007;
        x[test.address] = test.modifies ? 0 : 1;
        y[test.address] = test.modifies ? 0 : 1;

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
        core.dbg_reg_wr_addr = 0x04; // X0 register bit-operation source.
        core.dbg_reg_wr_data = test.modifies ? 0 : 1;
        tick();
        core.dbg_reg_wr_addr = 0x39; // C begins opposite the tested old bit.
        core.dbg_reg_wr_data = test.modifies ? 1 : 0;
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
        const uint32_t xwd = core.x_wdata & 0x00ffffff;
        const uint32_t ywd = core.y_wdata & 0x00ffffff;
        if (core.state_out == 2 &&
            (core.p_fetch_data & 0x00ffffff) == instruction && decode_step < 0)
            decode_step = static_cast<int64_t>(steps);
        if (core.state_out == 4 && core.cur_inst_out == instruction &&
            writeback_step < 0)
            writeback_step = static_cast<int64_t>(steps);
        if (core.state_out == 4 && core.cur_inst_out == kSuccessor &&
            successor_step < 0)
            successor_step = static_cast<int64_t>(steps);
        core.clk = 1;
        core.eval();
        if (fetch) core.p_fetch_data = p[fetch_address & 0xff] & 0x00ffffff;
        if (xr) {
            ++x_reads; read_address = xa; core.x_rdata = x[xa];
        }
        if (xw) {
            ++x_writes; write_address = xa; write_data = xwd; x[xa] = xwd;
        }
        if (yr) {
            ++y_reads; read_address = ya; core.y_rdata = y[ya];
        }
        if (yw) {
            ++y_writes; write_address = ya; write_data = ywd; y[ya] = ywd;
        }
        ++steps;
    }
};

bool run_case(const Case &test) {
    Harness h(test);
    h.core.run = 1;
    for (unsigned i = 0; i < 120 && h.successor_step < 0; ++i) h.tick();

    const int64_t clocks = h.decode_step >= 0 && h.writeback_step >= 0 ?
        h.writeback_step - h.decode_step + 1 : -1;
    const int64_t gap = h.writeback_step >= 0 && h.successor_step >= 0 ?
        h.successor_step - h.writeback_step : -1;
    bool effect_ok = false;
    if (!test.memory) {
        effect_ok = h.x_reads == 0 && h.x_writes == 0 && h.y_reads == 0 &&
                    h.y_writes == 0 && h.core.sr_out == (test.modifies ? 0 : 1) &&
                    (h.core.rootp->dsp56001_core__DOT__x0_reg & 1) == 1;
    } else {
        const unsigned reads = test.y_space ? h.y_reads : h.x_reads;
        const unsigned writes = test.y_space ? h.y_writes : h.x_writes;
        const unsigned other_reads = test.y_space ? h.x_reads : h.y_reads;
        const unsigned other_writes = test.y_space ? h.x_writes : h.y_writes;
        effect_ok = reads == 1 && writes == (test.modifies ? 1u : 0u) &&
                    other_reads == 0 && other_writes == 0 &&
                    h.read_address == test.address && h.core.sr_out ==
                    (test.modifies ? 0 : 1);
        if (test.modifies)
            effect_ok = effect_ok && h.write_address == test.address &&
                        h.write_data == 1;
    }
    const bool pass = clocks == test.expected_clocks && gap == 2 && effect_ok;
    std::printf("%s BIT timing %-18s clocks=%lld/%u gap=%lld/2 "
                "XY=%u/%u/%u/%u read=%04x write=%04x:%06x SR=%06x effect=%u\n",
                pass ? "PASS" : "FAIL", test.name,
                static_cast<long long>(clocks), test.expected_clocks,
                static_cast<long long>(gap), h.x_reads, h.x_writes,
                h.y_reads, h.y_writes, h.read_address, h.write_address,
                h.write_data, h.core.sr_out, effect_ok);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    unsigned failures = 0;
    for (const auto &test : kCases) failures += !run_case(test);
    std::printf("BIT TIMING SUMMARY cases=%zu failures=%u\n",
                kCases.size(), failures);
    return failures ? 1 : 0;
}
