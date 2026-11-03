// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 MOVEC production-core timing oracle
//
// DSP56001UMA3 Tables A-6, A-8, A-13 and A-14 define zero-wait-state MOVEC
// as 2 + mvc oscillator clocks.  Short-immediate, register, short-address and
// ordinary indirect forms take two clocks.  Long-immediate, indexed,
// predecrement and absolute forms add two effective-address clocks.  Each
// memory case also requires exactly one transfer, while non-memory cases check
// their architectural SR result, so timing-only no-ops cannot pass.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint32_t kSuccessor = 0x375500; // MOVE #<$55,R7.
constexpr uint32_t kValue = 0x000035;

enum class Effect { SrResult, XRead, XWrite, YRead, YWrite };

struct Case {
    const char *name;
    uint32_t instruction;
    bool two_word;
    unsigned expected_clocks;
    Effect effect;
    uint16_t address;
};

constexpr std::array<Case, 17> kCases{{
    {"short-immediate", 0x0535b9, false, 2, Effect::SrResult, 0},
    {"register",        0x04c4b9, false, 2, Effect::SrResult, 0},
    {"long-immediate",  0x05f439, true,  4, Effect::SrResult, 0},
    {"short-X-read",    0x059239, false, 2, Effect::XRead,  0x0012},
    {"short-X-write",   0x051239, false, 2, Effect::XWrite, 0x0012},
    {"direct-X-read",   0x05e039, false, 2, Effect::XRead,  0x0040},
    {"direct-X-write",  0x056039, false, 2, Effect::XWrite, 0x0040},
    {"index-X-read",    0x05e839, false, 4, Effect::XRead,  0x0043},
    {"index-X-write",   0x056839, false, 4, Effect::XWrite, 0x0043},
    {"predec-X-read",   0x05f839, false, 4, Effect::XRead,  0x003f},
    {"predec-X-write",  0x057839, false, 4, Effect::XWrite, 0x003f},
    {"absolute-X-read", 0x05f039, true,  4, Effect::XRead,  0x0044},
    {"absolute-X-write",0x057039, true,  4, Effect::XWrite, 0x0044},
    {"direct-Y-read",   0x05e079, false, 2, Effect::YRead,  0x0040},
    {"direct-Y-write",  0x056079, false, 2, Effect::YWrite, 0x0040},
    {"absolute-Y-read", 0x05f079, true,  4, Effect::YRead,  0x0044},
    {"absolute-Y-write",0x057079, true,  4, Effect::YWrite, 0x0044},
}};

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};
    std::array<uint32_t, 256> x{};
    std::array<uint32_t, 256> y{};
    uint32_t instruction;
    uint16_t successor_address;
    uint64_t steps = 0;
    int64_t decode_step = -1;
    int64_t writeback_step = -1;
    int64_t successor_step = -1;
    unsigned x_reads = 0, x_writes = 0, y_reads = 0, y_writes = 0;
    uint16_t transfer_address = 0;
    uint32_t transfer_data = 0;

    explicit Harness(const Case &test)
        : instruction(test.instruction), successor_address(test.two_word ? 6 : 5) {
        p[0] = 0x60f400; // MOVE #>$40,R0.
        p[1] = 0x000040;
        p[2] = 0x380300; // MOVE #<3,N0.
        p[3] = 0x000000; // NOP gives R0 its documented pipeline spacing.
        p[4] = instruction;
        if (test.two_word)
            p[5] = test.address ? test.address : kValue;
        p[successor_address] = kSuccessor;
        p[successor_address + 1] = 0x0c0007; // Stable terminal JMP.
        x[test.address & 0xff] = kValue;
        y[test.address & 0xff] = kValue;

        core.clk = 0;
        core.rst_n = 0;
        core.clock_enable = 1;
        core.halt = 0;
        core.run = 0;
        core.p_fetch_data = 0;
        core.p_fetch_ready = 1;
        core.x_rdata = core.y_rdata = core.p_data_rdata = 0;
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
        core.dbg_reg_wr_addr = 0x04; // X0, source for register MOVEC.
        core.dbg_reg_wr_data = kValue;
        tick();
        core.dbg_reg_wr_addr = 0x39; // SR, source for memory stores.
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
        if (fetch)
            core.p_fetch_data = p[fetch_address & 0xff] & 0x00ffffff;
        if (xr) {
            ++x_reads; transfer_address = xa; transfer_data = x[xa & 0xff];
            core.x_rdata = transfer_data;
        }
        if (xw) {
            ++x_writes; transfer_address = xa; transfer_data = xwd;
            x[xa & 0xff] = xwd;
        }
        if (yr) {
            ++y_reads; transfer_address = ya; transfer_data = y[ya & 0xff];
            core.y_rdata = transfer_data;
        }
        if (yw) {
            ++y_writes; transfer_address = ya; transfer_data = ywd;
            y[ya & 0xff] = ywd;
        }
        ++steps;
    }
};

bool run_case(const Case &test) {
    Harness h(test);
    h.core.run = 1;
    for (unsigned i = 0; i < 100 && h.successor_step < 0; ++i)
        h.tick();

    const int64_t clocks = h.decode_step >= 0 && h.writeback_step >= 0 ?
        h.writeback_step - h.decode_step + 1 : -1;
    const int64_t successor_gap = h.writeback_step >= 0 &&
                                  h.successor_step >= 0 ?
        h.successor_step - h.writeback_step : -1;
    bool effect_ok = false;
    switch (test.effect) {
    case Effect::SrResult:
        effect_ok = h.core.sr_out == kValue && h.x_reads == 0 &&
                    h.x_writes == 0 && h.y_reads == 0 && h.y_writes == 0;
        break;
    case Effect::XRead:
        effect_ok = h.x_reads == 1 && h.x_writes == 0 && h.y_reads == 0 &&
                    h.y_writes == 0 && h.transfer_address == test.address &&
                    h.core.sr_out == kValue;
        break;
    case Effect::XWrite:
        effect_ok = h.x_reads == 0 && h.x_writes == 1 && h.y_reads == 0 &&
                    h.y_writes == 0 && h.transfer_address == test.address &&
                    h.transfer_data == kValue;
        break;
    case Effect::YRead:
        effect_ok = h.x_reads == 0 && h.x_writes == 0 && h.y_reads == 1 &&
                    h.y_writes == 0 && h.transfer_address == test.address &&
                    h.core.sr_out == kValue;
        break;
    case Effect::YWrite:
        effect_ok = h.x_reads == 0 && h.x_writes == 0 && h.y_reads == 0 &&
                    h.y_writes == 1 && h.transfer_address == test.address &&
                    h.transfer_data == kValue;
        break;
    }
    const bool pass = clocks == test.expected_clocks &&
                      successor_gap == 2 && effect_ok;
    std::printf("%s MOVEC timing %-18s clocks=%lld/%u gap=%lld/2 "
                "XY=%u/%u/%u/%u@%04x data=%06x SR=%06x effect=%u\n",
                pass ? "PASS" : "FAIL", test.name,
                static_cast<long long>(clocks), test.expected_clocks,
                static_cast<long long>(successor_gap), h.x_reads, h.x_writes,
                h.y_reads, h.y_writes, h.transfer_address, h.transfer_data,
                h.core.sr_out, effect_ok);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    unsigned failures = 0;
    for (const auto &test : kCases)
        failures += !run_case(test);
    std::printf("MOVEC TIMING SUMMARY cases=%zu failures=%u\n",
                kCases.size(), failures);
    return failures ? 1 : 0;
}
