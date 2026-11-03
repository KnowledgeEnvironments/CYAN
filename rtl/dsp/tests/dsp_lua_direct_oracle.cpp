// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

// Direct production-core DSP56001 LUA address-update oracle.

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
    uint8_t mode;
    uint8_t source;
    uint8_t destination;
    uint16_t r;
    uint16_t n;
    uint16_t m;
    uint16_t expected_destination;
    uint16_t expected_source_r;
};

constexpr std::array<Case, 6> kCases{{
    {"minus-N",       0, 0, 0x11, 0x0010, 0x0003, 0xffff, 0x000d, 0x0010},
    {"plus-N",        1, 0, 0x11, 0x0010, 0x0003, 0xffff, 0x0013, 0x0010},
    {"post-decrement",2, 2, 0x1b, 0x0000, 0x7777, 0xffff, 0xffff, 0x0000},
    {"post-increment-same-D", 3, 4, 0x14, 0xffff, 0x7777, 0xffff,
                                            0x0000, 0x0000},
    {"modulo-wrap",   1, 1, 0x12, 0x001f, 0x0001, 0x000f, 0x0010, 0x001f},
    {"bit-reverse",   1, 3, 0x1a, 0x8000, 0x8000, 0x0000, 0x4000, 0x8000},
}};

uint32_t move_long_immediate(uint8_t destination) {
    return ((0x40u | ((destination >> 3) << 4) | (destination & 7)) << 16) |
           0xf400u;
}

uint32_t movec_long_immediate(uint8_t destination) {
    return 0x05f400u | destination;
}

uint32_t lua_opcode(const Case &test) {
    return 0x044010u | (uint32_t(test.mode) << 11) |
           (uint32_t(test.source) << 8) | (test.destination & 0x0f);
}

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 32> p{};
    uint32_t opcode;
    uint64_t cycles = 0;
    int64_t decode_cycle = -1;
    int64_t writeback_cycle = -1;
    int64_t successor_cycle = -1;
    unsigned xr = 0, xw = 0, yr = 0, yw = 0;

    explicit Harness(const Case &test) : opcode(lua_opcode(test)) {
        unsigned pc = 0;
        p[pc++] = move_long_immediate(0x10 + test.source);
        p[pc++] = test.r;
        p[pc++] = move_long_immediate(0x18 + test.source);
        p[pc++] = test.n;
        if (test.m != 0xffff) {
            p[pc++] = movec_long_immediate(0x20 + test.source);
            p[pc++] = test.m;
        }
        p[pc++] = 0x000000; // Address-register visibility spacing.
        p[pc++] = opcode;
        p[pc++] = kSuccessor;
        p[pc] = 0x0c0000u | pc;

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
        const bool fetch = core.p_fetch_en;
        const uint16_t address = core.p_fetch_addr;
        if (core.state_out == 2 &&
            (core.p_fetch_data & 0x00ffffff) == opcode && decode_cycle < 0)
            decode_cycle = cycles;
        if (core.state_out == 4 && core.cur_inst_out == opcode &&
            writeback_cycle < 0)
            writeback_cycle = cycles;
        if (core.state_out == 4 && core.cur_inst_out == kSuccessor &&
            successor_cycle < 0)
            successor_cycle = cycles;
        xr += core.x_rd_en;
        xw += core.x_wr_en;
        yr += core.y_rd_en;
        yw += core.y_wr_en;
        core.clk = 1;
        core.eval();
        if (fetch)
            core.p_fetch_data = p[address & 0x1f] & 0x00ffffff;
        ++cycles;
    }

    uint16_t destination(uint8_t code) const {
        if (code < 0x18)
            return core.rootp->dsp56001_core__DOT__r_reg[code & 7];
        return core.rootp->dsp56001_core__DOT__n_reg[code & 7];
    }
};

bool run_case(const Case &test) {
    Harness h(test);
    for (unsigned i = 0; i < 120 && h.successor_cycle < 0; ++i)
        h.tick();
    const int64_t clocks = h.decode_cycle >= 0 && h.writeback_cycle >= 0 ?
                           h.writeback_cycle - h.decode_cycle + 1 : -1;
    const int64_t gap = h.writeback_cycle >= 0 && h.successor_cycle >= 0 ?
                        h.successor_cycle - h.writeback_cycle : -1;
    const uint16_t destination = h.destination(test.destination);
    const uint16_t source =
        h.core.rootp->dsp56001_core__DOT__r_reg[test.source];
    const bool bus_quiet = h.xr == 0 && h.xw == 0 && h.yr == 0 && h.yw == 0;
    const bool pass = clocks == 4 && gap == 2 &&
                      destination == test.expected_destination &&
                      source == test.expected_source_r && bus_quiet &&
                      h.core.rootp->dsp56001_core__DOT__r_reg[7] == 0x0055;
    std::printf("%s LUA %-21s opcode=%06x clocks=%lld/4 gap=%lld/2 "
                "D=%04x/%04x R%u=%04x/%04x bus=X%u/%u,Y%u/%u\n",
                pass ? "PASS" : "FAIL", test.name, h.opcode,
                static_cast<long long>(clocks), static_cast<long long>(gap),
                destination, test.expected_destination, test.source, source,
                test.expected_source_r, h.xr, h.xw, h.yr, h.yw);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    unsigned failures = 0;
    for (const auto &test : kCases)
        failures += !run_case(test);
    std::printf("LUA SUMMARY cases=%zu failures=%u\n", kCases.size(), failures);
    return failures == 0 ? 0 : 1;
}
