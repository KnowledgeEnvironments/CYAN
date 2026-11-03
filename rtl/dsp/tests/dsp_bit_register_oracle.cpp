// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

// DSP56001 production-core register/controller B-operation oracle.

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "Vdsp56001_core___024root.h"
#include "verilated.h"

namespace {

enum class Operation { Bclr, Bset, Bchg, Btst };

struct Case {
    const char *name;
    uint8_t code;
    uint8_t setup_code;
    Operation operation;
    uint8_t bit;
    uint32_t initial;
    uint32_t expected;
    uint8_t expected_carry;
    uint8_t expected_sp;
};

constexpr std::array<Case, 10> kCases{{
    {"X0-BSET", 0x04, 0x04, Operation::Bset, 5, 0x000000, 0x000020, 0, 0},
    {"A2-BCLR", 0x0a, 0x0a, Operation::Bclr, 7, 0x0000ff, 0x00007f, 1, 0},
    {"R3-BCHG", 0x13, 0x13, Operation::Bchg, 11, 0x000800, 0x000000, 1, 0},
    {"M4-BTST", 0x24, 0x24, Operation::Btst, 15, 0x007fff, 0x007fff, 0, 0},
    {"SR-BTST", 0x29, 0x39, Operation::Btst, 5, 0x000020, 0x000021, 1, 0},
    {"OMR-BCHG", 0x2a, 0x3a, Operation::Bchg, 6, 0x000000, 0x000040, 0, 0},
    {"SP-BCLR-UF", 0x2b, 0x3b, Operation::Bclr, 5, 0x000020, 0x000000, 1, 0},
    {"SSH-BSET", 0x2c, 0x3c, Operation::Bset, 3, 0x001234, 0x00123c, 0, 1},
    {"LA-BSET", 0x2e, 0x3e, Operation::Bset, 15, 0x001234, 0x009234, 0, 0},
    {"LC-BCHG", 0x2f, 0x3f, Operation::Bchg, 4, 0x001234, 0x001224, 1, 0},
}};

uint32_t move_long_immediate(uint8_t destination) {
    return ((0x40u | ((destination >> 3) << 4) | (destination & 7)) << 16) |
           0xf400u;
}

uint32_t movec_long_immediate(uint8_t destination) {
    return 0x05f400u | destination;
}

uint32_t bit_opcode(uint8_t code, Operation operation, uint8_t bit) {
    const uint32_t family = (operation == Operation::Bclr ||
                             operation == Operation::Bset) ? 0x0a0000u :
                                                             0x0b0000u;
    const bool selector = operation == Operation::Bset ||
                          operation == Operation::Btst;
    return family | 0x00c040u | (uint32_t(code) << 8) |
           (selector ? 0x20u : 0u) | bit;
}

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 16> p{};
    uint64_t cycles = 0;
    int64_t decode_cycle = -1;
    int64_t writeback_cycle = -1;
    unsigned xr = 0, xw = 0, yr = 0, yw = 0;
    uint32_t tested_opcode = 0;

    explicit Harness(const Case &test) {
        // M and controller destinations use MOVEC's full six-bit selector.
        // R/N retain the ordinary MOVE data-register encoding.
        const bool control = test.setup_code >= 0x20;
        p[0] = control ? movec_long_immediate(test.setup_code) :
                         move_long_immediate(test.setup_code);
        p[1] = test.initial;
        p[2] = 0x000000;
        tested_opcode = bit_opcode(test.code, test.operation, test.bit);
        p[3] = tested_opcode;
        p[4] = 0x375500; // MOVE #<$55,R7.
        p[5] = 0x0c0005;

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
            (core.p_fetch_data & 0x00ffffff) == tested_opcode &&
            decode_cycle < 0)
            decode_cycle = cycles;
        if (core.state_out == 4 && core.cur_inst_out == tested_opcode &&
            writeback_cycle < 0)
            writeback_cycle = cycles;
        xr += core.x_rd_en;
        xw += core.x_wr_en;
        yr += core.y_rd_en;
        yw += core.y_wr_en;
        core.clk = 1;
        core.eval();
        if (fetch)
            core.p_fetch_data = p[address & 0x0f] & 0x00ffffff;
        ++cycles;
    }

    uint32_t target_value(uint8_t code) const {
        switch (code) {
            case 0x04: return core.rootp->dsp56001_core__DOT__x0_reg;
            case 0x0a: return (core.acc_a_out >> 48) & 0xff;
            case 0x13: return core.rootp->dsp56001_core__DOT__r_reg[3];
            case 0x24: return core.rootp->dsp56001_core__DOT__m_reg[4];
            case 0x29: return core.sr_out;
            case 0x2a: return core.rootp->dsp56001_core__DOT__omr;
            case 0x2b: return core.rootp->dsp56001_core__DOT__sp;
            case 0x2c: return
                (core.rootp->dsp56001_core__DOT__ss[0] >> 24) & 0xffff;
            case 0x2e: return core.rootp->dsp56001_core__DOT__la;
            case 0x2f: return core.rootp->dsp56001_core__DOT__lc & 0xffff;
            default: return 0xffffffff;
        }
    }
};

bool run_case(const Case &test) {
    Harness h(test);
    for (unsigned i = 0; i < 100 && h.writeback_cycle < 0; ++i)
        h.tick();
    if (h.writeback_cycle >= 0)
        h.tick();
    const int64_t clocks = h.decode_cycle >= 0 && h.writeback_cycle >= 0 ?
                           h.writeback_cycle - h.decode_cycle + 1 : -1;
    const uint32_t actual = h.target_value(test.code);
    const uint8_t carry = h.core.sr_out & 1;
    const uint8_t sp = h.core.rootp->dsp56001_core__DOT__sp;
    const bool bus_quiet = h.xr == 0 && h.xw == 0 && h.yr == 0 && h.yw == 0;
    const bool pass = clocks == 4 && actual == test.expected &&
                      carry == test.expected_carry && sp == test.expected_sp &&
                      h.core.rootp->dsp56001_core__DOT__r_reg[7] == 0 &&
                      bus_quiet;
    std::printf("%s BIT register %-12s opcode=%06x clocks=%lld/4 "
                "value=%06x/%06x C=%u/%u SP=%02x/%02x bus=X%u/%u,Y%u/%u\n",
                pass ? "PASS" : "FAIL", test.name, h.tested_opcode,
                static_cast<long long>(clocks), actual, test.expected,
                carry, test.expected_carry, sp, test.expected_sp,
                h.xr, h.xw, h.yr, h.yw);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    unsigned failures = 0;
    for (const auto &test : kCases)
        failures += !run_case(test);
    std::printf("BIT REGISTER SUMMARY cases=%zu failures=%u\n",
                kCases.size(), failures);
    return failures == 0 ? 0 : 1;
}
