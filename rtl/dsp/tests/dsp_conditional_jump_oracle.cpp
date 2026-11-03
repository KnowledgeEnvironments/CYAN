// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

// All-condition production-core Jcc/JScc matrix.

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "Vdsp56001_core___024root.h"
#include "verilated.h"

namespace {
constexpr uint16_t kTarget = 0x20;
constexpr uint32_t kFall = 0x365a00;   // MOVE #<$5a,R6.
constexpr uint32_t kTaken = 0x37a500;  // MOVE #<$a5,R7.

struct Condition { const char *name; uint8_t code, yes, no; };
constexpr std::array<Condition, 16> kConditions{{
    {"CC",0x0,0x00,0x01}, {"GE",0x1,0x00,0x08},
    {"NE",0x2,0x00,0x04}, {"PL",0x3,0x00,0x08},
    {"NN",0x4,0x10,0x00}, {"EC",0x5,0x00,0x20},
    {"LC",0x6,0x00,0x40}, {"GT",0x7,0x00,0x04},
    {"CS",0x8,0x01,0x00}, {"LT",0x9,0x08,0x00},
    {"EQ",0xa,0x04,0x00}, {"MI",0xb,0x08,0x00},
    {"NR",0xc,0x00,0x10}, {"ES",0xd,0x20,0x00},
    {"LS",0xe,0x40,0x00}, {"LE",0xf,0x04,0x00},
}};

uint32_t opcode(uint8_t cc, bool call, bool direct) {
    return direct ? ((call ? 0x0be0a0u : 0x0ae0a0u) | cc) :
        ((call ? 0x0f0000u : 0x0e0000u) | (uint32_t(cc) << 12) | kTarget);
}

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 64> p{};
    uint32_t op;
    uint64_t cycles = 0;
    int64_t decode = -1, writeback = -1, marker = -1;
    bool went_to_target = false;
    unsigned xr = 0, xw = 0, yr = 0, yw = 0;

    Harness(uint32_t instruction, uint8_t sr) : op(instruction) {
        p[0] = op; p[1] = kFall; p[2] = 0x0c0002;
        p[kTarget] = kTaken; p[kTarget + 1] = 0x0c0021;
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
        // The debug write port deliberately exposes only data/accumulator
        // registers.  Seed R0 directly before execution so the test still
        // exercises the synthesizable AGU/branch path and verifies that path
        // leaves R0 unchanged.
        core.rootp->dsp56001_core__DOT__r_reg[0] = kTarget;
        debug_write(0x39, sr);
        core.run = 1;
    }

    void debug_write(uint8_t address, uint64_t data) {
        core.dbg_reg_wr_en = 1; core.dbg_reg_wr_addr = address;
        core.dbg_reg_wr_data = data; tick(); core.dbg_reg_wr_en = 0;
    }

    void tick() {
        core.clk = 0; core.eval();
        const bool fetch = core.p_fetch_en;
        const uint16_t address = core.p_fetch_addr;
        if (core.state_out == 2 && (core.p_fetch_data & 0xffffff) == op &&
            decode < 0) decode = cycles;
        if (core.state_out == 4 && core.cur_inst_out == op && writeback < 0)
            writeback = cycles;
        if (core.state_out == 4 && marker < 0 &&
            (core.cur_inst_out == kFall || core.cur_inst_out == kTaken)) {
            marker = cycles; went_to_target = core.cur_inst_out == kTaken;
        }
        xr += core.x_rd_en; xw += core.x_wr_en;
        yr += core.y_rd_en; yw += core.y_wr_en;
        core.clk = 1; core.eval();
        if (fetch) core.p_fetch_data = p[address & 0x3f] & 0xffffff;
        ++cycles;
    }
};

bool run_case(const Condition &cc, bool taken, bool call, bool direct) {
    const uint8_t sr = taken ? cc.yes : cc.no;
    Harness h(opcode(cc.code, call, direct), sr);
    for (unsigned i = 0; i < 100 && h.marker < 0; ++i) h.tick();
    const auto *root = h.core.rootp;
    const int64_t clocks = h.writeback - h.decode + 1;
    const int64_t gap = h.marker - h.writeback;
    const bool pushed = call && taken;
    const uint64_t ss0 = root->dsp56001_core__DOT__ss[0];
    const bool stack = root->dsp56001_core__DOT__sp == (pushed ? 1 : 0) &&
        (!pushed || (((ss0 >> 24) & 0xffff) == 1 &&
                     (ss0 & 0xffffff) == sr));
    const bool pass = h.decode >= 0 && h.writeback >= 0 && h.marker >= 0 &&
        h.went_to_target == taken && clocks == 4 && gap == 2 && stack &&
        h.core.sr_out == sr && root->dsp56001_core__DOT__r_reg[0] == kTarget &&
        h.xr == 0 && h.xw == 0 && h.yr == 0 && h.yw == 0;
    if (!pass)
        std::printf("FAIL %s%s %s %s op=%06x target=%u/%u clocks=%lld "
                    "gap=%lld SP=%u SS0=%012llx SR=%02x/%02x R0=%04x "
                    "bus=%u/%u/%u/%u\n", call ? "JS" : "J", cc.name,
                    direct ? "direct" : "short", taken ? "taken" : "skip",
                    h.op, h.went_to_target, taken,
                    static_cast<long long>(clocks), static_cast<long long>(gap),
                    root->dsp56001_core__DOT__sp,
                    static_cast<unsigned long long>(ss0), h.core.sr_out & 0xff,
                    sr, root->dsp56001_core__DOT__r_reg[0],
                    h.xr, h.xw, h.yr, h.yw);
    return pass;
}
} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    unsigned cases = 0, failures = 0;
    for (const auto &cc : kConditions)
        for (unsigned taken = 0; taken < 2; ++taken)
            for (unsigned call = 0; call < 2; ++call)
                for (unsigned direct = 0; direct < 2; ++direct) {
                    failures += !run_case(cc, taken, call, direct); ++cases;
                }
    std::printf("%s DSP56001 conditional jump/call matrix cases=%u "
                "failures=%u conditions=16 forms=4 polarities=2\n",
                failures ? "FAIL" : "PASS", cases, failures);
    return failures ? 1 : 0;
}
