// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 RND production-core timing oracle
//
// The isolated RND description at DSP56001UMA2 A-193 says four oscillator
// clocks, but that contradicts both its optional parallel-move encoding and
// the same manual's consolidated timing table at DSP56001UMA3 A-228, which
// specifies 2+mv.  Motorola's DSP56002 instruction-set summary independently
// gives the same 2+mv timing.  Grade the no-parallel-move form here at two
// clocks, using the same decode-through-writeback convention as the admitted
// RESET and DO timing oracles.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint32_t kRndA = 0x200011;
constexpr uint32_t kSuccessor = 0x375500; // MOVE #<$55,R7

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 8> p{};
    uint64_t steps = 0;
    int64_t decode_step = -1;
    int64_t writeback_step = -1;
    int64_t successor_step = -1;

    Harness() {
        p[0] = kRndA;
        p[1] = kSuccessor;
        p[2] = 0x0c0002; // JMP $002

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
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch = core.p_fetch_en;
        const uint16_t fetch_address = core.p_fetch_addr;
        if (core.state_out == 2 &&
            (core.p_fetch_data & 0x00ffffff) == kRndA && decode_step < 0)
            decode_step = static_cast<int64_t>(steps);
        if (core.state_out == 4 && core.cur_inst_out == kRndA &&
            writeback_step < 0)
            writeback_step = static_cast<int64_t>(steps);
        if (core.state_out == 4 && core.cur_inst_out == kSuccessor &&
            successor_step < 0)
            successor_step = static_cast<int64_t>(steps);
        core.clk = 1;
        core.eval();
        if (fetch)
            core.p_fetch_data = p[fetch_address & 7] & 0x00ffffff;
        ++steps;
    }
};

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Harness h;
    h.core.run = 1;
    for (unsigned i = 0; i < 40 && h.successor_step < 0; ++i)
        h.tick();

    const int64_t clocks = h.decode_step >= 0 && h.writeback_step >= 0 ?
        h.writeback_step - h.decode_step + 1 : -1;
    const int64_t successor_gap = h.writeback_step >= 0 &&
                                  h.successor_step >= 0 ?
        h.successor_step - h.writeback_step : -1;
    const bool pass = clocks == 2 && successor_gap == 2;
    std::printf("%s RND timing clocks=%lld/2 successor_gap=%lld/2 "
                "decode=%lld wb=%lld successor=%lld\n",
                pass ? "PASS" : "FAIL",
                static_cast<long long>(clocks),
                static_cast<long long>(successor_gap),
                static_cast<long long>(h.decode_step),
                static_cast<long long>(h.writeback_step),
                static_cast<long long>(h.successor_step));
    return pass ? 0 : 1;
}

