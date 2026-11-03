// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 status-register reserved-bit boundary oracle
//
// DSP56001UM06 Figure 6-6 defines SR as a 16-bit register with bits 14, 12,
// and 7 reserved.  MOVEC #>S,SR is a production instruction path which must
// discard those bits and the nonexistent upper byte.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint8_t kStateWritebackFetch = 4;
constexpr uint32_t kMovecLongImmediateSr = 0x05f439;
constexpr uint32_t kMovecSrToA = 0x04f98e;
constexpr uint32_t kOriAllMr = 0x00fff8;
constexpr uint32_t kArchitecturalSrMask = 0x00af7f;

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 16> p{};

    Harness() {
        p[0] = kMovecLongImmediateSr;
        p[1] = 0xffffff;
        p[2] = kMovecSrToA;
        p[3] = 0x0af080; // JMP >$0003.
        p[4] = 0x000003;

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
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        core.clk = 1;
        core.eval();
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 0x0f] & 0x00ffffff;
    }

    bool retire(uint32_t opcode) {
        for (unsigned i = 0; i < 80; ++i) {
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

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);

    Harness h;
    const bool write_retired = h.retire(kMovecLongImmediateSr);
    const bool read_retired = h.retire(kMovecSrToA);
    const uint32_t actual = h.core.sr_out;
    const uint32_t readback = (h.core.acc_a_out >> 24) & 0x00ffffff;
    const bool movec_pass = write_retired && read_retired &&
                            actual == kArchitecturalSrMask &&
                            readback == kArchitecturalSrMask;
    std::printf("%s MOVEC #>$ffffff,SR / SR,A: retired=%u/%u "
                "SR=$%06x A1=$%06x expected=$%06x\n",
                movec_pass ? "PASS" : "FAIL", write_retired, read_retired,
                actual, readback, kArchitecturalSrMask);

    Harness ori;
    ori.p.fill(0);
    ori.p[0] = kMovecLongImmediateSr;
    ori.p[1] = 0x000000;
    ori.p[2] = kOriAllMr;
    ori.p[3] = kMovecSrToA;
    ori.p[4] = 0x0af080; // JMP >$0004.
    ori.p[5] = 0x000004;
    const bool clear_retired = ori.retire(kMovecLongImmediateSr);
    const bool ori_retired = ori.retire(kOriAllMr);
    const bool ori_read_retired = ori.retire(kMovecSrToA);
    const uint32_t ori_actual = ori.core.sr_out;
    const uint32_t ori_readback =
        (ori.core.acc_a_out >> 24) & 0x00ffffff;
    constexpr uint32_t kExpectedOriMr = 0x00af00;
    const bool ori_pass = clear_retired && ori_retired && ori_read_retired &&
                          ori_actual == kExpectedOriMr &&
                          ori_readback == kExpectedOriMr;
    std::printf("%s ORI #$ff,MR / SR,A: retired=%u/%u/%u "
                "SR=$%06x A1=$%06x expected=$%06x\n",
                ori_pass ? "PASS" : "FAIL", clear_retired, ori_retired,
                ori_read_retired, ori_actual, ori_readback, kExpectedOriMr);

    return movec_pass && ori_pass ? 0 : 1;
}

