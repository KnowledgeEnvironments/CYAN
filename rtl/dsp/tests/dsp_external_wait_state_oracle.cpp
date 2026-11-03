// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 external X-memory wait-state oracle
//
// DSP56001UMA3 Appendix A adds the programmed X-memory wait-state term to an
// external access, and DSP56001UM02 section 2.10 permits an external device to
// extend that access.  The production core's ready handshake must therefore
// hold the instruction and its exact request until the response is accepted.
//
// This fixture delays one external MOVEC X:(R0),M0 read for three oscillator
// clocks.  The request/address must remain stable, neither the load nor its
// successor may retire early, and the accepted response must update M0 exactly
// once before MOVEC M0,A publishes it architecturally.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint32_t kSetupR0 = 0x60f400;  // MOVE #>$1234,R0
constexpr uint32_t kNop = 0x000000;
constexpr uint32_t kLoadM0 = 0x05e020;   // MOVEC X:(R0),M0
constexpr uint32_t kPublishM0 = 0x04e08e; // MOVEC M0,A
constexpr uint32_t kExpected = 0xab4567;
constexpr uint32_t kStale = 0x112233;
constexpr unsigned kDeniedClocks = 3;

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};
    unsigned request_clocks = 0;
    unsigned denied_clocks = 0;
    unsigned accepted_reads = 0;
    bool address_stable = true;
    bool load_retired_before_accept = false;
    bool successor_retired_before_accept = false;
    bool load_retired = false;
    bool successor_retired = false;

    Harness() {
        p[0] = kSetupR0;
        p[1] = 0x001234;
        // DSP56001UMA3 A.8.6: a move-type R write is not available as an AGU
        // pointer to its immediate follower.  Keep this fixture about external
        // response retention by waiting one instruction before using R0.
        p[2] = kNop;
        p[3] = kLoadM0;
        p[4] = kPublishM0;
        p[5] = 0x0c0005; // Stable terminal JMP.

        core.clk = 0;
        core.rst_n = 0;
        core.clock_enable = 1;
        core.halt = 0;
        core.run = 0;
        core.p_fetch_data = 0;
        core.p_fetch_ready = 1;
        core.x_rdata = kStale;
        core.x_ready = 0;
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
        core.irq_sci_rx_exception = 0;
        core.irq_sci_idle = 0;
        core.irq_sci_timer = 0;
        core.irq_priority = 0;
        core.dbg_reg_wr_en = 0;
        core.dbg_reg_wr_addr = 0;
        core.dbg_reg_wr_data = 0;

        for (unsigned i = 0; i < 4; ++i)
            tick(false);
        core.rst_n = 1;
    }

    void tick(bool observe = true) {
        core.clk = 0;
        core.eval();

        const bool fetch = core.p_fetch_en;
        const uint16_t fetch_address = core.p_fetch_addr;
        const bool request = core.x_rd_en;

        if (observe && core.state_out == 4 && core.cur_inst_out == kLoadM0) {
            load_retired = true;
            if (accepted_reads == 0)
                load_retired_before_accept = true;
        }
        if (observe && core.state_out == 4 &&
            core.cur_inst_out == kPublishM0) {
            successor_retired = true;
            if (accepted_reads == 0)
                successor_retired_before_accept = true;
        }

        if (observe && request) {
            ++request_clocks;
            address_stable &= core.x_addr == 0x1234;
            if (denied_clocks < kDeniedClocks) {
                ++denied_clocks;
                core.x_ready = 0;
                core.x_rdata = kStale;
            } else {
                core.x_ready = 1;
                core.x_rdata = kExpected;
                core.eval();
                ++accepted_reads;
            }
        }

        core.clk = 1;
        core.eval();
        if (fetch)
            core.p_fetch_data = p[fetch_address & 0xff] & 0x00ffffff;
    }
};

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Harness h;
    h.core.run = 1;

    for (unsigned i = 0; i < 200 && !h.successor_retired; ++i)
        h.tick();

    const uint32_t published = (h.core.acc_a_out >> 24) & 0x00ffffff;
    const bool pass = h.denied_clocks == kDeniedClocks &&
                      h.request_clocks == kDeniedClocks + 1 &&
                      h.accepted_reads == 1 && h.address_stable &&
                      !h.load_retired_before_accept &&
                      !h.successor_retired_before_accept &&
                      h.load_retired && h.successor_retired &&
                      published == (kExpected & 0x00ffff);

    std::printf("%s external X wait state denied=%u/%u requests=%u/%u "
                "accepted=%u address=%u early=%u/%u retired=%u/%u "
                "published=%06x/%06x\n",
                pass ? "PASS" : "FAIL", h.denied_clocks, kDeniedClocks,
                h.request_clocks, kDeniedClocks + 1, h.accepted_reads,
                h.address_stable, h.load_retired_before_accept,
                h.successor_retired_before_accept, h.load_retired,
                h.successor_retired, published, kExpected & 0x00ffff);
    return pass ? 0 : 1;
}
