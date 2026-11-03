// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 external P-fetch wait-state oracle
//
// DSP56001UMA3 Appendix A accounts for the programmed P-memory wait term on
// non-sequential instruction fetches.  The production core exposes
// p_fetch_ready, so a denied fetch must retain its exact PC and must not decode
// stale program data.  An interrupt arriving during the denied transaction
// must likewise remain pending without replacing the outstanding bus address;
// the original fetch is accepted first, then the vector is serviced.
//
// This fixture denies the first fetch for three oscillator clocks, then
// supplies MOVE #>$1234,R0 and its extension.  The request must remain at
// P:$0000, exactly one fetch may be accepted, no stale instruction may retire,
// and the accepted instruction must publish R0=$1234.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "Vdsp56001_core___024root.h"
#include "verilated.h"

namespace {

constexpr uint32_t kLoadR0 = 0x60f400; // MOVE #>$1234,R0
constexpr uint32_t kStale = 0x000000;  // NOP must never retire.
constexpr unsigned kDeniedClocks = 3;

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 16> p{};
    unsigned denied_clocks = 0;
    unsigned request_clocks = 0;
    unsigned accepted_fetches = 0;
    unsigned stale_retirements = 0;
    bool address_stable = true;
    bool fetch_started = false;
    bool load_retired = false;

    Harness() {
        p[0] = kLoadR0;
        p[1] = 0x001234;
        p[2] = 0x0c0002; // Stable terminal JMP.

        core.clk = 0;
        core.rst_n = 0;
        core.clock_enable = 1;
        core.halt = 0;
        core.run = 0;
        core.p_fetch_data = kStale;
        core.p_fetch_ready = 0;
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
        bool update_fetch_data = false;
        uint32_t fetch_data_next = kStale;

        if (observe && fetch_started && core.p_fetch_ready &&
            core.state_out == 4) {
            stale_retirements += core.cur_inst_out == kStale;
            load_retired |= core.cur_inst_out == kLoadR0;
        }

        if (observe && core.p_fetch_en) {
            fetch_started = true;
            const uint16_t address = core.p_fetch_addr;
            if (accepted_fetches == 0) {
                ++request_clocks;
                address_stable &= address == 0;
                if (address == 0 && denied_clocks < kDeniedClocks) {
                    ++denied_clocks;
                    core.p_fetch_ready = 0;
                    core.p_fetch_data = kStale;
                } else if (address == 0) {
                    core.p_fetch_ready = 1;
                    update_fetch_data = true;
                    fetch_data_next = p[address & 15] & 0x00ffffff;
                    ++accepted_fetches;
                } else {
                    core.p_fetch_ready = 0;
                    core.p_fetch_data = kStale;
                }
            } else {
                core.p_fetch_ready = 1;
                update_fetch_data = true;
                fetch_data_next = p[address & 15] & 0x00ffffff;
            }
            core.eval();
        }

        core.clk = 1;
        core.eval();
        if (update_fetch_data)
            core.p_fetch_data = fetch_data_next;
    }
};

class InterruptWaitHarness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 64> p{};
    unsigned denied_clocks = 0;
    unsigned accepted_fetches = 0;
    unsigned vector_accepts = 0;
    uint16_t first_accept_addr = 0xffff;
    bool denied_address_stable = true;

    InterruptWaitHarness() {
        p[0x00] = 0x000000; // NOP whose successor fetch is initially denied.
        p[0x01] = 0x0af080; // JMP >$0001
        p[0x02] = 0x000001;
        p[0x08] = 0x0bf080; // IRQA long vector: JSR >$0020
        p[0x09] = 0x000020;
        p[0x20] = 0x000004; // RTI

        core.clk = 0;
        core.rst_n = 0;
        core.clock_enable = 1;
        core.halt = 0;
        core.run = 0;
        core.p_fetch_data = kStale;
        core.p_fetch_ready = 0;
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

        // Reset masks external interrupts.  Unmask them before starting the
        // program without changing interrupt-controller state.
        core.dbg_reg_wr_en = 1;
        core.dbg_reg_wr_addr = 0x39;
        core.dbg_reg_wr_data = 0;
        tick(false);
        core.dbg_reg_wr_en = 0;
        core.irq_priority = 0x07; // IRQA edge mode, level code 3.
    }

    void tick(bool observe = true) {
        core.clk = 0;
        core.eval();
        bool update_fetch_data = false;
        uint32_t fetch_data_next = kStale;

        if (observe && core.p_fetch_en) {
            const uint16_t requested_addr = core.p_fetch_addr;
            if (denied_clocks < kDeniedClocks) {
                core.p_fetch_ready = 0;
                denied_address_stable &= requested_addr == 0;
                ++denied_clocks;
                core.eval();
            } else {
                core.p_fetch_ready = 1;
                core.eval();
                // Ready can affect the core's combinational arbitration.  The
                // address observed after reevaluation is the one accepted on
                // the rising edge and therefore must also remain stable.
                const uint16_t accepted_addr = core.p_fetch_addr;
                if (accepted_fetches == 0)
                    first_accept_addr = accepted_addr;
                ++accepted_fetches;
                vector_accepts += accepted_addr == 0x0008;
                update_fetch_data = true;
                fetch_data_next = p[accepted_addr & 0x3f] & 0x00ffffff;
            }
        }

        core.clk = 1;
        core.eval();
        if (update_fetch_data)
            core.p_fetch_data = fetch_data_next;
    }
};

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Harness h;
    h.core.run = 1;

    for (unsigned i = 0; i < 200 && !h.load_retired; ++i)
        h.tick();

    const uint16_t r0 =
        h.core.rootp->dsp56001_core__DOT__r_reg[0];
    const bool pass = h.denied_clocks == kDeniedClocks &&
                      h.request_clocks == kDeniedClocks + 1 &&
                      h.accepted_fetches == 1 && h.address_stable &&
                      h.stale_retirements == 0 && h.load_retired &&
                      r0 == 0x1234;

    std::printf("%s external P fetch wait denied=%u/%u requests=%u/%u "
                "accepted=%u address=%u stale-retire=%u load-retired=%u "
                "R0=%04x/1234\n",
                pass ? "PASS" : "FAIL", h.denied_clocks, kDeniedClocks,
                h.request_clocks, kDeniedClocks + 1, h.accepted_fetches,
                h.address_stable, h.stale_retirements, h.load_retired, r0);

    InterruptWaitHarness irq;
    irq.core.run = 1;
    while (irq.denied_clocks < 1)
        irq.tick();
    irq.core.irq_a = 1;
    irq.tick();
    irq.core.irq_a = 0;
    for (unsigned i = 0; i < 200 && irq.vector_accepts == 0; ++i)
        irq.tick();

    const bool irq_pass = irq.denied_clocks == kDeniedClocks &&
                          irq.denied_address_stable &&
                          irq.first_accept_addr == 0 &&
                          irq.vector_accepts == 1;
    std::printf("%s interrupt during P wait denied=%u/%u stable=%u "
                "first-accepted=%04x/0000 vector=%u/1 total-accepted=%u\n",
                irq_pass ? "PASS" : "FAIL", irq.denied_clocks,
                kDeniedClocks, irq.denied_address_stable,
                irq.first_accept_addr, irq.vector_accepts,
                irq.accepted_fetches);
    return pass && irq_pass ? 0 : 1;
}
