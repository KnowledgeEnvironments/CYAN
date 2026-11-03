// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 WAIT processing-state boundary oracle
//
// Motorola defines WAIT as aborting the prefetched successor, stopping core
// retirement, and waking only for an unmasked interrupt or hardware reset.
// On an interrupt wake, the vector executes before the aborted successor is
// fetched again and retired.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint8_t kHaltState = 5;
constexpr uint16_t kIrqaVector = 0x0008;
constexpr uint16_t kSsiTxVector = 0x0010;
constexpr uint16_t kHandler = 0x0020;
constexpr uint16_t kSuccessorTarget = 0x0040;
constexpr uint32_t kWaitOpcode = 0x000086;

class WaitHarness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};
    unsigned vector_first_fetches = 0;
    unsigned vector_second_fetches = 0;
    unsigned handler_fetches = 0;
    unsigned successor_target_fetches = 0;
    bool successor_before_vector = false;
    unsigned ticks = 0;
    unsigned first_vector_tick = 0;
    uint16_t vector = kIrqaVector;

    explicit WaitHarness(bool internal_pending = false) {
        vector = internal_pending ? kSsiTxVector : kIrqaVector;
        p[0x00] = kWaitOpcode; // WAIT
        p[0x01] = 0x0af080; // JMP >$0040 -- architectural successor
        p[0x02] = kSuccessorTarget;
        p[vector] = 0x0bf080; // long vector: JSR >$0020
        p[vector + 1] = kHandler;
        p[kHandler] = 0x000004; // RTI
        p[kSuccessorTarget] = 0x0af080; // stable loop at successor target
        p[kSuccessorTarget + 1] = kSuccessorTarget;

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
        core.irq_priority = internal_pending ? 0x3000 : 0x0003;
        core.dbg_reg_wr_en = 0;
        core.dbg_reg_wr_addr = 0;
        core.dbg_reg_wr_data = 0;

        for (unsigned i = 0; i < 4; ++i)
            tick();
        core.rst_n = 1;

        // Hardware reset leaves the mask at IPL 3.  Lower it before starting
        // so the configured external interrupt is serviceable.
        core.dbg_reg_wr_en = 1;
        core.dbg_reg_wr_addr = 0x39;
        core.dbg_reg_wr_data = 0;
        tick();
        core.dbg_reg_wr_en = 0;
        core.run = 1;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;

        if (fetch_en) {
            if (fetch_addr == vector) {
                ++vector_first_fetches;
                if (first_vector_tick == 0)
                    first_vector_tick = ticks;
                core.irq_a = 0;
                core.irq_ssi_tx = 0;
            } else if (fetch_addr == vector + 1) {
                ++vector_second_fetches;
            } else if (fetch_addr == kHandler) {
                ++handler_fetches;
            } else if (fetch_addr == kSuccessorTarget) {
                if (vector_first_fetches == 0)
                    successor_before_vector = true;
                ++successor_target_fetches;
            }
        }

        core.clk = 1;
        core.eval();
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 0xff] & 0x00ffffff;
        ++ticks;
    }

    void run_cycles(unsigned count) {
        for (unsigned i = 0; i < count; ++i)
            tick();
    }
};

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    WaitHarness h;

    h.run_cycles(60);
    const bool held = h.core.state_out == kHaltState &&
                      h.vector_first_fetches == 0 &&
                      h.successor_target_fetches == 0;

    h.core.irq_a = 1;
    h.run_cycles(120);
    const bool woke = h.vector_first_fetches == 1 &&
                      h.vector_second_fetches == 1 &&
                      h.handler_fetches == 1 &&
                      h.successor_target_fetches != 0 &&
                      !h.successor_before_vector;
    const bool delayed_wake_pass = held && woke;
    std::printf("%s WAIT interrupt boundary: held=%u state=%u "
                "vector=%u/%u handler=%u successor=%u early=%u\n",
                delayed_wake_pass ? "PASS" : "FAIL", held, h.core.state_out,
                h.vector_first_fetches, h.vector_second_fetches,
                h.handler_fetches, h.successor_target_fetches,
                h.successor_before_vector);

    // DSP56001UMA3 Table A-6 note 2: an internal interrupt that becomes
    // pending while WAIT executes must not bypass WAIT's minimum 16
    // oscillator-clock execution interval. Assert SSI TX while the WAIT word
    // is in decode, then measure from that execution clock to vector fetch.
    WaitHarness pending(true);
    unsigned pending_tick = 0;
    for (unsigned guard = 0; guard < 80; ++guard) {
        if (pending.core.state_out == 2 &&
            pending.core.p_fetch_data == kWaitOpcode) {
            pending.core.irq_ssi_tx = 1;
            pending_tick = pending.ticks;
            break;
        }
        pending.tick();
    }
    while (pending.vector_first_fetches == 0 &&
           pending.ticks < pending_tick + 80)
        pending.tick();
    const unsigned pending_latency = pending.first_vector_tick - pending_tick;
    const bool pending_wait_pass = pending_tick != 0 &&
                                   pending.vector_first_fetches == 1 &&
                                   pending_latency >= 16;
    std::printf("%s WAIT pending-internal timing: asserted=%u vector=%u "
                "latency=%u minimum=16 state=%u\n",
                pending_wait_pass ? "PASS" : "FAIL", pending_tick,
                pending.first_vector_tick, pending_latency,
                pending.core.state_out);

    return delayed_wake_pass && pending_wait_pass ? 0 : 1;
}

