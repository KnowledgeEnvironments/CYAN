// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 external-interrupt priority/mask/arrival matrix oracle
//
// D1 residual: "exhaustive priority/mask/arrival combinations" beyond the
// admitted one-two-point edge-boundary cases.  This oracle walks the full
// small matrix against the production core:
//
//   for a_level in {1,2,3} (IPL0..IPL2), b_level in {1,2,3}, sr_mask in
//   {0,1,2,3}:
//     - simultaneous edge arrival at one instruction boundary:
//         service happens iff max(serviceable levels) exists;
//         the serviced vector is the highest serviceable level (A wins ties);
//         unserviced (masked or losing-equal) edges stay pending and run
//         after RTI if unmasked then.
//     - staggered arrival: lower-IPL edge arrives first and is accepted;
//         higher-IPL edge arrives mid-handler -> preempts iff its level
//         exceeds the raised mask (= accepted level).  Equal-or-lower stays
//         pending.
//
// Fail-closed: every case asserts the exact vector fetch count and the
// winner identity observed on the real fetch bus; no interrupt behavior is
// modeled in the harness.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint8_t kStateDecode = 2;

class PrioHarness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};
    uint64_t cycles = 0;
    unsigned a_vec = 0;
    unsigned b_vec = 0;
    std::array<uint16_t, 8> order{};
    unsigned order_n = 0;

    PrioHarness() {
        p[0x00] = 0x000000;         // NOP
        p[0x01] = 0x0af080;         // JMP >$0000
        p[0x02] = 0x000000;
        p[0x08] = 0x0bf080;         // IRQA: JSR >$0020
        p[0x09] = 0x000020;
        p[0x0a] = 0x0bf080;         // IRQB: JSR >$0030
        p[0x0b] = 0x000030;
        p[0x20] = 0x000000;         // handler A: NOP NOP RTI
        p[0x21] = 0x000000;
        p[0x22] = 0x000004;
        p[0x30] = 0x000000;         // handler B: NOP NOP RTI
        p[0x31] = 0x000000;
        p[0x32] = 0x000004;

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
        core.dbg_reg_wr_en = 1;
        core.dbg_reg_wr_addr = 0x39; // SR
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
            if (fetch_addr == 0x0008)
                ++a_vec;
            else if (fetch_addr == 0x000a)
                ++b_vec;
            if (order_n < order.size() &&
                (fetch_addr == 0x0008 || fetch_addr == 0x000a ||
                 fetch_addr == 0x0022))
                order[order_n++] = fetch_addr;
        }
        core.clk = 1;
        core.eval();
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 0xff] & 0x00ffffff;
        ++cycles;
    }

    void run_cycles(unsigned count) {
        for (unsigned i = 0; i < count; ++i)
            tick();
    }

    void set_mask(unsigned mask) {
        // SR write through the debug boundary; preserves other bits as zero.
        core.dbg_reg_wr_en = 1;
        core.dbg_reg_wr_addr = 0x39;
        core.dbg_reg_wr_data = static_cast<uint32_t>(mask) << 8;
        tick();
        core.dbg_reg_wr_en = 0;
    }

    // irq_priority packing: [1:0]=A level, [2]=A edge, [4:3]=B level,
    // [5]=B edge.  Edge mode for both.
    void set_levels(unsigned al, unsigned bl) {
        core.irq_priority =
            static_cast<uint16_t>((al & 3) | 0x4 | ((bl & 3) << 3) | 0x20);
    }

    void pulse_simultaneous() {
        for (unsigned i = 0; i < 200; ++i) {
            if (core.state_out == kStateDecode)
                break;
            tick();
        }
        core.irq_a = 1;
        core.irq_b = 1;
        tick();
        core.irq_a = 0;
        core.irq_b = 0;
    }
};

// Full simultaneous-arrival matrix.  Expected behavior:
//   a_srv = a_level > mask; b_srv = b_level > mask;
//   any serviced -> winner = max serviceable level, A on tie;
//   after the winner's RTI the mask returns to the configured value, so a
//   still-pending loser services in the same run iff its level > mask.
bool simultaneous_matrix() {
    unsigned cases = 0, fails = 0;
    for (unsigned al = 1; al <= 3; ++al) {
        for (unsigned bl = 1; bl <= 3; ++bl) {
            for (unsigned mask = 0; mask <= 3; ++mask) {
                PrioHarness h;
                h.set_levels(al, bl);
                if (mask)
                    h.set_mask(mask);
                h.run_cycles(6);
                h.pulse_simultaneous();
                h.run_cycles(120);

                const bool a_srv = al > mask;
                const bool b_srv = bl > mask;
                // Winner at the first boundary: highest serviceable level,
                // A wins ties.  Derived from the observed first vector
                // fetch in `order`, not from settled totals.
                const bool expect_a_first = a_srv && (!b_srv || al >= bl);
                const uint16_t expect_first = expect_a_first ? 0x0008 : 0x000a;
                const bool any_srv = a_srv || b_srv;

                // Settled totals: every serviceable request services
                // exactly once (loser after the winner's RTI restores the
                // configured mask, which is below both levels).
                const unsigned exp_a = a_srv ? 1 : 0;
                const unsigned exp_b = b_srv ? 1 : 0;

                const bool ok_order = !any_srv ||
                    (h.order_n >= 1 && h.order[0] == expect_first);
                const bool ok_totals = h.a_vec == exp_a && h.b_vec == exp_b;
                const bool ok = ok_order && ok_totals;
                if (!ok) {
                    ++fails;
                    std::printf(
                        "FAIL sim al=%u bl=%u mask=%u: a_vec=%u(exp %u) "
                        "b_vec=%u(exp %u) first=%04x(exp %04x)\n",
                        al, bl, mask, h.a_vec, exp_a, h.b_vec, exp_b,
                        h.order_n ? h.order[0] : 0xffff, expect_first);
                }
                ++cases;
            }
        }
    }
    const bool pass = fails == 0;
    std::printf("%s simultaneous priority/mask matrix: %u cases, %u fails\n",
                pass ? "PASS" : "FAIL", cases, fails);
    return pass;
}

// Staggered arrival: lower/equal-IPL A accepted first; higher-IPL B arrives
// inside the handler.  B preempts iff bl > al (the raised mask); otherwise
// it stays pending and services after the outer RTI iff bl > configured
// mask.
bool staggered_matrix() {
    unsigned cases = 0, fails = 0;
    for (unsigned al = 1; al <= 2; ++al) {
        for (unsigned bl = 1; bl <= 3; ++bl) {
            PrioHarness h;
            h.set_levels(al, bl);
            h.run_cycles(6);
            // Accept A first.
            for (unsigned i = 0; i < 200; ++i) {
                if (h.core.state_out == kStateDecode)
                    break;
                h.tick();
            }
            h.core.irq_a = 1;
            h.tick();
            h.core.irq_a = 0;
            // Run only until A's handler has ENTERED (first handler fetch),
            // then land B inside the handler body — before A's RTI.
            for (unsigned i = 0; i < 100; ++i) {
                h.tick();
                if (h.order_n >= 1 && h.order[0] == 0x0008)
                    break;
            }
            // A few more cycles put us inside the handler (after its first
            // instruction fetch), still well before RTI.
            h.run_cycles(4);
            const bool in_a = h.order_n >= 1 && h.order[0] == 0x0008;
            // Now B arrives mid-handler.
            for (unsigned i = 0; i < 200; ++i) {
                if (h.core.state_out == kStateDecode)
                    break;
                h.tick();
            }
            h.core.irq_b = 1;
            h.tick();
            h.core.irq_b = 0;
            h.run_cycles(150);

            // Expectations: B preempts iff bl > al (the running mask is the
            // accepted A level).  Order marks: 0x0008 (A vector), 0x000a
            // (B vector), 0x0022 (A-handler RTI fetch).
            const bool b_preempts = bl > al;
            bool ok = h.a_vec == 1 && h.b_vec == 1 && in_a;
            if (b_preempts) {
                // B's vector fetch must appear BEFORE A's RTI fetch.
                bool saw_b = false, b_before_rti = false;
                for (unsigned i = 0; i < h.order_n; ++i) {
                    if (h.order[i] == 0x000a)
                        saw_b = true;
                    if (h.order[i] == 0x0022 && saw_b)
                        b_before_rti = true;
                }
                ok = ok && b_before_rti;
            } else {
                // B defers: A's RTI (0x0022) precedes B's vector (0x000a).
                bool saw_rti = false, rti_before_b = false;
                for (unsigned i = 0; i < h.order_n; ++i) {
                    if (h.order[i] == 0x0022)
                        saw_rti = true;
                    if (h.order[i] == 0x000a && saw_rti)
                        rti_before_b = true;
                }
                ok = ok && rti_before_b;
            }
            if (!ok) {
                ++fails;
                std::printf("FAIL stagger al=%u bl=%u: a_vec=%u b_vec=%u "
                            "order_n=%u\n",
                            al, bl, h.a_vec, h.b_vec, h.order_n);
            }
            ++cases;
        }
    }
    const bool pass = fails == 0;
    std::printf("%s staggered arrival matrix: %u cases, %u fails\n",
                pass ? "PASS" : "FAIL", cases, fails);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const bool sim = simultaneous_matrix();
    const bool stag = staggered_matrix();
    return sim && stag ? 0 : 1;
}

