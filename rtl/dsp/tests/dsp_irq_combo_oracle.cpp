// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 combined peripheral/external interrupt arrival matrix oracle
//
// D1 residual: "peripheral-source arrival combinations" — the admitted SSI
// oracle covers single-source vectors, one same-level pair, and one
// higher-SSI pair; the external oracle covers A-vs-B.  This closes the
// multi-source combination space against the production core:
//
//   1. All-contenders simultaneous arrival (IRQA + IRQB + SSI RX + SSI TX
//      [+ exceptions]) with mixed IPLs: strict highest-IPL-first service,
//      Motorola within-level order (IRQA, IRQB, RX-exception, RX, TX-
//      exception, TX), every serviceable source serviced exactly once,
//      no lost request.
//   2. Level sweep: for every (a_lvl, b_lvl, ssi_lvl) in {0..2}^3 with all
//      four requests asserted at once: service count == number of distinct
//      serviceable sources, order strictly non-increasing in IPL with the
//      documented tie order.
//   3. SSI exception override: RX/TX exception vectors selected when the
//      exception inputs are coincident, including under external
//      contention.
//
// Fail-closed: per-case exact ordered vector sequence and total count.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint8_t kStateDecode = 2;
constexpr uint16_t kIrqaVector = 0x0008;
constexpr uint16_t kIrqbVector = 0x000a;
constexpr uint16_t kRxVector = 0x000c;
constexpr uint16_t kRxExcVector = 0x000e;
constexpr uint16_t kTxVector = 0x0010;
constexpr uint16_t kTxExcVector = 0x0012;

// Within-level fixed order rank (lower = earlier): A, B, RX-exc, RX, TX-exc, TX.
unsigned rank_of(uint16_t v) {
    switch (v) {
    case kIrqaVector: return 0;
    case kIrqbVector: return 1;
    case kRxExcVector: return 2;
    case kRxVector: return 3;
    case kTxExcVector: return 4;
    case kTxVector: return 5;
    }
    return 9;
}

class ComboHarness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};
    uint64_t cycles = 0;
    std::array<uint16_t, 16> order{};
    unsigned order_n = 0;

    ComboHarness() {
        // Main loop: NOP; JMP >$0000
        p[0x00] = 0x000000;
        p[0x01] = 0x0af080;
        p[0x02] = 0x000000;
        // All five vectors: two-word JSR to distinct handlers; each handler
        // is NOP;RTI (fast enough to chain services quickly).
        for (unsigned i = 0; i < 5; ++i) {
            const uint16_t vec = kIrqaVector + 2 * i;
            const uint16_t hnd = 0x0040 + 2 * i;
            p[vec] = 0x0bf080;
            p[vec + 1] = hnd;
            p[hnd] = 0x000000;
            p[hnd + 1] = 0x000004; // RTI
        }

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
        core.dbg_reg_wr_addr = 0x39;
        core.dbg_reg_wr_data = 0; // SR mask 0
        tick();
        core.dbg_reg_wr_en = 0;
        core.run = 1;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        bool accept_rx = false, accept_tx = false;
        if (fetch_en && fetch_addr >= kIrqaVector && fetch_addr <= kTxExcVector &&
            ((fetch_addr - kIrqaVector) & 1) == 0) {
            if (order_n < order.size())
                order[order_n++] = fetch_addr;
            // Model the handler clearing its own source (e.g. reading RX
            // drops RDF): deassert the SSI level when its vector is taken.
            // Externals are edge-latched and self-clear after the second
            // vector fetch.  This is the fixed-priority contract under
            // handler-cleared sources; a held dominant level starving an
            // equal-level subordinate is separately legitimate Motorola
            // fixed-priority behavior, out of scope here.
            if (fetch_addr == kRxVector || fetch_addr == kRxExcVector)
                accept_rx = true;
            if (fetch_addr == kTxVector || fetch_addr == kTxExcVector)
                accept_tx = true;
        }
        core.clk = 1;
        core.eval();
        if (accept_rx) {
            core.irq_ssi_rx = 0;
            core.irq_ssi_rx_exception = 0;
        }
        if (accept_tx) {
            core.irq_ssi_tx = 0;
            core.irq_ssi_tx_exception = 0;
        }
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 0xff] & 0x00ffffff;
        ++cycles;
    }

    void run_cycles(unsigned count) {
        for (unsigned i = 0; i < count; ++i)
            tick();
    }

    // irq_priority packing: A [1:0], A-edge [2], B [4:3], B-edge [5],
    // SSI [13:12].  All edge mode for externals.
    void set_levels(unsigned al, unsigned bl, unsigned sl) {
        core.irq_priority = static_cast<uint16_t>(
            (al & 3) | 0x4 | ((bl & 3) << 3) | 0x20 | ((sl & 3) << 12));
    }

    void assert_all(bool rx_exc, bool tx_exc) {
        for (unsigned i = 0; i < 200; ++i) {
            if (core.state_out == kStateDecode)
                break;
            tick();
        }
        // External A/B are edge-latched: a one-tick pulse suffices.  SSI
        // requests are LEVEL inputs: assert them and let the harness's
        // handler-clear model deassert each when its vector is taken.
        core.irq_a = 1;
        core.irq_b = 1;
        core.irq_ssi_rx = 1;
        core.irq_ssi_tx = 1;
        core.irq_ssi_rx_exception = rx_exc ? 1 : 0;
        core.irq_ssi_tx_exception = tx_exc ? 1 : 0;
        tick();
        core.irq_a = 0;
        core.irq_b = 0;
    }
};

// Expected service order for a level assignment:
//   contenders: A@al, B@bl, (RX or RX-exc)@sl, (TX or TX-exc)@sl
//   serviceable iff level > 0 (SR mask is 0).
//   Order: descending level; within a level: A, B, RX-exc, RX, TX-exc, TX.
// Exception inputs replace the normal vectors for that source.
bool check_case(unsigned al, unsigned bl, unsigned sl,
                bool rx_exc, bool tx_exc, const char *label) {
    ComboHarness h;
    h.set_levels(al, bl, sl);
    h.run_cycles(6);
    h.assert_all(rx_exc, tx_exc);
    h.run_cycles(200);

    struct Contender { unsigned level; unsigned rank; uint16_t vec; };
    Contender cs[4];
    unsigned n = 0;
    if (al) cs[n++] = {al, rank_of(kIrqaVector), kIrqaVector};
    if (bl) cs[n++] = {bl, rank_of(kIrqbVector), kIrqbVector};
    if (sl) cs[n++] = {sl, rank_of(rx_exc ? kRxExcVector : kRxVector),
                       rx_exc ? kRxExcVector : kRxVector};
    if (sl) cs[n++] = {sl, rank_of(tx_exc ? kTxExcVector : kTxVector),
                       tx_exc ? kTxExcVector : kTxVector};

    // Stable sort by (level desc, rank asc).
    for (unsigned i = 0; i < n; ++i)
        for (unsigned j = i + 1; j < n; ++j)
            if (cs[j].level > cs[i].level ||
                (cs[j].level == cs[i].level && cs[j].rank < cs[i].rank)) {
                Contender t = cs[i]; cs[i] = cs[j]; cs[j] = t;
            }

    bool ok = h.order_n == n;
    if (ok)
        for (unsigned i = 0; i < n; ++i)
            if (h.order[i] != cs[i].vec) { ok = false; break; }

    if (!ok) {
        std::printf("FAIL %s al=%u bl=%u sl=%u rx_exc=%u tx_exc=%u: "
                    "got %u [", label, al, bl, sl, rx_exc, tx_exc, h.order_n);
        for (unsigned i = 0; i < h.order_n && i < 8; ++i)
            std::printf("%s%04x", i ? "," : "", h.order[i]);
        std::printf("] exp %u [", n);
        for (unsigned i = 0; i < n && i < 8; ++i)
            std::printf("%s%04x", i ? "," : "", cs[i].vec);
        std::printf("]\n");
    }
    return ok;
}

bool full_matrix() {
    unsigned cases = 0, fails = 0;
    for (unsigned al = 0; al <= 2; ++al)
        for (unsigned bl = 0; bl <= 2; ++bl)
            for (unsigned sl = 0; sl <= 2; ++sl)
                for (unsigned exc = 0; exc < 4; ++exc) {
                    if (!check_case(al, bl, sl, exc & 1, exc & 2, "matrix"))
                        ++fails;
                    ++cases;
                }
    const bool pass = fails == 0;
    std::printf("%s combined arrival matrix: %u cases, %u fails\n",
                pass ? "PASS" : "FAIL", cases, fails);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const bool matrix = full_matrix();
    return matrix ? 0 : 1;
}

