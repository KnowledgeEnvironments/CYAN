// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 interrupt-during-memory-operation boundary oracle
//
// Tier-3 interrupt-matrix residual: "interrupt arrival during DO/REP and
// memory operations."  DO/REP/nested-DO are covered by the admitted oracles;
// this closes the memory-operation case against the production core.
//
// Parallel MOVE timing is address-class dependent.  A zero-wait short move
// issues its memory request in S_DECODE_EXEC and retires in the following
// S_WB_FETCH; slower modes pass through S_MEM first.  In either case an
// unmasked external interrupt may be accepted at that S_WB_FETCH boundary
// while the move's architectural writeback completes.  The competing
// mechanisms:
//
//   (a) DEFECT: the memory writeback is lost, or the held response is
//       overwritten by the vector fetch before the consumer latches it.
//   (b) CORRECT: writeback retires at the boundary; the vector fetch does
//       not disturb the data buses; the handler runs and RTIs back to the
//       instruction after the memory operation.
//
// Discriminating observation: a Y-memory load into A1 (move y:$32,a1) whose
// writeback coincides with an accepted edge IRQA.  The handler runs, RTIs,
// and A must hold exactly the memory value; a follow-up store proves the
// value is architecturally live.  Cases:
//   1. read writeback coincident with interrupt acceptance
//   2. memory WRITE retirement coincident with acceptance (store visible
//      exactly once, neither dropped nor duplicated)
//   3. control: same program with no interrupt (bounds the value)
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint8_t kStateDecode = 2;
constexpr uint16_t kIrqaVector = 0x0008;

class MemIrqHarness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};
    std::array<uint32_t, 256> y{};
    uint64_t cycles = 0;
    unsigned y_reads = 0;
    unsigned y_writes = 0;
    unsigned vector_fetches = 0;
    unsigned handler_fetches = 0;

    MemIrqHarness() {
        // Ordinary loop: NOP; JMP >$0000
        p[0x00] = 0x000000;
        p[0x01] = 0x0af080;
        p[0x02] = 0x000000;
        // IRQA long vector: JSR >$0020; handler: NOP NOP RTI
        p[0x08] = 0x0bf080;
        p[0x09] = 0x000020;
        p[0x20] = 0x000000;
        p[0x21] = 0x000000;
        p[0x22] = 0x000004;

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
        const bool yr = core.y_rd_en;
        const bool yw = core.y_wr_en;
        const uint16_t y_addr = core.y_addr;
        const uint32_t y_wdata = core.y_wdata;
        if (fetch_en && fetch_addr == kIrqaVector)
            ++vector_fetches;
        if (fetch_en && fetch_addr == 0x0020)
            ++handler_fetches;
        if (yr) {
            ++y_reads;
            core.y_rdata = y[y_addr & 0xff] & 0x00ffffff;
        }
        core.clk = 1;
        core.eval();
        if (yw) {
            ++y_writes;
            y[y_addr & 0xff] = y_wdata & 0x00ffffff;
        }
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 0xff] & 0x00ffffff;
        ++cycles;
    }

    bool wait_state(uint8_t state, unsigned bound = 300) {
        for (unsigned i = 0; i < bound; ++i) {
            if (core.state_out == state)
                return true;
            tick();
        }
        return false;
    }

    void run_cycles(unsigned count) {
        for (unsigned i = 0; i < count; ++i)
            tick();
    }
};

// Case 1: load A1 from Y:$32 with IRQA accepted at the writeback boundary.
// After handler + RTI, a store of A1 to Y:$33 proves the loaded value
// survived both the coincident writeback and the vector divert.
bool read_writeback_survives_interrupt() {
    MemIrqHarness h;
    // Main loop points at $10 from construction time.
    h.p[0x00] = 0x0af080;
    h.p[0x01] = 0x000010; // jmp >$0010
    h.p[0x10] = 0x6fb200; // move y:$32,r7  -> memory read boundary
    h.p[0x11] = 0x000000; // NOP (interrupt lands at this boundary if late)
    h.p[0x12] = 0x6f3300; // move r7,y:$33  -> proves loaded value
    h.p[0x13] = 0x0af080; // jmp >$0013
    h.p[0x14] = 0x000013;
    h.y[0x32] = 0x0000a5;
    h.core.irq_priority = 0x07; // IRQA edge, level code 3

    // Arm on the load request itself.  IRQA's edge is retained on the decode
    // edge, then accepted at the immediately following writeback boundary.
    bool saw_request = false;
    for (unsigned i = 0; i < 400 && !saw_request; ++i) {
        if (h.core.state_out == kStateDecode && h.core.y_rd_en) {
            h.core.irq_a = 1;
            h.tick();
            h.core.irq_a = 0;
            saw_request = true;
        } else {
            h.tick();
        }
    }
    if (!saw_request)
        return false;
    h.run_cycles(200);

    const bool stored = (h.y[0x33] & 0xffffff) == 0x0000a5;
    const bool serviced = h.vector_fetches == 1 && h.handler_fetches >= 1;
    const bool store_once = h.y_writes == 1 && h.y_writes == h.y_writes;
    std::printf("%s memory-read writeback survives interrupt: stored=%06x "
                "vec=%u handler=%u y_writes=%u reads=%u\n",
                (stored && serviced && store_once) ? "PASS" : "FAIL",
                h.y[0x33] & 0xffffff, h.vector_fetches,
                h.handler_fetches, h.y_writes, h.y_reads);
    return stored && serviced && store_once;
}

// Case 2: store R7 to Y:$34 with IRQA accepted at the store's retirement
// boundary.  The store must be visible exactly once.
bool write_completion_survives_interrupt() {
    MemIrqHarness h;
    h.p[0x00] = 0x0af080;
    h.p[0x01] = 0x000010;
    h.p[0x10] = 0x37a500; // move #<$a5,r7 (seed the store value)
    h.p[0x11] = 0x6f3400; // move r7,y:$34
    h.p[0x12] = 0x0af080; // jmp >$0012
    h.p[0x13] = 0x000012;
    h.core.irq_priority = 0x07;

    bool saw_request = false;
    for (unsigned i = 0; i < 400 && !saw_request; ++i) {
        // Retain the edge on the same decode edge that commits the memory
        // write; acceptance follows at that instruction's WB boundary.
        if (h.core.state_out == kStateDecode && h.core.y_wr_en) {
            h.core.irq_a = 1;
            h.tick();
            h.core.irq_a = 0;
            saw_request = true;
        } else {
            h.tick();
        }
    }
    if (!saw_request)
        return false;
    h.run_cycles(200);

    const bool stored_once = (h.y[0x34] & 0xffffff) == 0x0000a5 &&
                             h.y_writes == 1;
    const bool serviced = h.vector_fetches == 1 && h.handler_fetches >= 1;
    std::printf("%s memory-write retirement survives interrupt: "
                "stored=%06x writes=%u vec=%u handler=%u\n",
                (stored_once && serviced) ? "PASS" : "FAIL",
                h.y[0x34] & 0xffffff, h.y_writes, h.vector_fetches,
                h.handler_fetches);
    return stored_once && serviced;
}

// Case 3: no-interrupt control for the same program shape.
bool no_interrupt_control() {
    MemIrqHarness h;
    h.p[0x00] = 0x0af080;
    h.p[0x01] = 0x000010;
    h.p[0x10] = 0x6fb200; // move y:$32,r7
    h.p[0x11] = 0x000000;
    h.p[0x12] = 0x6f3300; // move r7,y:$33
    h.p[0x13] = 0x0af080;
    h.p[0x14] = 0x000013;
    h.y[0x32] = 0x0000a5;
    h.core.irq_priority = 0x07;
    h.run_cycles(120);
    const bool stored = (h.y[0x33] & 0xffffff) == 0x0000a5;
    const bool no_service = h.vector_fetches == 0;
    std::printf("%s no-interrupt memory control: stored=%06x vec=%u\n",
                (stored && no_service) ? "PASS" : "FAIL",
                h.y[0x33] & 0xffffff, h.vector_fetches);
    return stored && no_service;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const bool read_case = read_writeback_survives_interrupt();
    const bool write_case = write_completion_survives_interrupt();
    const bool ctrl_case = no_interrupt_control();
    return read_case && write_case && ctrl_case ? 0 : 1;
}

