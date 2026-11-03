// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 production-core multiply selector matrix
//
// DSP56001UMA3 Table A-28 defines all eight QQQ multiplier source pairs.
// Exercise every pair through MPY, MPYR, MAC and MACR, both accumulators and
// both signs.  Operands are distinct aligned positive fractions so every pair
// has a unique exact product and rounded forms do not change that product.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint8_t kStateWritebackFetch = 4;
constexpr uint64_t kMask56 = 0x00ffffffffffffffULL;

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 8> p{};
    uint32_t target_instruction;
    unsigned cycles = 0;
    int64_t decode_cycle = -1;
    int64_t writeback_cycle = -1;
    unsigned x_reads = 0;
    unsigned x_writes = 0;
    unsigned y_reads = 0;
    unsigned y_writes = 0;

    explicit Harness(uint32_t instruction) : target_instruction(instruction) {
        p[0] = instruction;
        p[1] = 0x0af080; // JMP >$0001.
        p[2] = 0x000001;

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
        core.irq_sci_rx_exception = 0;
        core.irq_sci_idle = 0;
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
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        if (core.state_out == 2 &&
            (core.p_fetch_data & 0x00ffffff) == target_instruction &&
            decode_cycle < 0)
            decode_cycle = cycles;
        if (core.state_out == kStateWritebackFetch &&
            core.cur_inst_out == target_instruction && writeback_cycle < 0)
            writeback_cycle = cycles;
        x_reads += core.x_rd_en;
        x_writes += core.x_wr_en;
        y_reads += core.y_rd_en;
        y_writes += core.y_wr_en;
        core.clk = 1;
        core.eval();
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 7] & 0x00ffffff;
        ++cycles;
    }

    void debug_write(uint8_t address, uint64_t value) {
        core.dbg_reg_wr_addr = address;
        core.dbg_reg_wr_data = value;
        core.dbg_reg_wr_en = 1;
        tick();
        core.dbg_reg_wr_en = 0;
    }

    bool retire(uint32_t instruction) {
        core.run = 1;
        for (unsigned i = 0; i < 80; ++i) {
            if (core.state_out == kStateWritebackFetch &&
                core.cur_inst_out == instruction) {
                tick();
                return true;
            }
            tick();
        }
        return false;
    }

    int64_t instruction_clocks() const {
        return decode_cycle >= 0 && writeback_cycle >= 0 ?
            writeback_cycle - decode_cycle + 1 : -1;
    }
};

struct Pair {
    const char *name;
    uint32_t lhs;
    uint32_t rhs;
};

bool run_case(unsigned qqq, unsigned operation, bool dest_b, bool negate,
              const Pair &pair) {
    const uint8_t low = 0x80 | (qqq << 4) | (dest_b ? 0x08 : 0) |
                        (negate ? 0x04 : 0) | operation;
    const uint32_t instruction = 0x200000 | low;
    Harness h(instruction);

    constexpr uint32_t kX0 = 0x100000;
    constexpr uint32_t kY0 = 0x180000;
    constexpr uint32_t kX1 = 0x200000;
    constexpr uint32_t kY1 = 0x280000;
    constexpr uint64_t kInitialA = 0x00000010000000ULL;
    constexpr uint64_t kInitialB = 0x00000020000000ULL;
    h.debug_write(0x04, kX0);
    h.debug_write(0x05, kX1);
    h.debug_write(0x06, kY0);
    h.debug_write(0x07, kY1);
    h.debug_write(0x0e, kInitialA);
    h.debug_write(0x0f, kInitialB);
    h.x_reads = h.x_writes = h.y_reads = h.y_writes = 0;

    const bool completed = h.retire(instruction);
    int64_t product = static_cast<int64_t>(pair.lhs) *
                      static_cast<int64_t>(pair.rhs) * 2;
    if (negate)
        product = -product;
    const bool accumulate = operation >= 2;
    const uint64_t initial_dest = dest_b ? kInitialB : kInitialA;
    const uint64_t expected_dest =
        (static_cast<uint64_t>(product) + (accumulate ? initial_dest : 0)) &
        kMask56;
    const uint64_t actual_dest = dest_b ? h.core.acc_b_out : h.core.acc_a_out;
    const uint64_t actual_other = dest_b ? h.core.acc_a_out : h.core.acc_b_out;
    const uint64_t expected_other = dest_b ? kInitialA : kInitialB;
    const bool bus_quiet = h.x_reads == 0 && h.x_writes == 0 &&
                           h.y_reads == 0 && h.y_writes == 0;
    const bool pass = completed && actual_dest == expected_dest &&
                      actual_other == expected_other && bus_quiet &&
                      h.instruction_clocks() == 2;
    static const char *kOperation[] = {"MPY", "MPYR", "MAC", "MACR"};
    std::printf(
        "%s MULTIPLY qqq=%u pair=%s op=%s dest=%c neg=%u "
        "result=%014llx/%014llx other=%014llx/%014llx "
        "bus=X%u/%u,Y%u/%u timing=%lld/2 cycles=%u\n",
        pass ? "PASS" : "FAIL", qqq, pair.name, kOperation[operation],
        dest_b ? 'B' : 'A', negate,
        static_cast<unsigned long long>(actual_dest),
        static_cast<unsigned long long>(expected_dest),
        static_cast<unsigned long long>(actual_other),
        static_cast<unsigned long long>(expected_other), h.x_reads,
        h.x_writes, h.y_reads, h.y_writes,
        static_cast<long long>(h.instruction_clocks()), h.cycles);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);

    // DSP56001UMA3 Table A-28 QQQ source order.
    constexpr uint32_t kX0 = 0x100000;
    constexpr uint32_t kY0 = 0x180000;
    constexpr uint32_t kX1 = 0x200000;
    constexpr uint32_t kY1 = 0x280000;
    const Pair pairs[] = {
        {"X0*X0", kX0, kX0}, {"Y0*Y0", kY0, kY0},
        {"X1*X0", kX1, kX0}, {"Y1*Y0", kY1, kY0},
        {"X0*Y1", kX0, kY1}, {"Y0*X0", kY0, kX0},
        {"X1*Y0", kX1, kY0}, {"Y1*X1", kY1, kX1},
    };

    unsigned failures = 0;
    unsigned cases = 0;
    for (unsigned qqq = 0; qqq < 8; ++qqq) {
        for (unsigned operation = 0; operation < 4; ++operation) {
            for (unsigned dest = 0; dest < 2; ++dest) {
                for (unsigned negate = 0; negate < 2; ++negate) {
                    ++cases;
                    if (!run_case(qqq, operation, dest, negate, pairs[qqq]))
                        ++failures;
                }
            }
        }
    }
    std::printf("MULTIPLY SELECTOR SUMMARY cases=%u failures=%u\n", cases,
                failures);
    return failures == 0 ? 0 : 1;
}
