// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 REP/DO production-core timing oracle
//
// DSP56001UMA3 Table A-6 requires REP to occupy 4+mv oscillator clocks, DO to
// occupy 6+mv, and ENDDO to occupy two clocks. Tables A-7/A-13 make internal
// short and ordinary indirect memory sources zero-cost, while indexed and
// predecrement effective addresses add two clocks. Every case below also
// executes a count of three
// and requires exactly three follower writes plus one successor write, so a
// timing-only no-op cannot pass.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint32_t kFollower = 0x6f3200;  // MOVE R7,Y:$32.
constexpr uint32_t kSuccessor = 0x6f3300; // MOVE R7,Y:$33.
constexpr uint32_t kJump = 0x0af080;

struct Case {
    const char *name;
    uint32_t instruction;
    bool is_do;
    unsigned expected_clocks;
    unsigned expected_x_reads;
    unsigned expected_y_reads;
    uint16_t r0;
};

constexpr std::array<Case, 15> kCases{{
    {"REP-immediate",    0x0603a0, false, 4, 0, 0, 0x0020},
    {"REP-register",     0x06d720, false, 4, 0, 0, 0x0020},
    {"REP-X-short",      0x062020, false, 4, 1, 0, 0x0020},
    {"REP-Y-short",      0x062060, false, 4, 0, 1, 0x0020},
    {"REP-X-direct",     0x066020, false, 4, 1, 0, 0x0020},
    {"REP-X-indexed",    0x066820, false, 6, 1, 0, 0x0020},
    {"REP-X-predecrement", 0x067820, false, 6, 1, 0, 0x0021},
    {"DO-immediate",     0x060380, true,  6, 0, 0, 0x0020},
    {"DO-register",      0x06d700, true,  6, 0, 0, 0x0020},
    {"DO-X-short",       0x062000, true,  6, 1, 0, 0x0020},
    {"DO-Y-short",       0x062040, true,  6, 0, 1, 0x0020},
    {"DO-X-direct",      0x066000, true,  6, 1, 0, 0x0020},
    {"DO-X-indexed",     0x066800, true,  8, 1, 0, 0x0020},
    {"DO-X-predecrement", 0x067800, true, 8, 1, 0, 0x0021},
    {"DO-Y-direct",      0x066040, true,  6, 0, 1, 0x0020},
}};

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 16> p{};
    std::array<uint32_t, 256> x{};
    std::array<uint32_t, 256> y{};
    uint32_t instruction;
    uint64_t steps = 0;
    int64_t decode_step = -1;
    int64_t writeback_step = -1;
    unsigned x_reads = 0;
    unsigned y_reads = 0;
    unsigned y32_writes = 0;
    unsigned y33_writes = 0;

    explicit Harness(const Case &test) : instruction(test.instruction) {
        p[0] = 0x370300; // MOVE #<$03,R7.
        p[1] = 0x60f400; // MOVE #>R0-value,R0.
        p[2] = test.r0;
        p[3] = 0x000000; // Settle explicit R0 write before pointer use.
        if (test.is_do) {
            p[4] = instruction;
            p[5] = 0x000006; // One-instruction loop ending at P:$6.
            p[6] = kFollower;
            p[7] = kSuccessor;
            p[8] = kJump;
            p[9] = 0x000008;
        } else {
            p[4] = instruction;
            p[5] = kFollower;
            p[6] = kSuccessor;
            p[7] = kJump;
            p[8] = 0x000007;
        }
        x[0x20] = 3;
        y[0x20] = 3;

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
        const bool xr = core.x_rd_en;
        const bool yr = core.y_rd_en;
        const bool yw = core.y_wr_en;
        const uint16_t xa = core.x_addr;
        const uint16_t ya = core.y_addr;
        const uint32_t yd = core.y_wdata;

        if (core.state_out == 2 &&
            (core.p_fetch_data & 0x00ffffff) == instruction &&
            decode_step < 0)
            decode_step = static_cast<int64_t>(steps);
        if (core.state_out == 4 && core.cur_inst_out == instruction &&
            writeback_step < 0)
            writeback_step = static_cast<int64_t>(steps);
        if (xr) {
            ++x_reads;
            core.x_rdata = x[xa & 0xff] & 0x00ffffff;
        }
        if (yr) {
            ++y_reads;
            core.y_rdata = y[ya & 0xff] & 0x00ffffff;
        }

        core.clk = 1;
        core.eval();
        if (yw) {
            y[ya & 0xff] = yd & 0x00ffffff;
            if (ya == 0x0032)
                ++y32_writes;
            if (ya == 0x0033)
                ++y33_writes;
        }
        if (fetch)
            core.p_fetch_data = p[fetch_address & 0x0f] & 0x00ffffff;
        ++steps;
    }
};

bool run_case(const Case &test) {
    Harness h(test);
    h.core.run = 1;
    for (unsigned i = 0; i < 240 && h.y33_writes == 0; ++i)
        h.tick();

    const int64_t clocks = h.decode_step >= 0 && h.writeback_step >= 0 ?
        h.writeback_step - h.decode_step + 1 : -1;
    const bool effects = h.x_reads == test.expected_x_reads &&
                         h.y_reads == test.expected_y_reads &&
                         h.y32_writes == 3 && h.y33_writes == 1;
    const bool pass = clocks == test.expected_clocks && effects;
    std::printf("%s LOOP timing %-20s clocks=%lld/%u XR=%u/%u YR=%u/%u "
                "Y32=%u/3 Y33=%u/1 effects=%u decode=%lld wb=%lld\n",
                pass ? "PASS" : "FAIL", test.name,
                static_cast<long long>(clocks), test.expected_clocks,
                h.x_reads, test.expected_x_reads,
                h.y_reads, test.expected_y_reads,
                h.y32_writes, h.y33_writes, effects,
                static_cast<long long>(h.decode_step),
                static_cast<long long>(h.writeback_step));
    return pass;
}

bool run_enddo_case() {
    constexpr uint32_t kEnddo = 0x00008c;
    const Case test{"ENDDO-active", kEnddo, false, 2, 0, 0, 0x0020};
    Harness h(test);

    // Enter a five-iteration loop, execute one visible body operation, and
    // terminate it explicitly. The following write must execute exactly once;
    // a timing-only no-op or an ENDDO that leaves the loop live cannot pass.
    h.p.fill(0);
    h.p[0] = 0x375500; // MOVE #<$55,R7.
    h.p[1] = 0x060580; // DO #5,$4.
    h.p[2] = 0x000004;
    h.p[3] = kFollower;
    h.p[4] = kEnddo;
    h.p[5] = kSuccessor;
    h.p[6] = kJump;
    h.p[7] = 0x000006;

    h.core.run = 1;
    for (unsigned i = 0; i < 160 && h.y33_writes == 0; ++i)
        h.tick();

    const int64_t clocks = h.decode_step >= 0 && h.writeback_step >= 0 ?
        h.writeback_step - h.decode_step + 1 : -1;
    const bool effects = h.y32_writes == 1 && h.y33_writes == 1 &&
                         (h.core.sr_out & 0x008000) == 0 &&
                         h.x_reads == 0 && h.y_reads == 0;
    const bool pass = clocks == 2 && effects;
    std::printf("%s LOOP timing %-20s clocks=%lld/2 XR=%u/0 YR=%u/0 "
                "Y32=%u/1 Y33=%u/1 LF=%u effects=%u decode=%lld wb=%lld\n",
                pass ? "PASS" : "FAIL", test.name,
                static_cast<long long>(clocks), h.x_reads, h.y_reads,
                h.y32_writes, h.y33_writes,
                (h.core.sr_out & 0x008000) != 0, effects,
                static_cast<long long>(h.decode_step),
                static_cast<long long>(h.writeback_step));
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    unsigned failures = 0;
    for (const auto &test : kCases)
        failures += !run_case(test);
    failures += !run_enddo_case();
    std::printf("LOOP TIMING SUMMARY cases=%zu failures=%u\n",
                kCases.size() + 1, failures);
    return failures ? 1 : 0;
}

