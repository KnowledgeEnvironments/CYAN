// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 production-core DIV opcode/source/destination oracle
//
// Motorola encodes DIV S,D as:
//   0000 0001 1000 0000 01JJ d000
// where JJ is bits 5:4 and d is bit 3.  These cases distinguish X0/A from
// Y1/B through the real decoder, ALU and core writeback path.  Neither form
// is a memory operation, so any X/Y transaction is forbidden.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint32_t div_opcode(unsigned jj, bool dest_b) {
    return 0x018040U | ((jj & 3U) << 4) | (dest_b ? 0x8U : 0U);
}

class DivHarness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 128> p{};
    bool saw_xy_request = false;

    DivHarness(uint32_t opcode, uint64_t initial_a, uint64_t initial_b,
               uint32_t initial_sr, uint32_t x0, uint32_t x1,
               uint32_t y0, uint32_t y1) {
        p[0] = opcode;
        p[1] = 0x000000; // DIV writeback spacing
        p[2] = 0x0af080; // stable terminal loop
        p[3] = 0x000002;

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
        debug_write(0x04, x0);
        debug_write(0x05, x1);
        debug_write(0x06, y0);
        debug_write(0x07, y1);
        debug_write(0x0e, initial_a);
        debug_write(0x0f, initial_b);
        debug_write(0x39, initial_sr);
        core.run = 1;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        saw_xy_request |= core.x_rd_en || core.x_wr_en ||
                          core.y_rd_en || core.y_wr_en;
        core.clk = 1;
        core.eval();
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 0x7f] & 0x00ffffff;
    }

    void debug_write(uint8_t reg, uint64_t value) {
        core.dbg_reg_wr_addr = reg;
        core.dbg_reg_wr_data = value;
        core.dbg_reg_wr_en = 1;
        tick();
        core.dbg_reg_wr_en = 0;
    }

    void run(unsigned count = 160) {
        for (unsigned i = 0; i < count; ++i)
            tick();
    }
};

constexpr uint64_t kMask56 = 0x00ffffffffffffffULL;
constexpr uint64_t kOther = 0x00123456789abcULL;
constexpr uint32_t kInitialSr = 0x00003c;
constexpr std::array<uint32_t, 4> kSources = {
    0x000001, 0x000002, 0x800003, 0xfffffe,
};

struct Expected {
    uint64_t result;
    uint32_t sr;
};

Expected expected_div(uint64_t destination, uint32_t source, uint32_t sr) {
    // Motorola UMA1 A-58--A-61: compare signs, shift D left with old C into
    // bit zero, then add or subtract the sign-extended divisor from D[55:24].
    const bool signs_differ = ((destination >> 55) & 1) !=
                              ((source >> 23) & 1);
    const uint64_t shifted = ((destination << 1) & kMask56) | (sr & 1);
    const uint32_t upper = shifted >> 24;
    const uint32_t source_extended =
        (source & 0x800000) ? (source | 0xff000000) : source;
    const uint32_t upper_result = signs_differ ?
        upper + source_extended : upper - source_extended;
    const uint64_t result = ((uint64_t{upper_result} << 24) |
                             (shifted & 0xffffff)) & kMask56;
    const bool carry = ((result >> 55) & 1) == 0;
    const bool overflow = ((shifted >> 55) & 1) !=
                          ((destination >> 55) & 1);
    const uint32_t result_sr = (sr & ~0x43U) |
                               (overflow ? 0x42U : 0) |
                               (carry ? 0x01U : 0);
    return {result, result_sr};
}

bool run_case(unsigned jj, bool dest_b) {
    const uint64_t initial_destination = 1;
    DivHarness h(div_opcode(jj, dest_b),
                 dest_b ? kOther : initial_destination,
                 dest_b ? initial_destination : kOther, kInitialSr,
                 kSources[0], kSources[2], kSources[1], kSources[3]);
    h.run();
    const Expected expected =
        expected_div(initial_destination, kSources[jj], kInitialSr);
    const uint64_t expected_a = dest_b ? kOther : expected.result;
    const uint64_t expected_b = dest_b ? expected.result : kOther;
    const bool pass = h.core.acc_a_out == expected_a &&
                      h.core.acc_b_out == expected_b &&
                      h.core.sr_out == expected.sr && !h.saw_xy_request;
    if (!pass) {
        static constexpr const char *kNames[] = {"X0", "Y0", "X1", "Y1"};
        std::printf("FAIL DIV %s,%c opcode=%06x A=%014llx/%014llx "
                    "B=%014llx/%014llx SR=%06x/%06x xy=%u\n",
                    kNames[jj], dest_b ? 'B' : 'A',
                    div_opcode(jj, dest_b),
                    static_cast<unsigned long long>(h.core.acc_a_out),
                    static_cast<unsigned long long>(expected_a),
                    static_cast<unsigned long long>(h.core.acc_b_out),
                    static_cast<unsigned long long>(expected_b),
                    h.core.sr_out, expected.sr, h.saw_xy_request);
    }
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    unsigned cases = 0;
    unsigned failures = 0;
    for (unsigned jj = 0; jj < 4; ++jj) {
        for (unsigned dest_b = 0; dest_b < 2; ++dest_b) {
            ++cases;
            failures += !run_case(jj, dest_b);
        }
    }
    std::printf("%s DIV MATRIX forms=%u failures=%u\n",
                failures ? "FAIL" : "PASS", cases, failures);
    return failures ? 1 : 0;
}

