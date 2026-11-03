// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 production-core NORM opcode and register-update oracle
//
// Motorola encodes NORM Rn,D as:
//   0000 0001 1101 1RRR 0001 d101
// where RRR is bits 10:8 and d is bit 3.  These cases execute both
// accumulator destinations and separated R-register numbers through the real
// decoder, ALU and core writeback path.  A following ordinary MOVE exposes
// the updated Rn in the otherwise-unused accumulator.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint32_t norm_opcode(unsigned r, bool dest_b) {
    return 0x01d815U | ((r & 7U) << 8) | (dest_b ? 0x8U : 0U);
}

constexpr uint32_t move_immediate_r(unsigned r) {
    return ((0x60U + (r & 7U)) << 16) | 0x00f400U;
}

constexpr uint32_t move_r_to_acc(unsigned r, bool dest_b) {
    const unsigned src_code = 0x10U + (r & 7U);
    const unsigned dst_code = dest_b ? 0x0fU : 0x0eU;
    return 0x200000U | (src_code << 13) | (dst_code << 8);
}

class NormHarness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 128> p{};
    bool saw_xy_request = false;

    NormHarness(unsigned r, bool dest_b, uint16_t initial_r,
                uint64_t initial_a, uint64_t initial_b, uint32_t initial_sr) {
        p[0] = move_immediate_r(r);
        p[1] = initial_r;
        p[2] = 0x000000; // address-register pipeline spacing
        p[3] = norm_opcode(r, dest_b);
        p[4] = 0x000000; // NORM writeback spacing
        p[5] = move_r_to_acc(r, !dest_b); // expose Rn, preserve NORM D
        p[6] = 0x000000; // accumulator writeback spacing
        p[7] = 0x0af080; // stable terminal loop
        p[8] = 0x000007;

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

    void run(unsigned count = 200) {
        for (unsigned i = 0; i < count; ++i)
            tick();
    }
};

enum class Action { Asl, Asr, None };

bool run_case(unsigned r, bool dest_b, Action action) {
    const uint16_t initial_r = 0x0100 + r;
    const uint64_t initial_destination =
        action == Action::Asl ? 0x00000000000002ULL :
        action == Action::Asr ? 0x01000000000000ULL :
                                0x00400000000000ULL;
    const uint32_t initial_sr =
        action == Action::Asl ? 0x000011 :
        action == Action::Asr ? 0x000021 : 0x000001;
    const uint16_t expected_r =
        action == Action::Asl ? initial_r - 1 :
        action == Action::Asr ? initial_r + 1 : initial_r;
    const uint64_t expected_destination =
        action == Action::Asl ? initial_destination << 1 :
        action == Action::Asr ? initial_destination >> 1 : initial_destination;
    const uint64_t expected_exposed = uint64_t{expected_r} << 24;

    NormHarness h(r, dest_b, initial_r,
                  dest_b ? 0 : initial_destination,
                  dest_b ? initial_destination : 0, initial_sr);
    h.run();
    const uint64_t expected_a = dest_b ? expected_exposed : expected_destination;
    const uint64_t expected_b = dest_b ? expected_destination : expected_exposed;
    const bool pass = h.core.pc_out == 0x0009 &&
                      h.core.acc_a_out == expected_a &&
                      h.core.acc_b_out == expected_b &&
                      h.core.sr_out == initial_sr && !h.saw_xy_request;
    if (!pass) {
        const char *name = action == Action::Asl ? "ASL" :
                           action == Action::Asr ? "ASR" : "NOP";
        std::printf("FAIL NORM R%u,%c action=%s opcode=%06x "
                    "A=%014llx/%014llx B=%014llx/%014llx "
                    "SR=%06x/%06x PC=%04x xy=%u\n",
                    r, dest_b ? 'B' : 'A', name,
                    norm_opcode(r, dest_b),
                    static_cast<unsigned long long>(h.core.acc_a_out),
                    static_cast<unsigned long long>(expected_a),
                    static_cast<unsigned long long>(h.core.acc_b_out),
                    static_cast<unsigned long long>(expected_b),
                    h.core.sr_out, initial_sr, h.core.pc_out,
                    h.saw_xy_request);
    }
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    unsigned cases = 0;
    unsigned failures = 0;
    for (unsigned r = 0; r < 8; ++r) {
        for (unsigned dest_b = 0; dest_b < 2; ++dest_b) {
            for (Action action : {Action::Asl, Action::Asr, Action::None}) {
                ++cases;
                failures += !run_case(r, dest_b, action);
            }
        }
    }
    std::printf("%s NORM MATRIX forms=%u failures=%u\n",
                failures ? "FAIL" : "PASS", cases, failures);
    return failures ? 1 : 0;
}

