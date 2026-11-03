// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 external P-data wait-state oracle
//
// MOVEM uses the same physical P-space port as instruction fetch.  A denied
// external P-data request must retain its issuing state, address, direction,
// and write payload until the memory router accepts it.  Read data may become
// architectural only after that acceptance.  Exercise both directions with
// the production core and the absolute external-P MOVEM encoding.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint16_t kTarget = 0xb000;
constexpr uint32_t kReadInstruction = 0x07f08c;  // MOVEM P:>$B000,A1.
constexpr uint32_t kWriteInstruction = 0x07708c; // MOVEM A1,P:>$B000.
constexpr uint32_t kSuccessor = 0x375500;        // MOVE #<$55,R7.
constexpr uint32_t kReadData = 0x654321;
constexpr uint32_t kWriteData = 0x123456;
constexpr uint32_t kStaleData = 0xdead00;
constexpr unsigned kDeniedCycles = 3;

struct Result {
    bool completed = false;
    bool held = true;
    unsigned denied = 0;
    unsigned accepted = 0;
    uint16_t address = 0;
    uint32_t data = 0;
    uint32_t a1 = 0;

    bool pass(bool read) const {
        const bool payload = read ? (a1 == kReadData) : (data == kWriteData);
        return completed && held && denied == kDeniedCycles && accepted == 1 &&
               address == kTarget && payload;
    }
};

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 65536> p{};
    bool response_live = false;

    explicit Harness(uint32_t instruction) {
        p[0] = instruction;
        p[1] = kTarget;
        p[2] = kSuccessor;
        p[3] = 0x0c0003; // Stable terminal JMP <$03.

        core.clk = 0;
        core.rst_n = 0;
        core.clock_enable = 1;
        core.halt = 0;
        core.run = 0;
        core.p_fetch_data = 0;
        core.p_fetch_ready = 1;
        core.x_rdata = core.y_rdata = 0;
        core.x_ready = core.y_ready = 1;
        core.p_data_rdata = kStaleData;
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
            plain_tick();
        core.rst_n = 1;

        // A1 is the dddddd=$0c source/destination selected by MOVEM.
        core.dbg_reg_wr_en = 1;
        core.dbg_reg_wr_addr = 0x0e;
        core.dbg_reg_wr_data = 0x00123456000000ULL;
        plain_tick();
        core.dbg_reg_wr_en = 0;
    }

    void plain_tick() {
        core.clk = 0;
        core.eval();
        const bool fetch = core.p_fetch_en;
        const uint16_t fetch_address = core.p_fetch_addr;
        core.clk = 1;
        core.eval();
        if (fetch)
            core.p_fetch_data = p[fetch_address] & 0x00ffffff;
    }

    void run(bool read, Result &result) {
        core.run = 1;
        bool saw_request = false;
        uint8_t held_state = 0;
        uint16_t held_pc = 0;

        for (unsigned cycle = 0; cycle < 200; ++cycle) {
            core.clk = 0;
            core.p_fetch_ready = 1;
            core.p_data_rdata = response_live ? kReadData : kStaleData;
            core.eval();

            const bool fetch = core.p_fetch_en;
            const uint16_t fetch_address = core.p_fetch_addr;
            const bool request = read ? core.p_data_rd_en : core.p_data_wr_en;
            const uint16_t request_address = core.p_data_addr;
            const uint32_t request_data = core.p_data_wdata & 0x00ffffff;

            bool accept = false;
            if (request) {
                if (!saw_request) {
                    saw_request = true;
                    held_state = core.state_out;
                    held_pc = core.pc_out;
                    result.address = request_address;
                } else if (core.state_out != held_state ||
                           core.pc_out != held_pc ||
                           request_address != result.address) {
                    result.held = false;
                }

                if (result.denied < kDeniedCycles) {
                    core.p_fetch_ready = 0;
                    ++result.denied;
                } else {
                    core.p_fetch_ready = 1;
                    accept = true;
                    ++result.accepted;
                    if (read) {
                        core.p_data_rdata = kReadData;
                        response_live = true;
                    } else {
                        result.data = request_data;
                    }
                }
                // Ready participates combinationally in the core's hold
                // decision, so settle again before the accepting edge.
                core.eval();
            } else if (saw_request && result.accepted == 0) {
                // A denied request disappeared before any accepting edge.
                result.held = false;
            }

            core.clk = 1;
            core.eval();
            if (fetch && core.p_fetch_ready)
                core.p_fetch_data = p[fetch_address] & 0x00ffffff;

            if (core.state_out == 4 && core.cur_inst_out == kSuccessor) {
                core.clk = 0;
                core.eval();
                core.clk = 1;
                core.eval();
                result.completed = true;
                break;
            }

            // A correct request is accepted once.  If it remains asserted on
            // a later edge, that is a duplicate external transaction.
            if (accept && result.accepted > 1)
                result.held = false;
        }
        result.a1 = static_cast<uint32_t>((core.acc_a_out >> 24) & 0x00ffffff);
    }
};

bool run_case(bool read) {
    Result result;
    Harness harness(read ? kReadInstruction : kWriteInstruction);
    harness.run(read, result);
    const bool pass = result.pass(read);
    std::printf("%s external P-data wait %s completed=%u held=%u "
                "denied=%u/%u accepted=%u addr=%04x data=%06x A1=%06x\n",
                pass ? "PASS" : "FAIL", read ? "read" : "write",
                result.completed, result.held, result.denied, kDeniedCycles,
                result.accepted, result.address, result.data, result.a1);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const bool read_ok = run_case(true);
    const bool write_ok = run_case(false);
    return read_ok && write_ok ? 0 : 1;
}
