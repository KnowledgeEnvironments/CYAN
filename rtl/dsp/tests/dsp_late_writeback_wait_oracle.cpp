// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 later-writeback X/Y wait-state oracle
//
// Modifying memory bit operations read in their issue stage and write the
// result in S_WB_FETCH while the next internal-P instruction is fetched.  A
// denied external X/Y write must retain the instruction, address, and payload
// until that data transfer is accepted; an independently ready P fetch must
// not allow retirement to discard the write.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint16_t kTarget = 0xb000;
constexpr uint32_t kXInstruction = 0x0a7020; // BSET #0,X:>$B000.
constexpr uint32_t kYInstruction = 0x0a7060; // BSET #0,Y:>$B000.
constexpr uint32_t kSuccessor = 0x375500;    // MOVE #<$55,R7.
constexpr unsigned kDeniedClocks = 3;

struct Result {
    bool completed = false;
    bool held = true;
    bool successor_early = false;
    unsigned denied = 0;
    unsigned accepted = 0;
    uint16_t address = 0;
    uint32_t data = 0;

    bool pass() const {
        return completed && held && !successor_early &&
               denied == kDeniedClocks && accepted == 1 &&
               address == kTarget && data == 1;
    }
};

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};

    explicit Harness(bool y_space) {
        p[0] = y_space ? kYInstruction : kXInstruction;
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
        core.x_rdata = 0;
        core.y_rdata = 0;
        core.x_ready = 1;
        core.y_ready = 1;
        core.p_data_rdata = 0;
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
    }

    void plain_tick() {
        core.clk = 0;
        core.eval();
        const bool fetch = core.p_fetch_en;
        const uint16_t fetch_address = core.p_fetch_addr;
        core.clk = 1;
        core.eval();
        if (fetch)
            core.p_fetch_data = p[fetch_address & 0xff] & 0x00ffffff;
    }

    void run(bool y_space, Result &result) {
        core.run = 1;
        bool saw_write = false;
        uint8_t held_state = 0;
        uint16_t held_pc = 0;

        for (unsigned cycle = 0; cycle < 200; ++cycle) {
            core.clk = 0;
            core.p_fetch_ready = 1;
            core.x_ready = 1;
            core.y_ready = 1;
            core.eval();

            const bool fetch = core.p_fetch_en;
            const uint16_t fetch_address = core.p_fetch_addr;
            const bool write = y_space ? core.y_wr_en : core.x_wr_en;
            const uint16_t address = y_space ? core.y_addr : core.x_addr;
            const uint32_t data =
                (y_space ? core.y_wdata : core.x_wdata) & 0x00ffffff;

            if (write) {
                if (!saw_write) {
                    saw_write = true;
                    held_state = core.state_out;
                    held_pc = core.pc_out;
                    result.address = address;
                    result.data = data;
                } else if (core.state_out != held_state ||
                           core.pc_out != held_pc ||
                           address != result.address || data != result.data) {
                    result.held = false;
                }

                if (result.denied < kDeniedClocks) {
                    if (y_space)
                        core.y_ready = 0;
                    else
                        core.x_ready = 0;
                    ++result.denied;
                } else {
                    ++result.accepted;
                }
                core.eval();
            } else if (saw_write && result.accepted == 0) {
                result.held = false;
            }

            if (core.state_out == 4 && core.cur_inst_out == kSuccessor &&
                result.accepted == 0)
                result.successor_early = true;

            core.clk = 1;
            core.eval();
            if (fetch && core.p_fetch_ready)
                core.p_fetch_data = p[fetch_address & 0xff] & 0x00ffffff;

            if (core.state_out == 4 && core.cur_inst_out == kSuccessor &&
                result.accepted != 0) {
                result.completed = true;
                break;
            }
        }
    }
};

bool run_case(bool y_space) {
    Result result;
    Harness harness(y_space);
    harness.run(y_space, result);
    const bool pass = result.pass();
    std::printf("%s late %c-writeback wait completed=%u held=%u early=%u "
                "denied=%u/%u accepted=%u addr=%04x data=%06x\n",
                pass ? "PASS" : "FAIL", y_space ? 'Y' : 'X',
                result.completed, result.held, result.successor_early,
                result.denied, kDeniedClocks, result.accepted,
                result.address, result.data);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const bool x_ok = run_case(false);
    const bool y_ok = run_case(true);
    return x_ok && y_ok ? 0 : 1;
}
