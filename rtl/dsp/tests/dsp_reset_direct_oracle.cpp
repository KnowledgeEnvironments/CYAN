// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

// Direct production-core DSP56001 RESET processing-state oracle.

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "Vdsp56001_core___024root.h"
#include "verilated.h"

namespace {

constexpr uint32_t kReset = 0x000084;
constexpr uint32_t kSuccessor = 0x375500; // MOVE #<$55,R7.

class ResetHarness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 16> p{};
    uint64_t cycles = 0;
    int64_t decode_cycle = -1;
    int64_t pulse_cycle = -1;
    int64_t successor_cycle = -1;
    unsigned reset_pulses = 0;

    ResetHarness() {
        p[0] = 0x366600; // MOVE #<$66,R6: state that RESET must preserve.
        p[1] = kReset;
        p[2] = kSuccessor;
        p[3] = 0x0c0003; // Stable one-word loop.

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
        debug_write(0x0e, 0x0055aa12345678ULL);
        debug_write(0x39, 0x000321);
        debug_write(0x3a, 0x000040);
        core.run = 1;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch = core.p_fetch_en;
        const uint16_t address = core.p_fetch_addr;

        if (core.state_out == 2 &&
            (core.p_fetch_data & 0x00ffffff) == kReset && decode_cycle < 0)
            decode_cycle = cycles;
        if (core.software_reset) {
            if (pulse_cycle < 0)
                pulse_cycle = cycles;
            ++reset_pulses;
        }
        if (core.state_out == 4 && core.cur_inst_out == kSuccessor &&
            successor_cycle < 0)
            successor_cycle = cycles;

        core.clk = 1;
        core.eval();
        if (fetch && core.p_fetch_ready)
            core.p_fetch_data = p[address & 0x0f] & 0x00ffffff;
        ++cycles;
    }

    void debug_write(uint8_t address, uint64_t data) {
        core.dbg_reg_wr_addr = address;
        core.dbg_reg_wr_data = data;
        core.dbg_reg_wr_en = 1;
        tick();
        core.dbg_reg_wr_en = 0;
    }

    uint16_t r(unsigned index) const {
        return core.rootp->dsp56001_core__DOT__r_reg[index & 7];
    }

    uint8_t omr() const {
        return core.rootp->dsp56001_core__DOT__omr;
    }
};

bool zero_wait_case() {
    ResetHarness h;
    for (unsigned i = 0; i < 80 && h.successor_cycle < 0; ++i)
        h.tick();
    const int64_t clocks = h.decode_cycle >= 0 && h.pulse_cycle >= 0 ?
                           h.pulse_cycle - h.decode_cycle + 1 : -1;
    const bool preserved = h.r(6) == 0x0066 &&
                           h.core.acc_a_out == 0x0055aa12345678ULL &&
                           h.core.sr_out == 0x000321 && h.omr() == 0x40;
    const bool pass = clocks == 4 && h.reset_pulses == 1 &&
                      h.successor_cycle == h.pulse_cycle + 2 &&
                      h.r(7) == 0x0055 && preserved;
    std::printf("%s RESET zero-wait clocks=%lld/4 pulses=%u successor=%lld/2 "
                "R6=%04x R7=%04x A=%014llx SR=%06x OMR=%02x\n",
                pass ? "PASS" : "FAIL", static_cast<long long>(clocks),
                h.reset_pulses,
                static_cast<long long>(h.successor_cycle - h.pulse_cycle),
                h.r(6), h.r(7),
                static_cast<unsigned long long>(h.core.acc_a_out),
                h.core.sr_out, h.omr());
    return pass;
}

bool stalled_successor_case() {
    ResetHarness h;
    for (unsigned guard = 0; guard < 80; ++guard) {
        h.core.clk = 0;
        h.core.eval();
        if (h.core.state_out == 4 && h.core.cur_inst_out == kReset) {
            h.core.p_fetch_ready = 0;
            break;
        }
        h.tick();
    }
    const bool reached = h.core.state_out == 4 &&
                         h.core.cur_inst_out == kReset;
    for (unsigned i = 0; i < 5; ++i)
        h.tick();
    const bool held = h.core.state_out == 4 && h.reset_pulses == 0 &&
                      h.r(7) == 0;
    h.core.p_fetch_ready = 1;
    for (unsigned i = 0; i < 40 && h.successor_cycle < 0; ++i)
        h.tick();
    const bool pass = reached && held && h.reset_pulses == 1 &&
                      h.r(6) == 0x0066 && h.r(7) == 0x0055;
    std::printf("%s RESET stalled-successor reached=%u held=%u pulses=%u "
                "R6=%04x R7=%04x\n", pass ? "PASS" : "FAIL", reached,
                held, h.reset_pulses, h.r(6), h.r(7));
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const bool zero_wait_ok = zero_wait_case();
    const bool stalled_ok = stalled_successor_case();
    return zero_wait_ok && stalled_ok ? 0 : 1;
}
