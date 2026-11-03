// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

// Direct production-core DSP56001 STOP processing-state oracle.

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "Vdsp56001_core___024root.h"
#include "verilated.h"

namespace {

constexpr uint32_t kStop = 0x000087;
constexpr uint16_t kVector = 0x0008;
constexpr uint16_t kHandler = 0x0040;
constexpr unsigned kShortRecovery = 17;
constexpr unsigned kLongRecovery = 65545;

class StopHarness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};
    uint64_t cycles = 0;
    uint64_t vector_cycle = 0;
    unsigned vector_fetches = 0;
    unsigned handler_fetches = 0;

    explicit StopHarness(bool short_delay) {
        p[0x00] = kStop;
        p[0x01] = 0x375500;       // MOVE #<$55,R7 after wake/RTI.
        p[0x02] = 0x0af080;       // JMP >$0002.
        p[0x03] = 0x000002;
        p[kVector] = 0x0bf080;    // IRQA long vector: JSR >$0040.
        p[kVector + 1] = kHandler;
        p[kHandler] = 0x366600;   // MOVE #<$66,R6.
        p[kHandler + 1] = 0x000004; // RTI.

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
        core.irq_priority = 0x0003; // Level-mode IRQA at IPL 1.
        core.dbg_reg_wr_en = 0;
        core.dbg_reg_wr_addr = 0;
        core.dbg_reg_wr_data = 0;

        for (unsigned i = 0; i < 4; ++i)
            tick();
        core.rst_n = 1;
        debug_write(0x39, 0x000000); // SR interrupt mask IPL 0.
        debug_write(0x3a, short_delay ? 0x000040 : 0x000000); // OMR SD.
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        if (fetch_en && fetch_addr == kVector) {
            vector_cycle = cycles;
            ++vector_fetches;
            core.irq_a = 0;
        }
        if (fetch_en && fetch_addr == kHandler)
            ++handler_fetches;

        core.clk = 1;
        core.eval();
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 0xff] & 0x00ffffff;
        ++cycles;
    }

    void debug_write(uint8_t addr, uint64_t data) {
        core.dbg_reg_wr_addr = addr;
        core.dbg_reg_wr_data = data;
        core.dbg_reg_wr_en = 1;
        tick();
        core.dbg_reg_wr_en = 0;
    }

    bool enter_stop() {
        core.run = 1;
        for (unsigned i = 0; i < 100; ++i) {
            tick();
            if (core.state_out == 5 && core.stop_reset)
                return true;
        }
        return false;
    }

    uint16_t r(unsigned index) const {
        return core.rootp->dsp56001_core__DOT__r_reg[index & 7];
    }
};

bool short_recovery_case() {
    StopHarness h(true);
    const bool held = h.enter_stop() && h.r(7) == 0 &&
                      h.vector_fetches == 0;
    const uint64_t request_cycle = h.cycles;
    h.core.irq_a = 1;
    for (unsigned i = 0; i < 100 && h.vector_fetches == 0; ++i)
        h.tick();
    const uint64_t delay = h.vector_fetches ?
        h.vector_cycle - request_cycle : 0;
    for (unsigned i = 0; i < 120; ++i)
        h.tick();
    const bool resumed = h.vector_fetches == 1 && h.handler_fetches != 0 &&
                         h.r(6) == 0x0066 && h.r(7) == 0x0055 &&
                         !h.core.stop_reset;
    const bool pass = held && delay >= kShortRecovery && resumed;
    std::printf("%s STOP short recovery held=%u delay=%llu vector=%u "
                "handler=%u R6=%04x R7=%04x stop-reset=%u\n",
                pass ? "PASS" : "FAIL", held,
                static_cast<unsigned long long>(delay), h.vector_fetches,
                h.handler_fetches, h.r(6), h.r(7), h.core.stop_reset);
    return pass;
}

bool long_recovery_case() {
    StopHarness h(false);
    const bool held = h.enter_stop() && h.r(7) == 0 &&
                      h.vector_fetches == 0;
    const uint64_t request_cycle = h.cycles;
    h.core.irq_a = 1;
    for (unsigned i = 0; i < kLongRecovery - 1; ++i)
        h.tick();
    const bool not_early = h.vector_fetches == 0 && h.core.stop_reset;
    for (unsigned i = 0; i < 64 && h.vector_fetches == 0; ++i)
        h.tick();
    const uint64_t delay = h.vector_fetches ?
        h.vector_cycle - request_cycle : 0;
    for (unsigned i = 0; i < 120; ++i)
        h.tick();
    const bool resumed = h.vector_fetches == 1 && h.handler_fetches != 0 &&
                         h.r(6) == 0x0066 && h.r(7) == 0x0055 &&
                         !h.core.stop_reset;
    const bool pass = held && not_early && delay >= kLongRecovery && resumed;
    std::printf("%s STOP long recovery held=%u not-early=%u delay=%llu "
                "vector=%u handler=%u R6=%04x R7=%04x\n",
                pass ? "PASS" : "FAIL", held, not_early,
                static_cast<unsigned long long>(delay), h.vector_fetches,
                h.handler_fetches, h.r(6), h.r(7));
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const bool short_ok = short_recovery_case();
    const bool long_ok = long_recovery_case();
    return short_ok && long_ok ? 0 : 1;
}
