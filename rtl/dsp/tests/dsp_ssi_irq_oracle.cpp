// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 internal SSI interrupt-controller boundary oracle
//
// The synthesizable core consumes level requests from the synthesizable SSI
// peripheral.  This test proves architectural vectors, IPR[13:12] masking,
// exception-vector selection, fixed same-level priority, highest-IPL choice,
// and WAIT wake-up without modeling interrupt behavior in the harness.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint16_t kIrqaVector = 0x0008;
constexpr uint16_t kRxVector = 0x000c;
constexpr uint16_t kRxExceptionVector = 0x000e;
constexpr uint16_t kTxVector = 0x0010;
constexpr uint16_t kTxExceptionVector = 0x0012;
constexpr uint16_t kHandlerBase = 0x0040;

class IrqHarness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};
    std::array<uint16_t, 16> vector_order{};
    unsigned vector_count = 0;
    unsigned handler_fetches = 0;

    explicit IrqHarness(bool wait_program = false) {
        p[0x00] = wait_program ? 0x000086 : 0x000000; // WAIT or NOP
        p[0x01] = 0x0af080; // JMP >$0000
        p[0x02] = 0x000000;

        install_vector(kIrqaVector, 0);
        install_vector(kRxVector, 1);
        install_vector(kRxExceptionVector, 2);
        install_vector(kTxVector, 3);
        install_vector(kTxExceptionVector, 4);

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
        core.dbg_reg_wr_data = 0;    // unmask all programmable IPLs
        tick();
        core.dbg_reg_wr_en = 0;
        core.run = 1;
    }

    void install_vector(uint16_t vector, unsigned handler_index) {
        const uint16_t handler = kHandlerBase + handler_index * 2;
        p[vector] = 0x0bf080; // JSR >handler
        p[vector + 1] = handler;
        p[handler] = 0x000004; // RTI
    }

    bool is_vector(uint16_t addr) const {
        return addr == kIrqaVector || addr == kRxVector ||
               addr == kRxExceptionVector || addr == kTxVector ||
               addr == kTxExceptionVector;
    }

    void clear_requests() {
        core.irq_a = 0;
        core.irq_b = 0;
        core.irq_ssi_tx = 0;
        core.irq_ssi_rx = 0;
        core.irq_ssi_tx_exception = 0;
        core.irq_ssi_rx_exception = 0;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        bool clear_after_edge = false;
        if (fetch_en && is_vector(fetch_addr)) {
            if (vector_count < vector_order.size())
                vector_order[vector_count] = fetch_addr;
            ++vector_count;
            clear_after_edge = true;
        }
        if (fetch_en && fetch_addr >= kHandlerBase && fetch_addr < 0x0050)
            ++handler_fetches;

        core.clk = 1;
        core.eval();
        if (clear_after_edge)
            clear_requests();
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 0xff] & 0x00ffffff;
    }

    void run(unsigned cycles) {
        for (unsigned i = 0; i < cycles; ++i)
            tick();
    }
};

bool expect_vector(const char *name, bool rx, bool exception,
                   uint16_t expected) {
    IrqHarness h;
    h.core.irq_priority = 0x3000; // SSI IPL2
    h.run(20);
    h.core.irq_ssi_rx = rx;
    h.core.irq_ssi_tx = !rx;
    h.core.irq_ssi_rx_exception = rx && exception;
    h.core.irq_ssi_tx_exception = !rx && exception;
    h.run(100);
    const bool pass = h.vector_count == 1 && h.vector_order[0] == expected &&
                      h.handler_fetches == 1;
    std::printf("%s SSI IRQ %s vector=%04x count=%u handler=%u\n",
                pass ? "PASS" : "FAIL", name,
                h.vector_count ? h.vector_order[0] : 0,
                h.vector_count, h.handler_fetches);
    return pass;
}

bool disabled_is_masked() {
    IrqHarness h;
    h.core.irq_priority = 0x0000;
    h.run(20);
    h.core.irq_ssi_tx = 1;
    h.run(60);
    const bool pass = h.vector_count == 0;
    std::printf("%s SSI disabled IPR request vector_count=%u\n",
                pass ? "PASS" : "FAIL", h.vector_count);
    return pass;
}

bool fixed_priority_same_level() {
    IrqHarness h;
    h.core.irq_priority = 0x3003; // IRQA and SSI both IPL2
    h.run(20);
    h.core.irq_a = 1;
    h.core.irq_ssi_rx = 1;
    h.run(100);
    const bool pass = h.vector_count == 1 &&
                      h.vector_order[0] == kIrqaVector;
    std::printf("%s SSI same-level priority first=%04x count=%u\n",
                pass ? "PASS" : "FAIL",
                h.vector_count ? h.vector_order[0] : 0, h.vector_count);
    return pass;
}

bool higher_ssi_ipl_wins() {
    IrqHarness h;
    h.core.irq_priority = 0x3001; // SSI IPL2, IRQA IPL0
    h.run(20);
    h.core.irq_a = 1;
    h.core.irq_ssi_rx = 1;
    h.run(100);
    const bool pass = h.vector_count == 1 &&
                      h.vector_order[0] == kRxVector;
    std::printf("%s SSI higher-IPL priority first=%04x count=%u\n",
                pass ? "PASS" : "FAIL",
                h.vector_count ? h.vector_order[0] : 0, h.vector_count);
    return pass;
}

bool ssi_wakes_wait() {
    IrqHarness h(true);
    h.core.irq_priority = 0x3000;
    h.run(50);
    const bool halted = h.core.state_out == 5 && h.vector_count == 0;
    h.core.irq_ssi_tx = 1;
    h.run(100);
    const bool pass = halted && h.vector_count == 1 &&
                      h.vector_order[0] == kTxVector &&
                      h.handler_fetches == 1;
    std::printf("%s SSI WAIT wake halted=%u vector=%04x handler=%u\n",
                pass ? "PASS" : "FAIL", halted,
                h.vector_count ? h.vector_order[0] : 0,
                h.handler_fetches);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    bool pass = true;
    pass &= expect_vector("RX normal", true, false, kRxVector);
    pass &= expect_vector("RX exception", true, true, kRxExceptionVector);
    pass &= expect_vector("TX normal", false, false, kTxVector);
    pass &= expect_vector("TX exception", false, true, kTxExceptionVector);
    pass &= disabled_is_masked();
    pass &= fixed_priority_same_level();
    pass &= higher_ssi_ipl_wins();
    pass &= ssi_wakes_wait();
    std::printf("%s DSP56001 SSI interrupt-controller boundary\n",
                pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

