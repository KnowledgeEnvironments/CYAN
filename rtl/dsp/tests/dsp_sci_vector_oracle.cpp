// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 SCI interrupt-vector and fixed-priority oracle
//
// DSP56001UM interrupt priority order within one level is SCI receive
// exception, receive data, transmit data, idle, then timer.  The production
// core currently exposes only receive, transmit and timer requests.  This
// oracle deliberately names the missing exception and idle inputs so the
// test-first revision fails to build until product RTL exposes and routes the
// complete five-source SCI boundary.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint8_t kStateDecode = 2;
constexpr uint16_t kIrqaVector = 0x0008;
constexpr uint16_t kIrqbVector = 0x000a;
constexpr uint16_t kSsiRxExceptionVector = 0x000e;
constexpr uint16_t kSsiTxExceptionVector = 0x0012;
constexpr uint16_t kSciRxVector = 0x0014;
constexpr uint16_t kSciRxExceptionVector = 0x0016;
constexpr uint16_t kSciTxVector = 0x0018;
constexpr uint16_t kSciIdleVector = 0x001a;
constexpr uint16_t kSciTimerVector = 0x001c;

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};
    std::array<uint16_t, 16> order{};
    unsigned order_n = 0;

    Harness() {
        p[0x00] = 0x000000; // NOP
        p[0x01] = 0x0af080; // JMP >$0000
        p[0x02] = 0x000000;

        unsigned handler = 0x0040;
        for (uint16_t vec = kIrqaVector; vec <= kSciTimerVector; vec += 2) {
            p[vec] = 0x0bf080; // JSR >handler
            p[vec + 1] = handler;
            p[handler] = 0x000000;     // NOP
            p[handler + 1] = 0x000004; // RTI
            handler += 2;
        }

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
        core.dbg_reg_wr_en = 1;
        core.dbg_reg_wr_addr = 0x39;
        core.dbg_reg_wr_data = 0; // SR interrupt mask 0
        tick();
        core.dbg_reg_wr_en = 0;
        core.run = 1;
        for (unsigned i = 0; i < 100 && core.state_out != kStateDecode; ++i)
            tick();
    }

    static bool is_vector(uint16_t addr) {
        switch (addr) {
        case kIrqaVector:
        case kIrqbVector:
        case kSsiRxExceptionVector:
        case kSsiTxExceptionVector:
        case kSciRxVector:
        case kSciRxExceptionVector:
        case kSciTxVector:
        case kSciIdleVector:
        case kSciTimerVector:
            return true;
        default:
            return false;
        }
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        const bool accepted = fetch_en && is_vector(fetch_addr);
        if (accepted && order_n < order.size())
            order[order_n++] = fetch_addr;

        core.clk = 1;
        core.eval();

        if (accepted) {
            switch (fetch_addr) {
            case kSsiRxExceptionVector:
                core.irq_ssi_rx_exception = 0;
                core.irq_ssi_rx = 0;
                break;
            case kSsiTxExceptionVector:
                core.irq_ssi_tx_exception = 0;
                core.irq_ssi_tx = 0;
                break;
            case kSciRxExceptionVector:
                core.irq_sci_rx_exception = 0;
                break;
            case kSciRxVector: core.irq_sci_rx = 0; break;
            case kSciTxVector: core.irq_sci_tx = 0; break;
            case kSciIdleVector: core.irq_sci_idle = 0; break;
            case kSciTimerVector: core.irq_sci_timer = 0; break;
            default: break;
            }
        }
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 0xff] & 0x00ffffff;
    }

    void run(unsigned count) {
        for (unsigned i = 0; i < count; ++i)
            tick();
    }

    void set_levels(unsigned external, unsigned ssi, unsigned sci) {
        // IRQA/IRQB use edge mode.  SSI and SCI use their documented IPR
        // fields.  SR is unmasked by the constructor.
        core.irq_priority = static_cast<uint16_t>(
            (external & 3) | 0x4 |
            ((external & 3) << 3) | 0x20 |
            ((ssi & 3) << 12) | ((sci & 3) << 14));
    }

    void assert_all() {
        core.irq_a = 1;
        core.irq_b = 1;
        core.irq_ssi_rx = 1;
        core.irq_ssi_rx_exception = 1;
        core.irq_ssi_tx = 1;
        core.irq_ssi_tx_exception = 1;
        core.irq_sci_rx_exception = 1;
        core.irq_sci_rx = 1;
        core.irq_sci_tx = 1;
        core.irq_sci_idle = 1;
        core.irq_sci_timer = 1;
        tick();
        core.irq_a = 0;
        core.irq_b = 0;
    }
};

template <std::size_t N>
bool run_case(const char *name, unsigned external, unsigned ssi, unsigned sci,
              const std::array<uint16_t, N> &expected) {
    Harness h;
    h.set_levels(external, ssi, sci);
    h.assert_all();
    h.run(700);

    bool ok = h.order_n == expected.size();
    if (ok) {
        for (unsigned i = 0; i < expected.size(); ++i) {
            if (h.order[i] != expected[i]) {
                ok = false;
                break;
            }
        }
    }

    std::printf("%s SCI vector order %-17s got=", ok ? "PASS" : "FAIL", name);
    for (unsigned i = 0; i < h.order_n; ++i)
        std::printf("%s%04x", i ? "," : "", h.order[i]);
    std::printf(" expected=");
    for (unsigned i = 0; i < expected.size(); ++i)
        std::printf("%s%04x", i ? "," : "", expected[i]);
    std::printf("\n");
    return ok;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);

    // All sources at one level: complete Motorola fixed order.
    const bool equal = run_case(
        "equal-level", 2, 2, 2,
        std::array<uint16_t, 9>{
            kIrqaVector, kIrqbVector,
            kSsiRxExceptionVector, kSsiTxExceptionVector,
            kSciRxExceptionVector, kSciRxVector, kSciTxVector,
            kSciIdleVector, kSciTimerVector});

    // Higher SCI must outrank lower external/SSI sources as a class.
    const bool sci_high = run_case(
        "SCI-high", 1, 2, 3,
        std::array<uint16_t, 9>{
            kSciRxExceptionVector, kSciRxVector, kSciTxVector,
            kSciIdleVector, kSciTimerVector,
            kSsiRxExceptionVector, kSsiTxExceptionVector,
            kIrqaVector, kIrqbVector});

    // Higher external/SSI sources must still precede SCI.
    const bool sci_low = run_case(
        "SCI-low", 3, 2, 1,
        std::array<uint16_t, 9>{
            kIrqaVector, kIrqbVector,
            kSsiRxExceptionVector, kSsiTxExceptionVector,
            kSciRxExceptionVector, kSciRxVector, kSciTxVector,
            kSciIdleVector, kSciTimerVector});

    return equal && sci_high && sci_low ? 0 : 1;
}

