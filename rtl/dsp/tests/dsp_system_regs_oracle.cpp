// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 BCR/IPR architectural register oracle
//
// DSP56001UM09 section 9.2.2 and Figure 9-10 define BCR as four four-bit
// wait-state fields at X:$FFFE.  DSP56001UM08 section 8.2.2.1 and Figure 8-5
// define the 16-bit IPR at X:$FFFF, with bits 9:6 reserved and reading zero.
///////////////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <cstdio>

#include "Vdsp56001_periph.h"
#include "verilated.h"

namespace {

constexpr uint8_t kBcr = 0x3e;
constexpr uint8_t kIpr = 0x3f;
constexpr uint32_t kBcrMask = 0x00ffff;
constexpr uint32_t kIprMask = 0x00fc3f;

class Harness {
public:
    Vdsp56001_periph top;

    Harness() {
        top.clk = 0;
        top.rst_n = 0;
        top.clock_enable = 0;
        top.soft_reset = 0;
        top.stop_reset = 0;
        top.addr = 0;
        top.wdata = 0;
        top.rd_en = 0;
        top.wr_en = 0;
        top.ssi_tx_ready = 0;
        top.ssi_rx_valid = 0;
        top.ssi_rx_data = 0;
        top.sci_tx_ready = 0;
        top.sci_tx_done = 0;
        top.sci_rx_valid = 0;
        top.sci_rx_data = 0;
        top.sci_rx_r8 = 0;
        top.sci_rx_framing_error = 0;
        top.sci_rx_parity_error = 0;
        top.sci_rx_idle = 0;
        top.sci_idle_ack = 0;
        top.sci_timer_ack = 0;
        for (unsigned i = 0; i < 3; ++i)
            tick();
        top.rst_n = 1;
        tick();
    }

    void tick() {
        top.clk = 0;
        top.eval();
        top.clk = 1;
        top.eval();
        top.clk = 0;
        top.eval();
    }

    void write(uint8_t addr, uint32_t data) {
        top.addr = addr;
        top.wdata = data & 0x00ffffff;
        top.wr_en = 1;
        tick();
        top.wr_en = 0;
        top.eval();
    }

    uint32_t peek(uint8_t addr) {
        top.addr = addr;
        top.eval();
        return top.rdata & 0x00ffffff;
    }

    void pulse_soft_reset() {
        top.soft_reset = 1;
        tick();
        top.soft_reset = 0;
        tick();
    }

    void pulse_stop_reset() {
        top.stop_reset = 1;
        tick();
        top.stop_reset = 0;
        tick();
    }
};

bool check(const char *name, uint32_t actual, uint32_t expected) {
    const bool pass = actual == expected;
    std::printf("%s DSP system %s actual=%06x expected=%06x\n",
                pass ? "PASS" : "FAIL", name,
                actual & 0x00ffffff, expected & 0x00ffffff);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    bool pass = true;

    {
        Harness h;
        pass &= check("hard-reset BCR", h.peek(kBcr), kBcrMask);
        pass &= check("hard-reset IPR", h.peek(kIpr), 0);
    }

    {
        Harness h;
        h.write(kBcr, 0x00ffffff);
        h.write(kIpr, 0x00ffffff);
        pass &= check("BCR 16-bit width", h.peek(kBcr), kBcrMask);
        pass &= check("IPR width/reserved bits", h.peek(kIpr), kIprMask);
    }

    {
        Harness h;
        h.write(kBcr, 0x001234);
        h.write(kIpr, 0x00c033);
        h.pulse_stop_reset();
        pass &= check("STOP preserves BCR", h.peek(kBcr), 0x001234);
        pass &= check("STOP preserves IPR", h.peek(kIpr), 0x00c033);
        h.pulse_soft_reset();
        pass &= check("software-reset BCR", h.peek(kBcr), kBcrMask);
        pass &= check("software-reset IPR", h.peek(kIpr), 0);
    }

    if (!pass)
        return 1;
    std::puts("PASS DSP56001 BCR/IPR architectural register boundary");
    return 0;
}

