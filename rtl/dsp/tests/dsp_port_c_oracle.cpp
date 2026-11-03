// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 Port C production-peripheral register oracle
//
// DSP56001UM11 section 11.1 and Figure 11-3 define three nine-bit registers:
// PCC at X:$FFE1, PCDDR at X:$FFE3, and PCD at X:$FFE5.  Reset clears all
// three.  Reading an output-configured PCD bit returns its output-data latch.
// The CYAN board boundary does not expose Port C GPIO input pins, so this
// oracle deliberately grades only output-latch readback and requires unwired
// input-configured bits to read low.
///////////////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <cstdio>

#include "Vdsp56001_periph.h"
#include "verilated.h"

namespace {

constexpr uint8_t kPcc = 0x21;
constexpr uint8_t kPcddr = 0x23;
constexpr uint8_t kPcd = 0x25;
constexpr uint32_t kPortMask = 0x0001ff;

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
    std::printf("%s Port C %s actual=%06x expected=%06x\n",
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
        pass &= check("hard-reset PCC", h.peek(kPcc), 0);
        pass &= check("hard-reset PCDDR", h.peek(kPcddr), 0);
        pass &= check("hard-reset PCD", h.peek(kPcd), 0);
    }

    // Upper X-data-bus bits do not exist in any Port C register.
    {
        Harness h;
        h.write(kPcc, 0x00ffffff);
        h.write(kPcddr, 0x00ffffff);
        h.write(kPcd, 0x00ffffff);
        pass &= check("PCC nine-bit width", h.peek(kPcc), kPortMask);
        pass &= check("PCDDR nine-bit width", h.peek(kPcddr), kPortMask);
        pass &= check("PCD nine-bit output latch", h.peek(kPcd), kPortMask);
    }

    // With GPIO input pins unwired at the CYAN boundary, input-configured
    // bits read low while output-configured bits return their data latches.
    {
        Harness h;
        h.write(kPcddr, 0x000155);
        h.write(kPcd, 0x0001e3);
        pass &= check("mixed direction readback", h.peek(kPcd),
                      0x0001e3 & 0x000155);
    }

    // The RESET instruction clears all three Port C registers.
    {
        Harness h;
        h.write(kPcc, 0x0001ff);
        h.write(kPcddr, 0x0001ff);
        h.write(kPcd, 0x0001ff);
        h.pulse_soft_reset();
        pass &= check("software-reset PCC", h.peek(kPcc), 0);
        pass &= check("software-reset PCDDR", h.peek(kPcddr), 0);
        pass &= check("software-reset PCD", h.peek(kPcd), 0);
    }

    // STOP places the serial interfaces in individual reset; it does not
    // reset the Port C GPIO configuration or output latch.
    {
        Harness h;
        h.write(kPcc, 0x00012a);
        h.write(kPcddr, 0x000155);
        h.write(kPcd, 0x0001e3);
        h.pulse_stop_reset();
        pass &= check("STOP preserves PCC", h.peek(kPcc), 0x00012a);
        pass &= check("STOP preserves PCDDR", h.peek(kPcddr), 0x000155);
        pass &= check("STOP preserves PCD latch", h.peek(kPcd),
                      0x0001e3 & 0x000155);
    }

    if (!pass)
        return 1;
    std::puts("PASS DSP56001 Port C nine-bit register boundary");
    return 0;
}
