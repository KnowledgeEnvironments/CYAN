// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

// Production Port-B audio-glue protocol and block-boundary oracle.

#include <cstdint>
#include <cstdio>

#include "Vindigo_hp1_audio_glue.h"
#include "verilated.h"

namespace {

constexpr uint8_t kPbc = 0x20;
constexpr uint8_t kPbddr = 0x22;
constexpr uint8_t kPbd = 0x24;
constexpr uint8_t kRevision = 0x05;
constexpr uint8_t kIrqMask = 0x07;
constexpr uint8_t kIrqStatus = 0x08;
constexpr uint8_t kTxControl = 0x0a;

class Harness {
public:
    Vindigo_hp1_audio_glue dut;

    Harness() {
        dut.clk = 0;
        dut.rst_n = 0;
        dut.dsp_software_reset = 0;
        dut.xperiph_addr = 0;
        dut.xperiph_wdata = 0;
        dut.xperiph_wr_en = 0;
        dut.tx_subframe_accepted = 0;
        dut.rx_subframe_received = 0;
        for (unsigned i = 0; i < 3; ++i)
            tick();
        dut.rst_n = 1;
        tick();
    }

    void tick() {
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        dut.clk = 0;
        dut.eval();
    }

    void port_write(uint8_t address, uint32_t data) {
        dut.xperiph_addr = address;
        dut.xperiph_wdata = data & 0x00ffffff;
        dut.xperiph_wr_en = 1;
        tick();
        dut.xperiph_wr_en = 0;
        dut.eval();
    }

    void configure(uint16_t directions = 0x7fff) {
        port_write(kPbc, 0);
        port_write(kPbddr, directions);
    }

    void glue_write(uint8_t reg, uint8_t data) {
        const uint16_t base = (uint16_t(reg & 0x1f) << 8) | data;
        port_write(kPbd, base | (1u << 14));
        port_write(kPbd, base);
    }

    uint8_t glue_read(uint8_t reg) {
        // PB14:8 are outputs; PB7:0 remain inputs driven by the glue.
        configure(0x7f00);
        const uint16_t base = uint16_t(reg & 0x1f) << 8;
        port_write(kPbd, base | (1u << 13));
        port_write(kPbd, base);
        dut.eval();
        const uint8_t value = dut.pbd_rdata & 0xff;
        port_write(kPbd, base | (1u << 13));
        return value;
    }

    void pulse_software_reset() {
        dut.dsp_software_reset = 1;
        tick();
        dut.dsp_software_reset = 0;
        tick();
    }

    void accept_subframes(unsigned count) {
        for (unsigned i = 0; i < count; ++i) {
            dut.tx_subframe_accepted = 1;
            tick();
            dut.tx_subframe_accepted = 0;
            tick();
        }
    }
};

bool protocol_case() {
    Harness h;
    const uint8_t revision = h.glue_read(kRevision);

    h.configure();
    h.glue_write(kTxControl, 0x2b); // GO=1, rate selector=0x15.
    const bool tx_programmed = h.dut.tx_clock_active &&
                               h.dut.tx_rate_select == 0x15;

    h.glue_write(kIrqMask, 0x0f);   // Enable only soft source 4.
    h.glue_write(kIrqStatus, 0x00); // All-zero write raises source 4.
    const bool irq_raised = h.dut.irq_b;
    const uint8_t pending = h.glue_read(kIrqStatus);

    h.pulse_software_reset();
    const bool reset_preserved = h.dut.tx_clock_active &&
                                 h.dut.tx_rate_select == 0x15 && h.dut.irq_b;

    h.configure();
    h.glue_write(kIrqStatus, 0x10); // W1C source 4.
    const bool irq_cleared = !h.dut.irq_b;
    const bool pass = revision == 0x02 && tx_programmed && irq_raised &&
                      (pending & 0x10) == 0 && reset_preserved && irq_cleared;
    std::printf("%s AUDIO glue protocol revision=%02x TX=%u rate=%02x "
                "IRQ=%u status=%02x reset-preserved=%u cleared=%u\n",
                pass ? "PASS" : "FAIL", revision, h.dut.tx_clock_active,
                h.dut.tx_rate_select, irq_raised, pending, reset_preserved,
                irq_cleared);
    return pass;
}

bool block_boundary_case() {
    Harness h;
    h.configure();
    h.glue_write(kIrqMask, 0x1b);   // Enable only TX block source 2.
    h.glue_write(kTxControl, 0x01); // GO at base rate.
    h.accept_subframes(383);
    const bool quiet_before = !h.dut.irq_b;
    h.accept_subframes(1);
    const bool exact_boundary = h.dut.irq_b;
    const uint8_t pending = h.glue_read(kIrqStatus);
    const bool pass = quiet_before && exact_boundary && (pending & 0x04) == 0;
    std::printf("%s AUDIO TX block boundary quiet383=%u irq384=%u status=%02x\n",
                pass ? "PASS" : "FAIL", quiet_before, exact_boundary,
                pending);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const bool protocol_ok = protocol_case();
    const bool block_ok = block_boundary_case();
    return protocol_ok && block_ok ? 0 : 1;
}
