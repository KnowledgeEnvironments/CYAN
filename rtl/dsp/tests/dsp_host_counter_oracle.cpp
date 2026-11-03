// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// HPC1 free-running GIO counter oracle
//
// The product host block runs at the 50 MHz machine clock while its 24-bit
// COUNTER register advances at the 2:3 GIO/SysAD ratio.  Prove the exact
// fractional cadence, identical CPU/DSP visibility, read-only behavior,
// independence from DMA progress, reset, and 24-bit wrap.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_host.h"
#include "Vdsp56001_host___024root.h"
#include "verilated.h"

namespace {

constexpr uint8_t kCount = 0x05;

class Harness {
public:
    Vdsp56001_host dut;

    Harness() {
        dut.clk = 0;
        dut.rst_n = 0;
        dut.dsp_addr = 0;
        dut.dsp_wdata = 0;
        dut.dsp_rd_en = 0;
        dut.dsp_wr_en = 0;
        dut.cpu_addr = 0;
        dut.cpu_wdata = 0;
        dut.cpu_rd_en = 0;
        dut.cpu_wr_en = 0;
        dut.dma_count_current = 0;
        dut.dma_done = 0;
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

    uint32_t cpu_count() {
        dut.cpu_addr = kCount;
        dut.eval();
        return dut.cpu_rdata & 0x00ffffff;
    }

    uint32_t dsp_count() {
        dut.dsp_addr = kCount;
        dut.eval();
        return dut.dsp_rdata & 0x00ffffff;
    }

    void write_both(uint32_t value) {
        dut.cpu_addr = kCount;
        dut.cpu_wdata = value;
        dut.cpu_wr_en = 1;
        tick();
        dut.cpu_wr_en = 0;
        dut.dsp_addr = kCount;
        dut.dsp_wdata = value;
        dut.dsp_wr_en = 1;
        tick();
        dut.dsp_wr_en = 0;
        dut.eval();
    }

    void pulse_reset() {
        dut.rst_n = 0;
        tick();
        dut.rst_n = 1;
        dut.eval();
    }
};

bool cadence_case() {
    Harness h;
    constexpr std::array<uint32_t, 13> expected = {
        0, 1, 2, 2, 3, 4, 4, 5, 6, 6, 7, 8, 8
    };
    bool cadence = h.cpu_count() == expected[0];
    bool coherent = h.cpu_count() == h.dsp_count();
    for (unsigned i = 1; i < expected.size(); ++i) {
        h.tick();
        cadence &= h.cpu_count() == expected[i];
        coherent &= h.cpu_count() == h.dsp_count();
    }
    const bool pass = cadence && coherent;
    std::printf("%s HPC counter cadence=%u coherent=%u final=%06x\n",
                pass ? "PASS" : "FAIL", cadence, coherent, h.cpu_count());
    return pass;
}

bool readonly_and_dma_independent_case() {
    Harness h;
    for (unsigned i = 0; i < 30; ++i) {
        h.dut.dma_count_current = uint16_t(0xf000u - i * 17u);
        h.tick();
    }
    const uint32_t before = h.cpu_count();
    h.write_both(0xabcdef);
    const uint32_t after = h.cpu_count();
    const bool free_running = before == 20 && after == 22;
    const bool read_only = after != 0xabcdef && h.dsp_count() == after;
    const bool pass = free_running && read_only;
    std::printf("%s HPC counter read-only=%u DMA-independent=%u "
                "before=%06x after=%06x\n",
                pass ? "PASS" : "FAIL", read_only, free_running,
                before, after);
    return pass;
}

bool reset_and_wrap_case() {
    Harness h;
    for (unsigned i = 0; i < 7; ++i)
        h.tick();
    h.pulse_reset();
    const bool reset = h.cpu_count() == 0;

    // Seed only to reach the otherwise 25-million-cycle wrap boundary; the
    // observed transition still occurs through the synthesizable counter.
    h.dut.rootp->dsp56001_host__DOT__counter = 0x00fffffe;
    h.dut.rootp->dsp56001_host__DOT__counter_phase = 2;
    h.dut.eval();
    h.tick();
    const bool penultimate = h.cpu_count() == 0x00ffffff;
    h.tick();
    const bool wrapped = h.cpu_count() == 0;
    const bool pass = reset && penultimate && wrapped;
    std::printf("%s HPC counter reset=%u penultimate=%u wrap=%u final=%06x\n",
                pass ? "PASS" : "FAIL", reset, penultimate, wrapped,
                h.cpu_count());
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const bool pass = cadence_case() && readonly_and_dma_independent_case() &&
                      reset_and_wrap_case();
    std::printf("%s HPC1 24-bit GIO counter\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
