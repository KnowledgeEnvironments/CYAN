// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

#include "Vdsp56001_host.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace {

constexpr uint8_t R_HANDTX = 0x06;
constexpr uint8_t R_HANDRX = 0x07;
constexpr uint8_t C_DMACTRL = 0x04;
constexpr uint8_t C_CINTSTAT = 0x08;
constexpr uint8_t C_CINTMASK = 0x09;

[[noreturn]] void fail(const char *message, uint32_t actual = 0,
                       uint32_t expected = 0) {
    std::fprintf(stderr,
                 "FAIL dsp_host_dma_w0c_collision: %s actual=0x%06x "
                 "expected=0x%06x\n",
                 message, actual & 0xffffffu, expected & 0xffffffu);
    std::exit(1);
}

void eval_low(Vdsp56001_host &top) {
    top.clk = 0;
    top.eval();
}

void tick(Vdsp56001_host &top) {
    eval_low(top);
    top.clk = 1;
    top.eval();
    top.clk = 0;
    top.eval();
}

void clear_requests(Vdsp56001_host &top) {
    top.dsp_rd_en = 0;
    top.dsp_wr_en = 0;
    top.cpu_rd_en = 0;
    top.cpu_wr_en = 0;
    top.dma_done = 0;
}

void defaults(Vdsp56001_host &top) {
    top.clk = 0;
    top.rst_n = 1;
    top.dsp_addr = 0;
    top.dsp_wdata = 0;
    top.cpu_addr = 0;
    top.cpu_wdata = 0;
    top.dma_count_current = 0;
    clear_requests(top);
    top.eval();
}

void reset(Vdsp56001_host &top) {
    defaults(top);
    top.rst_n = 0;
    for (unsigned cycle = 0; cycle < 3; ++cycle)
        tick(top);
    top.rst_n = 1;
    tick(top);
}

void cpu_write(Vdsp56001_host &top, uint8_t address, uint32_t data) {
    clear_requests(top);
    top.cpu_addr = address;
    top.cpu_wdata = data;
    top.cpu_wr_en = 1;
    tick(top);
    clear_requests(top);
    eval_low(top);
}

uint32_t cpu_read(Vdsp56001_host &top, uint8_t address) {
    clear_requests(top);
    top.cpu_addr = address;
    top.cpu_rd_en = 1;
    eval_low(top);
    const uint32_t value = top.cpu_rdata & 0xffffffu;
    clear_requests(top);
    eval_low(top);
    return value;
}

void dsp_write(Vdsp56001_host &top, uint8_t address, uint32_t data) {
    clear_requests(top);
    top.dsp_addr = address;
    top.dsp_wdata = data;
    top.dsp_wr_en = 1;
    tick(top);
    clear_requests(top);
    eval_low(top);
}

void dsp_read_event(Vdsp56001_host &top, uint8_t address) {
    clear_requests(top);
    top.dsp_addr = address;
    top.dsp_rd_en = 1;
    tick(top);
    clear_requests(top);
    eval_low(top);
}

} // namespace

int main() {
    Vdsp56001_host top;
    reset(top);

    // Seed the two unrelated CPU-facing causes and arm DMA.  These establish
    // that the collision repair preserves both ordinary W0C and GO clearing.
    dsp_write(top, R_HANDTX, 0x1234);
    dsp_read_event(top, R_HANDRX);
    cpu_write(top, C_DMACTRL, 0x1);
    cpu_write(top, C_CINTMASK, 0x5);

    if (cpu_read(top, C_CINTSTAT) != 0x6)
        fail("could not seed TX/RX status", cpu_read(top, C_CINTSTAT), 0x6);
    if (!top.dma_go)
        fail("DMA GO was not armed", top.dma_go, 1);
    if (!top.irq_to_cpu)
        fail("seeded RX cause did not assert masked CPU IRQ", top.irq_to_cpu, 1);

    // Clear TX while preserving RX on the exact edge that DMA completes.
    // W0C data 101 preserves existing DMA/RX and clears TX.  Since DMA was
    // previously clear, the new completion must be ORed after the W0C clear.
    clear_requests(top);
    top.cpu_addr = C_CINTSTAT;
    top.cpu_wdata = 0x5;
    top.cpu_wr_en = 1;
    top.dma_done = 1;
    tick(top);
    clear_requests(top);
    eval_low(top);

    const uint32_t collision_status = cpu_read(top, C_CINTSTAT);
    if (collision_status != 0x5)
        fail("DMA completion was lost across simultaneous CPU W0C",
             collision_status, 0x5);
    if (top.dma_go)
        fail("DMA completion did not clear GO", top.dma_go, 0);
    if (!top.irq_to_cpu)
        fail("retained DMA/RX causes did not assert CPU IRQ", top.irq_to_cpu, 1);

    // Ordinary W0C remains intact after the collision: clear RX while
    // preserving DMA, then clear DMA itself and observe IRQ deassertion.
    cpu_write(top, C_CINTSTAT, 0x1);
    if (cpu_read(top, C_CINTSTAT) != 0x1)
        fail("post-collision W0C did not preserve DMA and clear RX",
             cpu_read(top, C_CINTSTAT), 0x1);
    cpu_write(top, C_CINTSTAT, 0x6);
    if (cpu_read(top, C_CINTSTAT) != 0x0)
        fail("post-collision W0C did not clear DMA",
             cpu_read(top, C_CINTSTAT), 0x0);
    if (top.irq_to_cpu)
        fail("CPU IRQ remained asserted after all causes cleared",
             top.irq_to_cpu, 0);

    std::puts("PASS dsp_host_dma_w0c_collision status=0x5 go=0 w0c=preserved cycles=bounded");
    return 0;
}
