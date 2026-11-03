// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

#include "Vdsp56001_host.h"

#include <cstdint>
#include <cstdio>

namespace {

constexpr uint8_t R_HANDTX = 0x06;
constexpr uint8_t R_HANDRX = 0x07;
constexpr uint8_t C_DMACTRL = 0x04;
constexpr uint8_t C_CINTSTAT = 0x08;
constexpr uint8_t C_CINTMASK = 0x09;

unsigned failures = 0;

void expect_equal(const char *boundary, uint32_t actual, uint32_t expected) {
    if (actual == expected)
        return;
    std::fprintf(stderr,
                 "FAIL dsp_host_txrx_w0c_collision: %s actual=0x%06x "
                 "expected=0x%06x\n",
                 boundary, actual & 0xffffffu, expected & 0xffffffu);
    ++failures;
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

void collide_tx_dma_with_w0c(Vdsp56001_host &top) {
    clear_requests(top);
    top.dsp_addr = R_HANDTX;
    top.dsp_wdata = 0x1234;
    top.dsp_wr_en = 1;
    top.cpu_addr = C_CINTSTAT;
    top.cpu_wdata = 0x4; // Preserve old RX; clear old DMA/TX.
    top.cpu_wr_en = 1;
    top.dma_done = 1;
    tick(top);
    clear_requests(top);
    eval_low(top);
}

void collide_rx_dma_with_w0c(Vdsp56001_host &top) {
    clear_requests(top);
    top.dsp_addr = R_HANDRX;
    top.dsp_rd_en = 1;
    top.cpu_addr = C_CINTSTAT;
    top.cpu_wdata = 0x2; // Preserve old TX; clear old DMA/RX.
    top.cpu_wr_en = 1;
    top.dma_done = 1;
    tick(top);
    clear_requests(top);
    eval_low(top);
}

void test_tx_collision(Vdsp56001_host &top) {
    reset(top);

    // Old RX is an unrelated retained cause.  Only a newly retained TX event
    // can assert the selected CPU interrupt after the collision.
    dsp_read_event(top, R_HANDRX);
    cpu_write(top, C_DMACTRL, 0x1);
    cpu_write(top, C_CINTMASK, 0x2);
    expect_equal("TX seed status", cpu_read(top, C_CINTSTAT), 0x4);
    expect_equal("TX seed IRQ", top.irq_to_cpu, 0x0);
    expect_equal("TX seed GO", top.dma_go, 0x1);

    collide_tx_dma_with_w0c(top);

    // Old RX follows the W0C preserve bit.  New DMA and TX must both win.
    expect_equal("TX event lost across CPU W0C",
                 cpu_read(top, C_CINTSTAT), 0x7);
    expect_equal("TX event did not assert selected CPU IRQ", top.irq_to_cpu,
                 0x1);
    expect_equal("TX collision did not clear DMA GO", top.dma_go, 0x0);

    cpu_write(top, C_CINTSTAT, 0x4);
    expect_equal("TX post-collision W0C", cpu_read(top, C_CINTSTAT), 0x4);
    expect_equal("TX post-collision IRQ clear", top.irq_to_cpu, 0x0);
    cpu_write(top, C_CINTSTAT, 0x0);
    expect_equal("TX final W0C", cpu_read(top, C_CINTSTAT), 0x0);
}

void test_rx_collision(Vdsp56001_host &top) {
    reset(top);

    // Old TX is an unrelated retained cause.  Only a newly retained RX event
    // can assert the selected CPU interrupt after the collision.
    dsp_write(top, R_HANDTX, 0x5678);
    cpu_write(top, C_DMACTRL, 0x1);
    cpu_write(top, C_CINTMASK, 0x4);
    expect_equal("RX seed status", cpu_read(top, C_CINTSTAT), 0x2);
    expect_equal("RX seed IRQ", top.irq_to_cpu, 0x0);
    expect_equal("RX seed GO", top.dma_go, 0x1);

    collide_rx_dma_with_w0c(top);

    // Old TX follows the W0C preserve bit.  New DMA and RX must both win.
    expect_equal("RX event lost across CPU W0C",
                 cpu_read(top, C_CINTSTAT), 0x7);
    expect_equal("RX event did not assert selected CPU IRQ", top.irq_to_cpu,
                 0x1);
    expect_equal("RX collision did not clear DMA GO", top.dma_go, 0x0);

    cpu_write(top, C_CINTSTAT, 0x2);
    expect_equal("RX post-collision W0C", cpu_read(top, C_CINTSTAT), 0x2);
    expect_equal("RX post-collision IRQ clear", top.irq_to_cpu, 0x0);
    cpu_write(top, C_CINTSTAT, 0x0);
    expect_equal("RX final W0C", cpu_read(top, C_CINTSTAT), 0x0);
}

} // namespace

int main() {
    Vdsp56001_host top;
    test_tx_collision(top);
    test_rx_collision(top);

    if (failures != 0) {
        std::fprintf(stderr,
                     "FAIL dsp_host_txrx_w0c_collision failures=%u\n",
                     failures);
        return 1;
    }

    std::puts("PASS dsp_host_txrx_w0c_collision tx=7 rx=7 dma=retained "
              "go=0 w0c=preserved irq=selected");
    return 0;
}
