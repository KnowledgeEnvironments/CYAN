// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// HPC1 host/DSP handshake lifecycle oracle
//
// Run 33 complete bidirectional exchanges through the production host block.
// Every round proves payload visibility, the distinct CPU and DSP interrupt
// domains, mask behavior, acknowledge events, write-zero-to-clear, and clean
// re-arming before the next exchange.
///////////////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <cstdio>

#include "Vdsp56001_host.h"
#include "verilated.h"

namespace {

constexpr uint8_t kHandtx = 0x06;
constexpr uint8_t kHandrx = 0x07;
constexpr uint8_t kStatus = 0x08;
constexpr uint8_t kMask = 0x09;

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

    void idle() {
        dut.dsp_rd_en = 0;
        dut.dsp_wr_en = 0;
        dut.cpu_rd_en = 0;
        dut.cpu_wr_en = 0;
        dut.dma_done = 0;
        dut.eval();
    }

    void cpu_write(uint8_t address, uint32_t value) {
        idle();
        dut.cpu_addr = address;
        dut.cpu_wdata = value & 0x00ffffff;
        dut.cpu_wr_en = 1;
        tick();
        idle();
    }

    uint32_t cpu_read(uint8_t address, bool acknowledge = false) {
        idle();
        dut.cpu_addr = address;
        dut.cpu_rd_en = 1;
        dut.eval();
        const uint32_t value = dut.cpu_rdata & 0x00ffffff;
        if (acknowledge)
            tick();
        idle();
        return value;
    }

    void dsp_write(uint8_t address, uint32_t value) {
        idle();
        dut.dsp_addr = address;
        dut.dsp_wdata = value & 0x00ffffff;
        dut.dsp_wr_en = 1;
        tick();
        idle();
    }

    uint32_t dsp_read(uint8_t address, bool acknowledge = false) {
        idle();
        dut.dsp_addr = address;
        dut.dsp_rd_en = 1;
        dut.eval();
        const uint32_t value = dut.dsp_rdata & 0x00ffffff;
        if (acknowledge)
            tick();
        idle();
        return value;
    }
};

bool run_exchanges() {
    Harness h;
    h.cpu_write(kMask, 0x06); // CPU: HANDTX and HANDRX-consumed causes.
    h.dsp_write(kMask, 0x60); // DSP: HANDTX-ack and HANDRX-arrival causes.

    unsigned failures = 0;
    for (unsigned round = 0; round < 33; ++round) {
        const uint32_t rx_value = 0xa50000u | (round * 0x101u);
        const uint32_t tx_value = 0x5a0000u | (round * 0x103u);

        h.cpu_write(kHandrx, rx_value);
        const uint32_t dsp_arrival = h.dsp_read(kStatus);
        const bool rx_arrived = dsp_arrival == 0x40 && h.dut.irq_to_dsp;
        const uint32_t dsp_rx = h.dsp_read(kHandrx, true);
        const bool rx_consumed = dsp_rx == (rx_value & 0xffff) &&
                                 h.cpu_read(kStatus) == 0x04 &&
                                 h.dut.irq_to_cpu;
        h.dsp_write(kStatus, 0x00);
        const bool rx_acknowledged = h.dsp_read(kStatus) == 0 &&
                                     !h.dut.irq_to_dsp;

        h.dsp_write(kHandtx, tx_value);
        const bool tx_arrived = h.cpu_read(kStatus) == 0x06 &&
                                h.dut.irq_to_cpu;
        const uint32_t cpu_tx = h.cpu_read(kHandtx, true);
        const bool tx_consumed = cpu_tx == (tx_value & 0xffff) &&
                                 h.dsp_read(kStatus) == 0x20 &&
                                 h.dut.irq_to_dsp;

        h.cpu_write(kStatus, 0x00);
        h.dsp_write(kStatus, 0x00);
        const bool rearmed = h.cpu_read(kStatus) == 0 &&
                             h.dsp_read(kStatus) == 0 &&
                             !h.dut.irq_to_cpu && !h.dut.irq_to_dsp;

        const bool pass = rx_arrived && rx_consumed && rx_acknowledged &&
                          tx_arrived && tx_consumed && rearmed;
        if (!pass) {
            ++failures;
            std::printf("FAIL HPC handshake round=%u rx-arrive=%u "
                        "rx-consume=%u rx-ack=%u tx-arrive=%u "
                        "tx-consume=%u rearm=%u cpu=%02x dsp=%02x\n",
                        round, rx_arrived, rx_consumed, rx_acknowledged,
                        tx_arrived, tx_consumed, rearmed,
                        h.cpu_read(kStatus), h.dsp_read(kStatus));
        }
    }

    const bool pass = failures == 0;
    std::printf("%s HPC host/DSP handshake rounds=33 failures=%u "
                "rx-arrival=40 rx-consumed=04 tx-arrival=06 tx-ack=20\n",
                pass ? "PASS" : "FAIL", failures);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    return run_exchanges() ? 0 : 1;
}
