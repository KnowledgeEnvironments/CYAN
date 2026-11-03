// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 SSI data/status lifecycle oracle
//
// Exercise the production peripheral at its word-oriented board boundary.
// The external audio glue supplies complete time-slot events, so this oracle
// grades the architectural TX/RX registers, status, exceptions, interrupt
// enables, individual reset, RESET-instruction behavior, and STOP behavior.
///////////////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <cstdio>

#include "Vdsp56001_periph.h"
#include "verilated.h"

namespace {

constexpr uint8_t kPcc = 0x21;
constexpr uint8_t kCra = 0x2c;
constexpr uint8_t kCrb = 0x2d;
constexpr uint8_t kSsisr = 0x2e;
constexpr uint8_t kTxrx = 0x2f;

constexpr uint32_t kTfs = 1u << 2;
constexpr uint32_t kRfs = 1u << 3;
constexpr uint32_t kTue = 1u << 4;
constexpr uint32_t kRoe = 1u << 5;
constexpr uint32_t kTde = 1u << 6;
constexpr uint32_t kRdf = 1u << 7;

constexpr uint32_t kTe = 1u << 12;
constexpr uint32_t kRe = 1u << 13;
constexpr uint32_t kTie = 1u << 14;
constexpr uint32_t kRie = 1u << 15;

class Harness {
public:
    Vdsp56001_periph dut;

    Harness() {
        dut.clk = 0;
        dut.rst_n = 0;
        dut.clock_enable = 0;
        dut.soft_reset = 0;
        dut.stop_reset = 0;
        dut.addr = 0;
        dut.wdata = 0;
        dut.rd_en = 0;
        dut.wr_en = 0;
        dut.ssi_tx_ready = 0;
        dut.ssi_rx_valid = 0;
        dut.ssi_rx_data = 0;
        dut.sci_tx_ready = 0;
        dut.sci_tx_done = 0;
        dut.sci_rx_valid = 0;
        dut.sci_rx_data = 0;
        dut.sci_rx_r8 = 0;
        dut.sci_rx_framing_error = 0;
        dut.sci_rx_parity_error = 0;
        dut.sci_rx_idle = 0;
        dut.sci_idle_ack = 0;
        dut.sci_timer_ack = 0;
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

    void write(uint8_t address, uint32_t data) {
        dut.addr = address;
        dut.wdata = data & 0x00ffffff;
        dut.wr_en = 1;
        tick();
        dut.wr_en = 0;
        dut.eval();
    }

    uint32_t peek(uint8_t address) {
        dut.addr = address;
        dut.eval();
        return dut.rdata & 0x00ffffff;
    }

    uint32_t read(uint8_t address) {
        const uint32_t value = peek(address);
        dut.rd_en = 1;
        tick();
        dut.rd_en = 0;
        dut.eval();
        return value;
    }

    void enable(uint32_t crb) {
        write(kPcc, 0x0001f8);
        write(kCra, 0x005a3c);
        write(kCrb, crb);
    }

    bool tx_slot() {
        dut.ssi_tx_ready = 1;
        tick();
        const bool accepted = dut.ssi_tx_en;
        dut.ssi_tx_ready = 0;
        tick();
        return accepted;
    }

    void rx_word(uint32_t data) {
        dut.ssi_rx_data = data & 0x00ffffff;
        dut.ssi_rx_valid = 1;
        tick();
        dut.ssi_rx_valid = 0;
        tick();
    }

    void pulse_soft_reset() {
        dut.soft_reset = 1;
        tick();
        dut.soft_reset = 0;
        tick();
    }

    void pulse_stop_reset() {
        dut.stop_reset = 1;
        tick();
        dut.stop_reset = 0;
        tick();
    }
};

bool tx_case() {
    Harness h;
    h.enable(kTe | kTie);
    h.write(kTxrx, 0x123456);
    const bool loaded = (h.peek(kSsisr) & kTde) == 0 &&
                        !h.dut.irq_ssi_tx;
    const bool accepted = h.tx_slot();
    const uint32_t first_status = h.peek(kSsisr);
    const bool completed = accepted && h.dut.ssi_tx_data == 0x123456 &&
                           (first_status & (kTde | kTfs)) == (kTde | kTfs) &&
                           h.dut.irq_ssi_tx && !h.dut.irq_ssi_tx_exception;

    const bool retransmitted = h.tx_slot();
    const bool underrun = retransmitted && (h.peek(kSsisr) & kTue) &&
                          h.dut.irq_ssi_tx_exception;
    h.read(kSsisr);
    h.write(kTxrx, 0x654321);
    const bool cleared = (h.peek(kSsisr) & (kTue | kTde)) == 0 &&
                         !h.dut.irq_ssi_tx;

    const bool pass = loaded && completed && underrun && cleared;
    std::printf("%s SSI TX load=%u accepted=%u status=%02x underrun=%u "
                "cleared=%u data=%06x\n",
                pass ? "PASS" : "FAIL", loaded, accepted,
                first_status & 0xff, underrun, cleared,
                h.dut.ssi_tx_data & 0x00ffffff);
    return pass;
}

bool rx_case() {
    Harness h;
    h.enable(kRe | kRie);
    h.rx_word(0x0abcde);
    const uint32_t first_status = h.peek(kSsisr);
    const bool received = (first_status & (kRdf | kRfs)) == (kRdf | kRfs) &&
                          h.dut.irq_ssi_rx &&
                          !h.dut.irq_ssi_rx_exception &&
                          h.peek(kTxrx) == 0x0abcde;

    h.rx_word(0x654321);
    const bool overrun = (h.peek(kSsisr) & (kRdf | kRoe)) ==
                         (kRdf | kRoe) &&
                         h.dut.irq_ssi_rx_exception &&
                         h.peek(kTxrx) == 0x0abcde;
    h.read(kSsisr);
    const uint32_t retained = h.read(kTxrx);
    const bool cleared = retained == 0x0abcde &&
                         (h.peek(kSsisr) & (kRdf | kRoe)) == 0 &&
                         !h.dut.irq_ssi_rx;

    h.dut.addr = kTxrx;
    h.dut.rd_en = 1;
    h.dut.ssi_rx_data = 0x13579b;
    h.dut.ssi_rx_valid = 1;
    h.tick();
    h.dut.rd_en = 0;
    h.dut.ssi_rx_valid = 0;
    h.tick();
    const bool collision_preserved = (h.peek(kSsisr) & kRdf) &&
                                     h.peek(kTxrx) == 0x13579b;

    const bool pass = received && overrun && cleared && collision_preserved;
    std::printf("%s SSI RX received=%u status=%02x overrun=%u cleared=%u "
                "collision=%u data=%06x\n",
                pass ? "PASS" : "FAIL", received, first_status & 0xff,
                overrun, cleared, collision_preserved,
                h.peek(kTxrx) & 0x00ffffff);
    return pass;
}

bool reset_case() {
    Harness h;
    h.enable(kTe | kRe | kTie | kRie);
    h.write(kTxrx, 0x2468ac);
    h.rx_word(0x112233);
    h.pulse_stop_reset();
    const bool stop_preserves_control = h.peek(kPcc) == 0x0001f8 &&
                                        h.peek(kCra) == 0x005a3c &&
                                        h.peek(kCrb) ==
                                            (kTe | kRe | kTie | kRie);
    const bool stop_resets_status = h.peek(kSsisr) == kTde;
    const bool stop_preserves_data = h.dut.ssi_tx_data == 0x2468ac &&
                                     h.peek(kTxrx) == 0x112233;

    h.pulse_soft_reset();
    const bool soft_clears_control = h.peek(kPcc) == 0 &&
                                     h.peek(kCra) == 0 &&
                                     h.peek(kCrb) == 0;
    const bool soft_preserves_data = h.dut.ssi_tx_data == 0x2468ac &&
                                     h.peek(kTxrx) == 0x112233;

    h.enable(kTe | kRe);
    h.write(kTxrx, 0xabcdef);
    h.write(kPcc, 0);
    h.tick();
    const bool individual_reset = h.peek(kSsisr) == kTde &&
                                  !h.dut.ssi_tx_en &&
                                  !h.dut.irq_ssi_tx && !h.dut.irq_ssi_rx;

    const bool pass = stop_preserves_control && stop_resets_status &&
                      stop_preserves_data && soft_clears_control &&
                      soft_preserves_data && individual_reset;
    std::printf("%s SSI reset STOP-control=%u STOP-status=%u STOP-data=%u "
                "RESET-control=%u RESET-data=%u individual=%u\n",
                pass ? "PASS" : "FAIL", stop_preserves_control,
                stop_resets_status, stop_preserves_data,
                soft_clears_control, soft_preserves_data, individual_reset);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const bool pass = tx_case() && rx_case() && reset_case();
    std::printf("%s DSP56001 SSI data lifecycle\n",
                pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
