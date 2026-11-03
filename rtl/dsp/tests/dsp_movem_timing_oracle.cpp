// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 MOVEM production-core timing oracle
//
// DSP56001UMA3 Tables A-6, A-13 and A-14 define zero-wait-state MOVEM as
// 6 + ea + ap oscillator clocks.  Internal P memory makes ap zero; indexed,
// predecrement and absolute addressing add two ea clocks.  Thus short and
// ordinary indirect forms take six clocks and the three slow forms take eight.
// Every case also requires exactly one real P-data transfer at the expected
// address so a timing-only no-op cannot pass.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint32_t kSuccessor = 0x375500; // MOVE #<$55,R7.
constexpr uint32_t kStoreData = 0x123456;
constexpr uint32_t kReadData = 0x654321;

struct Case {
    const char *name;
    uint32_t instruction;
    bool two_word;
    bool read;
    uint16_t expected_address;
    unsigned expected_clocks;
};

constexpr std::array<Case, 10> kCases{{
    {"short-write",  0x07130c, false, false, 0x0013, 6},
    {"short-read",   0x07930c, false, true,  0x0013, 6},
    {"direct-write", 0x07608c, false, false, 0x0040, 6},
    {"direct-read",  0x07e08c, false, true,  0x0040, 6},
    {"index-write",  0x07688c, false, false, 0x0043, 8},
    {"index-read",   0x07e88c, false, true,  0x0043, 8},
    {"predec-write", 0x07788c, false, false, 0x003f, 8},
    {"predec-read",  0x07f88c, false, true,  0x003f, 8},
    {"absolute-write", 0x07708c, true, false, 0x0044, 8},
    {"absolute-read",  0x07f08c, true, true,  0x0044, 8},
}};

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};
    uint32_t instruction;
    uint16_t successor_address;
    uint64_t steps = 0;
    int64_t decode_step = -1;
    int64_t writeback_step = -1;
    int64_t successor_step = -1;
    unsigned reads = 0;
    unsigned writes = 0;
    uint16_t transfer_address = 0;
    uint32_t transfer_data = 0;

    explicit Harness(const Case &test)
        : instruction(test.instruction),
          successor_address(test.two_word ? 6 : 5) {
        // Establish R0=$40, N0=3, then leave one architectural spacing
        // instruction before the timed MOVEM.
        p[0] = 0x60f400; // MOVE #>$40,R0.
        p[1] = 0x000040;
        p[2] = 0x380300; // MOVE #<3,N0.
        p[3] = 0x000000; // NOP.
        p[4] = instruction;
        if (test.two_word)
            p[5] = test.expected_address;
        p[successor_address] = kSuccessor;
        p[successor_address + 1] = 0x0c0007; // Stable terminal JMP.
        p[test.expected_address] = kReadData;

        core.clk = 0;
        core.rst_n = 0;
        core.clock_enable = 1;
        core.halt = 0;
        core.run = 0;
        core.p_fetch_data = 0;
        core.p_fetch_ready = 1;
        core.x_rdata = core.y_rdata = 0;
        core.p_data_rdata = 0;
        core.x_ready = core.y_ready = 1;
        core.irq_a = core.irq_b = 0;
        core.irq_ssi_tx = core.irq_ssi_rx = 0;
        core.irq_ssi_tx_exception = core.irq_ssi_rx_exception = 0;
        core.irq_sci_tx = core.irq_sci_rx = 0;
        core.irq_sci_rx_exception = core.irq_sci_idle = 0;
        core.irq_sci_timer = 0;
        core.irq_priority = 0;
        core.dbg_reg_wr_en = 0;
        core.dbg_reg_wr_addr = 0;
        core.dbg_reg_wr_data = 0;
        for (unsigned i = 0; i < 4; ++i)
            tick();
        core.rst_n = 1;

        // A1 is the source/destination selected by dddddd=$0c.
        core.dbg_reg_wr_en = 1;
        core.dbg_reg_wr_addr = 0x0e;
        core.dbg_reg_wr_data = 0x00123456000000ULL;
        tick();
        core.dbg_reg_wr_en = 0;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch = core.p_fetch_en;
        const uint16_t fetch_address = core.p_fetch_addr;
        const bool read = core.p_data_rd_en;
        const bool write = core.p_data_wr_en;
        const uint16_t address = core.p_data_addr;
        const uint32_t write_data = core.p_data_wdata;
        if (core.state_out == 2 &&
            (core.p_fetch_data & 0x00ffffff) == instruction &&
            decode_step < 0)
            decode_step = static_cast<int64_t>(steps);
        if (core.state_out == 4 && core.cur_inst_out == instruction &&
            writeback_step < 0)
            writeback_step = static_cast<int64_t>(steps);
        if (core.state_out == 4 && core.cur_inst_out == kSuccessor &&
            successor_step < 0)
            successor_step = static_cast<int64_t>(steps);
        core.clk = 1;
        core.eval();
        if (fetch)
            core.p_fetch_data = p[fetch_address & 0xff] & 0x00ffffff;
        if (read) {
            ++reads;
            transfer_address = address;
            core.p_data_rdata = p[address & 0xff] & 0x00ffffff;
        }
        if (write) {
            ++writes;
            transfer_address = address;
            transfer_data = write_data & 0x00ffffff;
            p[address & 0xff] = transfer_data;
        }
        ++steps;
    }
};

bool run_case(const Case &test) {
    Harness h(test);
    h.core.run = 1;
    for (unsigned i = 0; i < 100 && h.successor_step < 0; ++i)
        h.tick();

    const int64_t clocks = h.decode_step >= 0 && h.writeback_step >= 0 ?
        h.writeback_step - h.decode_step + 1 : -1;
    const int64_t successor_gap = h.writeback_step >= 0 &&
                                  h.successor_step >= 0 ?
        h.successor_step - h.writeback_step : -1;
    const uint32_t a1 = (h.core.acc_a_out >> 24) & 0x00ffffff;
    const bool event_ok = test.read ?
        (h.reads == 1 && h.writes == 0 &&
         h.transfer_address == test.expected_address && a1 == kReadData) :
        (h.reads == 0 && h.writes == 1 &&
         h.transfer_address == test.expected_address &&
         h.transfer_data == kStoreData);
    const bool pass = clocks == test.expected_clocks &&
                      successor_gap == 2 && event_ok;
    std::printf("%s MOVEM timing %-14s clocks=%lld/%u gap=%lld/2 "
                "P-rw=%u/%u@%04x data=%06x A1=%06x event=%u\n",
                pass ? "PASS" : "FAIL", test.name,
                static_cast<long long>(clocks), test.expected_clocks,
                static_cast<long long>(successor_gap), h.reads, h.writes,
                h.transfer_address, h.transfer_data, a1, event_ok);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    unsigned failures = 0;
    for (const auto &test : kCases)
        failures += !run_case(test);
    std::printf("MOVEM TIMING SUMMARY cases=%zu failures=%u\n",
                kCases.size(), failures);
    return failures ? 1 : 0;
}
