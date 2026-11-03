// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

// Production-core DSP56001 TFR source/destination matrix.
// DSP56001UMA2 A-216--A-217 defines B/A and X0/X1/Y0/Y1 sources into
// either accumulator, with the full source value and SR preserved.
#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {
constexpr uint64_t kAInitial = 0x00112233445566ULL;
constexpr uint64_t kBInitial = 0xffeeddccbbaa99ULL;
constexpr uint32_t kX0 = 0x123456;
constexpr uint32_t kX1 = 0x923456;
constexpr uint32_t kY0 = 0x345678;
constexpr uint32_t kY1 = 0xb45678;
constexpr uint32_t kSr = 0x000055;

uint64_t data_to_acc(uint32_t value) {
    return ((value & 0x800000) ? 0xff000000000000ULL : 0) |
           (static_cast<uint64_t>(value) << 24);
}

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 8> p{};
    unsigned x_reads = 0, x_writes = 0, y_reads = 0, y_writes = 0;

    explicit Harness(uint32_t opcode) {
        p[0] = opcode;
        p[1] = 0x0c0001; // JMP $001.
        core.clk = 0;
        core.rst_n = 0;
        core.clock_enable = 1;
        core.halt = 0;
        core.run = 0;
        core.p_fetch_data = 0;
        core.p_fetch_ready = 1;
        core.x_rdata = core.y_rdata = core.p_data_rdata = 0;
        core.x_ready = core.y_ready = 1;
        core.irq_a = core.irq_b = 0;
        core.irq_ssi_tx = core.irq_ssi_rx = 0;
        core.irq_ssi_tx_exception = core.irq_ssi_rx_exception = 0;
        core.irq_sci_tx = core.irq_sci_rx = 0;
        core.irq_sci_timer = core.irq_sci_rx_exception = core.irq_sci_idle = 0;
        core.irq_priority = 0;
        core.dbg_reg_wr_en = 0;
        core.dbg_reg_wr_addr = 0;
        core.dbg_reg_wr_data = 0;
        for (unsigned i = 0; i < 4; ++i) tick();
        core.rst_n = 1;
        debug_write(0x0e, kAInitial);
        debug_write(0x0f, kBInitial);
        debug_write(0x04, kX0);
        debug_write(0x05, kX1);
        debug_write(0x06, kY0);
        debug_write(0x07, kY1);
        debug_write(0x39, kSr);
    }

    void debug_write(uint8_t reg, uint64_t value) {
        core.dbg_reg_wr_addr = reg;
        core.dbg_reg_wr_data = value;
        core.dbg_reg_wr_en = 1;
        tick();
        core.dbg_reg_wr_en = 0;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch = core.p_fetch_en;
        const uint16_t address = core.p_fetch_addr;
        x_reads += core.x_rd_en;
        x_writes += core.x_wr_en;
        y_reads += core.y_rd_en;
        y_writes += core.y_wr_en;
        core.clk = 1;
        core.eval();
        if (fetch) core.p_fetch_data = p[address & 7] & 0xffffff;
    }

    bool retire(uint32_t opcode) {
        core.run = 1;
        for (unsigned i = 0; i < 40; ++i) {
            if (core.state_out == 4 && core.cur_inst_out == opcode) {
                tick();
                return true;
            }
            tick();
        }
        return false;
    }
};

struct Case {
    const char *name;
    uint8_t source;
    bool dest_b;
    uint64_t expected;
};

bool run(const Case &test) {
    const uint32_t opcode = 0x200001 | (test.source << 4) |
                            (test.dest_b ? 0x08 : 0);
    Harness h(opcode);
    const bool retired = h.retire(opcode);
    const uint64_t a_expected = test.dest_b ? kAInitial : test.expected;
    const uint64_t b_expected = test.dest_b ? test.expected : kBInitial;
    const bool pass = retired && h.core.pc_out == 1 &&
                      h.core.acc_a_out == a_expected &&
                      h.core.acc_b_out == b_expected &&
                      h.core.sr_out == kSr &&
                      h.x_reads == 0 && h.x_writes == 0 &&
                      h.y_reads == 0 && h.y_writes == 0;
    std::printf("%s TFR %s opcode=%06x A=%014llx/%014llx "
                "B=%014llx/%014llx SR=%06x/%06x XY=%u/%u/%u/%u pc=%04x\n",
                pass ? "PASS" : "FAIL", test.name, opcode,
                static_cast<unsigned long long>(h.core.acc_a_out),
                static_cast<unsigned long long>(a_expected),
                static_cast<unsigned long long>(h.core.acc_b_out),
                static_cast<unsigned long long>(b_expected), h.core.sr_out,
                kSr, h.x_reads, h.x_writes, h.y_reads, h.y_writes,
                h.core.pc_out);
    return pass;
}
} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const Case cases[] = {
        {"B,A", 0, false, kBInitial},
        {"A,B", 0, true, kAInitial},
        {"X0,A", 4, false, data_to_acc(kX0)},
        {"X0,B", 4, true, data_to_acc(kX0)},
        {"Y0,A", 5, false, data_to_acc(kY0)},
        {"Y0,B", 5, true, data_to_acc(kY0)},
        {"X1,A", 6, false, data_to_acc(kX1)},
        {"X1,B", 6, true, data_to_acc(kX1)},
        {"Y1,A", 7, false, data_to_acc(kY1)},
        {"Y1,B", 7, true, data_to_acc(kY1)},
    };
    unsigned failures = 0;
    for (const Case &test : cases) failures += !run(test);
    std::printf("TFR SUMMARY cases=%zu failures=%u\n",
                sizeof(cases) / sizeof(cases[0]), failures);
    return failures ? 1 : 0;
}

