// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 fixed two-clock control/standalone timing oracle
//
// DSP56001UMA3 Table A-6 assigns two oscillator clocks to NOP, ANDI, ORI,
// Tcc, NORM and DIV.  Their architectural effects and legal encodings have
// independent production-core oracles; this matrix closes the common timing,
// successor-alignment and bus-quiet boundary without a private ISA model.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint32_t kSuccessor = 0x375500; // MOVE #<$55,R7

struct Case {
    const char *name;
    uint32_t opcode;
};

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 8> p{};
    uint32_t target;
    uint64_t cycles = 0;
    int64_t decode_cycle = -1;
    int64_t writeback_cycle = -1;
    int64_t successor_cycle = -1;
    unsigned x_reads = 0;
    unsigned x_writes = 0;
    unsigned y_reads = 0;
    unsigned y_writes = 0;

    explicit Harness(uint32_t opcode) : target(opcode) {
        p[0] = opcode;
        p[1] = kSuccessor;
        p[2] = 0x0c0002; // JMP $002

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
        core.irq_sci_rx_exception = core.irq_sci_idle = 0;
        core.irq_sci_timer = 0;
        core.irq_priority = 0;
        core.dbg_reg_wr_en = 0;
        core.dbg_reg_wr_addr = 0;
        core.dbg_reg_wr_data = 0;
        for (unsigned i = 0; i < 4; ++i)
            tick();
        core.rst_n = 1;
        core.run = 1;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch = core.p_fetch_en;
        const uint16_t address = core.p_fetch_addr;
        if (core.run && core.state_out == 2 &&
            (core.p_fetch_data & 0x00ffffff) == target && decode_cycle < 0)
            decode_cycle = cycles;
        if (decode_cycle >= 0 && core.state_out == 4 &&
            core.cur_inst_out == target && writeback_cycle < 0)
            writeback_cycle = cycles;
        if (writeback_cycle >= 0 && core.state_out == 4 &&
            core.cur_inst_out == kSuccessor && successor_cycle < 0)
            successor_cycle = cycles;
        if (core.run) {
            x_reads += core.x_rd_en;
            x_writes += core.x_wr_en;
            y_reads += core.y_rd_en;
            y_writes += core.y_wr_en;
        }
        core.clk = 1;
        core.eval();
        if (fetch)
            core.p_fetch_data = p[address & 7] & 0x00ffffff;
        ++cycles;
    }

    bool complete() {
        for (unsigned i = 0; i < 48 && successor_cycle < 0; ++i)
            tick();
        return successor_cycle >= 0;
    }

    int64_t instruction_clocks() const {
        return decode_cycle >= 0 && writeback_cycle >= 0 ?
            writeback_cycle - decode_cycle + 1 : -1;
    }

    int64_t successor_gap() const {
        return writeback_cycle >= 0 && successor_cycle >= 0 ?
            successor_cycle - writeback_cycle : -1;
    }
};

bool run_case(const Case &test) {
    Harness h(test.opcode);
    const bool completed = h.complete();
    const bool bus_quiet = h.x_reads == 0 && h.x_writes == 0 &&
                           h.y_reads == 0 && h.y_writes == 0;
    const bool pass = completed && h.instruction_clocks() == 2 &&
                      h.successor_gap() == 2 && bus_quiet;
    std::printf("%s FIXED timing %-10s opcode=%06x clocks=%lld/2 "
                "successor=%lld/2 bus=X%u/%u,Y%u/%u\n",
                pass ? "PASS" : "FAIL", test.name, test.opcode,
                static_cast<long long>(h.instruction_clocks()),
                static_cast<long long>(h.successor_gap()), h.x_reads,
                h.x_writes, h.y_reads, h.y_writes);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const std::array<Case, 13> cases{{
        {"NOP", 0x000000},
        {"ORI-MR", 0x000af8},
        {"ANDI-MR", 0x00f0b8},
        {"ORI-CCR", 0x004af9},
        {"ANDI-CCR", 0x002db9},
        {"ORI-OMR", 0x0006fa},
        {"ANDI-OMR", 0x00fbba},
        {"TGT-A-B", 0x027008},
        {"TGT-dual", 0x03720d},
        {"NORM-R3-A", 0x01db15},
        {"NORM-R7-B", 0x01df1d},
        {"DIV-X0-A", 0x018040},
        {"DIV-Y1-B", 0x018078},
    }};

    unsigned failures = 0;
    for (const auto &test : cases)
        failures += !run_case(test);
    std::printf("FIXED TIMING SUMMARY cases=%zu failures=%u\n", cases.size(),
                failures);
    return failures == 0 ? 0 : 1;
}
