// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 Class-I X:R / R:Y selector cross-product oracle
//
// DSP56001UMA2 A-131--A-144 defines four primary selectors and four
// independent accumulator/input-register selectors in both memory spaces.
// Cross every selector in both load/store directions so a representative
// A/X0 or A/Y0 case cannot conceal a swapped field or stale source.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint8_t kStateWritebackFetch = 4;
constexpr uint32_t kSetupR0 = 0x60f400; // MOVE #>$1234,R0

constexpr std::array<uint8_t, 4> kXPrimary{{0x04, 0x05, 0x0e, 0x0f}};
constexpr std::array<uint8_t, 4> kYPrimary{{0x06, 0x07, 0x0e, 0x0f}};
constexpr std::array<uint32_t, 4> kXInitial{{
    0x100001, 0x200002, 0x500005, 0x600006,
}};
constexpr std::array<uint32_t, 4> kYInitial{{
    0x300003, 0x400004, 0x500005, 0x600006,
}};

constexpr uint32_t register_to_b(uint8_t source) {
    return 0x200f00 | (static_cast<uint32_t>(source) << 13);
}

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 16> p{};
    unsigned x_reads = 0;
    unsigned x_writes = 0;
    unsigned y_reads = 0;
    unsigned y_writes = 0;
    uint16_t x_addr = 0;
    uint16_t y_addr = 0;
    uint32_t x_data = 0;
    uint32_t y_data = 0;

    Harness(uint32_t opcode, uint8_t primary, uint8_t independent) {
        p[0] = kSetupR0;
        p[1] = 0x001234;
        p[2] = 0x000000; // Settle explicit R0 write before pointer use.
        p[3] = opcode;
        p[4] = register_to_b(primary);
        p[5] = register_to_b(independent);
        p[6] = 0x0af080; // JMP >$0006.
        p[7] = 0x000006;

        core.clk = 0;
        core.rst_n = 0;
        core.clock_enable = 1;
        core.halt = 0;
        core.run = 0;
        core.p_fetch_data = 0;
        core.p_fetch_ready = 1;
        core.x_rdata = 0x710007;
        core.x_ready = 1;
        core.y_rdata = 0x720008;
        core.y_ready = 1;
        core.p_data_rdata = 0;
        core.irq_a = 0;
        core.irq_b = 0;
        core.irq_ssi_tx = 0;
        core.irq_ssi_rx = 0;
        core.irq_ssi_tx_exception = 0;
        core.irq_ssi_rx_exception = 0;
        core.irq_sci_tx = 0;
        core.irq_sci_rx = 0;
        core.irq_sci_timer = 0;
        core.irq_sci_rx_exception = 0;
        core.irq_sci_idle = 0;
        core.irq_priority = 0;
        core.dbg_reg_wr_en = 0;
        core.dbg_reg_wr_addr = 0;
        core.dbg_reg_wr_data = 0;
        for (unsigned i = 0; i < 4; ++i)
            tick();
        core.rst_n = 1;

        debug_write(0x39, 0);
        debug_write(0x04, 0x100001);
        debug_write(0x05, 0x200002);
        debug_write(0x06, 0x300003);
        debug_write(0x07, 0x400004);
        debug_write(0x0e, 0x00500005000000ULL);
        debug_write(0x0f, 0x00600006000000ULL);
        core.run = 1;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        if (core.x_rd_en) {
            ++x_reads;
            x_addr = core.x_addr;
        }
        if (core.x_wr_en) {
            ++x_writes;
            x_addr = core.x_addr;
            x_data = core.x_wdata & 0x00ffffff;
        }
        if (core.y_rd_en) {
            ++y_reads;
            y_addr = core.y_addr;
        }
        if (core.y_wr_en) {
            ++y_writes;
            y_addr = core.y_addr;
            y_data = core.y_wdata & 0x00ffffff;
        }
        core.clk = 1;
        core.eval();
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 0x0f] & 0x00ffffff;
    }

    void debug_write(uint8_t address, uint64_t data) {
        core.dbg_reg_wr_addr = address;
        core.dbg_reg_wr_data = data;
        core.dbg_reg_wr_en = 1;
        tick();
        core.dbg_reg_wr_en = 0;
    }

    bool retire(uint32_t opcode) {
        for (unsigned i = 0; i < 100; ++i) {
            if (core.state_out == kStateWritebackFetch &&
                core.cur_inst_out == opcode) {
                tick();
                return true;
            }
            tick();
        }
        return false;
    }

    uint32_t b_word() const {
        return (core.acc_b_out >> 24) & 0x00ffffff;
    }
};

bool run_case(bool ry, bool load, unsigned primary_sel,
              unsigned independent_sel) {
    const auto &primary_codes = ry ? kYPrimary : kXPrimary;
    const auto &primary_initial = ry ? kYInitial : kXInitial;
    const uint8_t independent_code = ry ?
        ((independent_sel & 1) ? 0x05 : 0x04) :
        ((independent_sel & 1) ? 0x07 : 0x06);
    const uint32_t independent_expected =
        (independent_sel & 2) ? 0x600006 : 0x500005;

    uint32_t opcode = ry ? 0x106000 : 0x102000;
    opcode |= static_cast<uint32_t>(primary_sel) << (ry ? 16 : 18);
    opcode |= static_cast<uint32_t>(independent_sel) << (ry ? 18 : 16);
    if (load)
        opcode |= 0x008000;

    Harness h(opcode, primary_codes[primary_sel], independent_code);
    const bool setup = h.retire(kSetupR0);
    const bool moved = h.retire(opcode);
    const bool primary_read = h.retire(register_to_b(primary_codes[primary_sel]));
    const uint32_t primary_actual = h.b_word();
    const bool independent_read = h.retire(register_to_b(independent_code));
    const uint32_t independent_actual = h.b_word();
    const uint32_t memory_input = ry ? 0x720008 : 0x710007;
    const uint32_t primary_expected = load ? memory_input :
                                                primary_initial[primary_sel];
    const bool x_ok = ry ? (h.x_reads == 0 && h.x_writes == 0) :
        (load ? (h.x_reads == 1 && h.x_writes == 0 && h.x_addr == 0x1234) :
                (h.x_reads == 0 && h.x_writes == 1 && h.x_addr == 0x1234 &&
                 h.x_data == primary_initial[primary_sel]));
    const bool y_ok = !ry ? (h.y_reads == 0 && h.y_writes == 0) :
        (load ? (h.y_reads == 1 && h.y_writes == 0 && h.y_addr == 0x1234) :
                (h.y_reads == 0 && h.y_writes == 1 && h.y_addr == 0x1234 &&
                 h.y_data == primary_initial[primary_sel]));
    const bool pass = setup && moved && primary_read && independent_read &&
                      x_ok && y_ok && primary_actual == primary_expected &&
                      independent_actual == independent_expected &&
                      h.core.sr_out == 0;
    std::printf("%s Class-I %s %s P%u I%u opcode=%06x "
                "primary=%06x/%06x independent=%06x/%06x "
                "X=%uR/%uW Y=%uR/%uW\n",
                pass ? "PASS" : "FAIL", ry ? "R:Y" : "X:R",
                load ? "load " : "store", primary_sel, independent_sel,
                opcode, primary_actual, primary_expected,
                independent_actual, independent_expected,
                h.x_reads, h.x_writes, h.y_reads, h.y_writes);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    bool pass = true;
    for (bool ry : {false, true})
        for (bool load : {false, true})
            for (unsigned primary = 0; primary < 4; ++primary)
                for (unsigned independent = 0; independent < 4; ++independent)
                    pass &= run_case(ry, load, primary, independent);
    return pass ? 0 : 1;
}
