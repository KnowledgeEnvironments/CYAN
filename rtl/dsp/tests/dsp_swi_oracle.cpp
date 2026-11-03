// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 software-interrupt boundary oracle
//
// This instantiates the production core directly.  The program executes SWI,
// whose architectural long-interrupt vector is P:$0006/$0007.  DSP56001UMA2
// defines SWI as exactly eight oscillator clocks.  The vector JSR must stack
// the instruction following SWI, raise the interrupt mask to IPL 3, and RTI
// must restore both the resume PC and the original SR.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint16_t kSwiVector = 0x0006;
constexpr uint16_t kHandler = 0x0020;
constexpr uint32_t kInitialSr = 0x000055;
constexpr uint32_t kSwi = 0x000006;

class SwiHarness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};
    unsigned vector_first_fetches = 0;
    unsigned vector_second_fetches = 0;
    unsigned irqa_vector_fetches = 0;
    unsigned irqa_second_fetches = 0;
    unsigned handler_fetches = 0;
    unsigned successor_fetches = 0;
    uint16_t first_vector = 0xffff;
    uint64_t steps = 0;
    int64_t swi_decode_step = -1;
    int64_t swi_writeback_step = -1;
    bool successor_before_vector = false;
    bool handler_mask_was_ipl3 = false;
    bool successor_sr_was_restored = false;

    SwiHarness() {
        p[0x00] = kSwi;
        p[0x01] = 0x0af080; // JMP >$0001 (stable post-RTI loop)
        p[0x02] = 0x000001;
        p[0x06] = 0x0bf080; // SWI long vector: JSR >$0020
        p[0x07] = kHandler;
        p[0x08] = 0x0bf080; // IRQA long vector: JSR >$0030
        p[0x09] = 0x000030;
        p[kHandler] = 0x000004; // RTI
        p[0x30] = 0x000004;     // RTI

        core.clk = 0;
        core.rst_n = 0;
        core.clock_enable = 1;
        core.halt = 0;
        core.run = 0;
        core.p_fetch_data = 0;
        core.p_fetch_ready = 1;
        core.x_rdata = 0;
        core.x_ready = 1;
        core.y_rdata = 0;
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

        core.dbg_reg_wr_en = 1;
        core.dbg_reg_wr_addr = 0x39;
        core.dbg_reg_wr_data = kInitialSr;
        tick();
        core.dbg_reg_wr_en = 0;
        core.run = 1;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;

        if (core.state_out == 2 &&
            (core.p_fetch_data & 0x00ffffff) == kSwi &&
            swi_decode_step < 0)
            swi_decode_step = static_cast<int64_t>(steps);
        if (core.state_out == 4 && core.cur_inst_out == kSwi &&
            swi_writeback_step < 0)
            swi_writeback_step = static_cast<int64_t>(steps);

        if (fetch_en) {
            switch (fetch_addr) {
            case 0x0006:
                if (first_vector == 0xffff)
                    first_vector = fetch_addr;
                ++vector_first_fetches;
                break;
            case 0x0007:
                ++vector_second_fetches;
                break;
            case 0x0008:
                if (first_vector == 0xffff)
                    first_vector = fetch_addr;
                ++irqa_vector_fetches;
                break;
            case 0x0009:
                ++irqa_second_fetches;
                break;
            case kHandler:
                ++handler_fetches;
                break;
            case 0x0001:
                if (vector_first_fetches == 0)
                    successor_before_vector = true;
                ++successor_fetches;
                break;
            default:
                break;
            }
        }

        core.clk = 1;
        core.eval();
        if (fetch_en) {
            core.p_fetch_data = p[fetch_addr & 0xff] & 0x00ffffff;
            if (fetch_addr == 0x0008)
                core.irq_a = 0;
            if (fetch_addr == kHandler)
                handler_mask_was_ipl3 = ((core.sr_out >> 8) & 0x3) == 0x3;
            if (fetch_addr == 0x0001 && handler_fetches != 0)
                successor_sr_was_restored = core.sr_out == kInitialSr;
        }
        ++steps;
    }

    void run_cycles(unsigned count) {
        for (unsigned i = 0; i < count; ++i)
            tick();
    }

    int64_t swi_clocks() const {
        return swi_decode_step >= 0 && swi_writeback_step >= 0 ?
            swi_writeback_step - swi_decode_step + 1 : -1;
    }
};

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    SwiHarness h;
    h.run_cycles(80);

    const bool ordinary = h.vector_first_fetches == 1 &&
                          h.vector_second_fetches == 1 &&
                          h.handler_fetches == 1 &&
                          h.successor_fetches != 0 &&
                          !h.successor_before_vector &&
                          h.handler_mask_was_ipl3 &&
                          h.successor_sr_was_restored &&
                          h.swi_clocks() == 8;
    std::printf("%s SWI long-vector boundary: clocks=%lld/8 "
                "vector=%u/%u handler=%u successor=%u early=%u "
                "mask3=%u restored=%u\n",
                ordinary ? "PASS" : "FAIL",
                static_cast<long long>(h.swi_clocks()),
                h.vector_first_fetches, h.vector_second_fetches, h.handler_fetches,
                h.successor_fetches, h.successor_before_vector,
                h.handler_mask_was_ipl3, h.successor_sr_was_restored);

    // A simultaneously serviceable external input must not outrank the
    // nonmaskable software interrupt.  It remains pending while SWI runs at
    // IPL 3 and is then serviced after RTI restores the original mask.
    SwiHarness coincident;
    coincident.core.irq_priority = 0x03; // level-mode IRQA, IPL 3
    for (unsigned i = 0; i < 20 && coincident.core.state_out != 2; ++i)
        coincident.tick();
    coincident.core.irq_a = 1;
    coincident.run_cycles(100);
    const bool priority = coincident.first_vector == kSwiVector &&
                          coincident.vector_first_fetches == 1 &&
                          coincident.vector_second_fetches == 1 &&
                          coincident.irqa_vector_fetches == 1 &&
                          coincident.irqa_second_fetches == 1;
    std::printf("%s coincident SWI/IRQA priority: first=$%04x "
                "swi=%u/%u irqa=%u/%u\n", priority ? "PASS" : "FAIL",
                coincident.first_vector, coincident.vector_first_fetches,
                coincident.vector_second_fetches,
                coincident.irqa_vector_fetches,
                coincident.irqa_second_fetches);
    return ordinary && priority ? 0 : 1;
}

