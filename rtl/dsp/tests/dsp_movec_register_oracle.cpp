// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 MOVEC register/control selector matrix
//
// Move a distinctive X0 value into every legal control-register selector and
// back into X1. Check architectural width, stack side effects and bus silence.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "Vdsp56001_core.h"
#include "Vdsp56001_core___024root.h"
#include "verilated.h"

namespace {

constexpr uint8_t kWriteback = 4;

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 16> p{};
    unsigned cycles = 0, xr = 0, xw = 0, yr = 0, yw = 0;

    Harness() {
        core.clk = 0; core.rst_n = 0; core.clock_enable = 1;
        core.halt = 0; core.run = 0; core.p_fetch_data = 0;
        core.p_fetch_ready = 1; core.x_rdata = 0; core.x_ready = 1;
        core.y_rdata = 0; core.y_ready = 1; core.p_data_rdata = 0;
        core.irq_a = 0; core.irq_b = 0; core.irq_priority = 0;
        core.irq_ssi_tx = 0; core.irq_ssi_rx = 0;
        core.irq_ssi_tx_exception = 0; core.irq_ssi_rx_exception = 0;
        core.irq_sci_tx = 0; core.irq_sci_rx = 0;
        core.irq_sci_rx_exception = 0; core.irq_sci_idle = 0;
        core.irq_sci_timer = 0; core.dbg_reg_wr_en = 0;
        core.dbg_reg_wr_addr = 0; core.dbg_reg_wr_data = 0;
        for (unsigned i = 0; i < 4; ++i) tick();
        core.rst_n = 1;
    }
    void tick() {
        core.clk = 0; core.eval();
        const bool fe = core.p_fetch_en; const uint16_t fa = core.p_fetch_addr;
        xr += core.x_rd_en; xw += core.x_wr_en;
        yr += core.y_rd_en; yw += core.y_wr_en;
        core.clk = 1; core.eval();
        if (fe) core.p_fetch_data = p[fa & 15] & 0xffffff;
        ++cycles;
    }
    void debug_write(uint8_t reg, uint64_t value) {
        core.dbg_reg_wr_addr = reg; core.dbg_reg_wr_data = value;
        core.dbg_reg_wr_en = 1; tick(); core.dbg_reg_wr_en = 0;
    }
    bool retire(uint32_t opcode) {
        for (unsigned i = 0; i < 100; ++i) {
            if (core.state_out == kWriteback && core.cur_inst_out == opcode) {
                tick(); return true;
            }
            tick();
        }
        return false;
    }
};

struct Case { const char *name; uint8_t control; uint32_t input, expected;
              uint8_t initial_sp, expected_sp; };

struct SelectorCase {
    const char *name;
    uint8_t code;
    uint32_t input;
    uint32_t expected_source;
};

uint32_t move_long_immediate(uint8_t destination) {
    return ((0x40u | ((destination >> 3) << 4) | (destination & 7)) << 16) |
           0xf400u;
}

uint32_t movec_long_immediate(uint8_t destination) {
    return 0x05f400u | destination;
}

uint32_t movec_register(uint8_t source, uint8_t destination) {
    return 0x04c080u | (uint32_t(source) << 8) | destination;
}

bool run_case(const Case &test) {
    // W=1: high register field is source, low register field destination.
    const uint32_t write = 0x04c480u | test.control;       // MOVEC X0,C
    const uint32_t read = 0x04c085u | (test.control << 8); // MOVEC C,X1
    Harness h;
    unsigned pc = 0;
    if (test.initial_sp != 0)
        h.p[pc++] = 0x0500bbu | (uint32_t(test.initial_sp) << 8);
    h.p[pc++] = write;
    h.p[pc++] = read;
    h.p[pc++] = 0x0af080;
    h.p[pc++] = pc - 2;
    h.debug_write(0x04, test.input);
    h.xr = h.xw = h.yr = h.yw = 0;
    h.core.run = 1;
    bool setup_ok = true;
    if (test.initial_sp != 0)
        setup_ok = h.retire(0x0500bbu | (uint32_t(test.initial_sp) << 8));
    const bool wrote = h.retire(write);
    const bool read_back = h.retire(read);
    const bool bus_quiet = h.xr == 0 && h.xw == 0 && h.yr == 0 && h.yw == 0;
    const uint32_t actual_x1 = h.core.rootp->dsp56001_core__DOT__x1_reg;
    const uint8_t actual_sp = h.core.rootp->dsp56001_core__DOT__sp;
    const bool pass = setup_ok && wrote && read_back &&
                      actual_x1 == test.expected &&
                      actual_sp == test.expected_sp && bus_quiet;
    std::printf("%s MOVEC_REGISTER %s C=%02x X1=%06x/%06x "
                "SP=%02x/%02x retired=%u/%u/%u bus=X%u/%u,Y%u/%u\n",
                pass ? "PASS" : "FAIL", test.name, test.control,
                actual_x1, test.expected, actual_sp,
                test.expected_sp, setup_ok, wrote, read_back,
                h.xr, h.xw, h.yr, h.yw);
    return pass;
}

bool run_source_selector(const SelectorCase &test) {
    // DSP56001UMA2 A-157: S2,D1 admits every assigned S2 code.  Initialize
    // S2 architecturally, transfer it to M0, then return M0 through X1.
    const bool control = test.code >= 0x20;
    const uint32_t setup = control ? movec_long_immediate(test.code) :
                                     move_long_immediate(test.code);
    const uint32_t transfer = movec_register(test.code, 0x20); // S2,M0
    const uint32_t readback = movec_register(0x20, 0x05);      // M0,X1
    Harness h;
    h.p[0] = setup;
    h.p[1] = test.input;
    h.p[2] = 0x000000;
    h.p[3] = transfer;
    h.p[4] = 0x000000;
    h.p[5] = readback;
    h.p[6] = 0x0c0006;
    h.core.run = 1;
    const bool setup_ok = h.retire(setup);
    const bool transferred = h.retire(transfer);
    const bool read_back = h.retire(readback);
    const uint32_t actual = h.core.rootp->dsp56001_core__DOT__x1_reg;
    const uint8_t expected_sp = test.code == 0x3b ? 3 : 0;
    const bool pass = setup_ok && transferred && read_back &&
                      actual == test.expected_source &&
                      h.core.rootp->dsp56001_core__DOT__sp == expected_sp &&
                      h.xr == 0 && h.xw == 0 && h.yr == 0 && h.yw == 0;
    std::printf("%s MOVEC S2,D1 %-3s code=%02x value=%06x/%06x "
                "SP=%02x/%02x retired=%u/%u/%u bus=X%u/%u,Y%u/%u\n",
                pass ? "PASS" : "FAIL", test.name, test.code, actual,
                test.expected_source, h.core.rootp->dsp56001_core__DOT__sp,
                expected_sp, setup_ok, transferred, read_back,
                h.xr, h.xw, h.yr, h.yw);
    return pass;
}

bool run_destination_selector(const SelectorCase &test) {
    // The reciprocal S1,D2 form drives every assigned D2 from M0, then
    // returns D2 through M1 and X1.  Special destinations retain their
    // documented width and SSH stack lifecycle.
    const uint32_t input = test.code == 0x39 ? 0x000035 :
                           test.code == 0x3b ? 0x000003 : 0x00cdef;
    const uint32_t expected = (test.code == 0x0a || test.code == 0x0b ||
                               test.code == 0x3a) ? input & 0xff : input;
    const uint32_t setup = movec_long_immediate(0x20);         // #>input,M0
    const uint32_t transfer = movec_register(0x20, test.code); // M0,D2
    const uint32_t return_to_control =
        movec_register(test.code, 0x21);                        // D2,M1
    const uint32_t readback = movec_register(0x21, 0x05);      // M1,X1
    Harness h;
    h.p[0] = setup;
    h.p[1] = input;
    h.p[2] = 0x000000;
    h.p[3] = transfer;
    h.p[4] = 0x000000;
    h.p[5] = return_to_control;
    h.p[6] = 0x000000;
    h.p[7] = readback;
    h.p[8] = 0x0c0008;
    h.core.run = 1;
    const bool setup_ok = h.retire(setup);
    const bool transferred = h.retire(transfer);
    const bool returned = h.retire(return_to_control);
    const bool read_back = h.retire(readback);
    const uint32_t actual = h.core.rootp->dsp56001_core__DOT__x1_reg;
    const uint8_t expected_sp = test.code == 0x3b ? 3 : 0;
    const bool pass = setup_ok && transferred && returned && read_back &&
                      actual == expected &&
                      h.core.rootp->dsp56001_core__DOT__sp == expected_sp &&
                      h.xr == 0 && h.xw == 0 && h.yr == 0 && h.yw == 0;
    std::printf("%s MOVEC S1,D2 %-3s code=%02x value=%06x/%06x "
                "SP=%02x/%02x retired=%u/%u/%u/%u bus=X%u/%u,Y%u/%u\n",
                pass ? "PASS" : "FAIL", test.name, test.code, actual,
                expected, h.core.rootp->dsp56001_core__DOT__sp,
                expected_sp, setup_ok, transferred, returned, read_back,
                h.xr, h.xw, h.yr, h.yw);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const Case cases[] = {
        {"M0",0x20,0x12abcd,0x00abcd,0,0}, {"M1",0x21,0x12abc1,0x00abc1,0,0},
        {"M2",0x22,0x12abc2,0x00abc2,0,0}, {"M3",0x23,0x12abc3,0x00abc3,0,0},
        {"M4",0x24,0x12abc4,0x00abc4,0,0}, {"M5",0x25,0x12abc5,0x00abc5,0,0},
        {"M6",0x26,0x12abc6,0x00abc6,0,0}, {"M7",0x27,0x12abc7,0x00abc7,0,0},
        {"SR",0x39,0x000035,0x000035,0,0}, {"OMR",0x3a,0x12ab55,0x000055,0,0},
        {"SP",0x3b,0x12ab03,0x000003,0,3}, {"SSH",0x3c,0x12cdef,0x00cdef,0,0},
        {"SSL",0x3d,0x12cdef,0x00cdef,1,1}, {"LA",0x3e,0x124567,0x004567,0,0},
        // DSP56001UM07 7.2.7 and DSP56001UMA2 A-157: LC is 16 bits;
        // moving a 24-bit source into it discards the upper eight bits.
        {"LC",0x3f,0xabcdef,0x00cdef,0,0},
    };
    unsigned failures = 0;
    for (const auto &test : cases) failures += !run_case(test);
    std::vector<SelectorCase> selectors;
    const char *data_names[] = {
        "X0", "X1", "Y0", "Y1", "A0", "B0", "A2", "B2",
        "A1", "B1", "A", "B"
    };
    for (unsigned i = 0; i < 12; ++i) {
        const uint8_t code = 0x04 + i;
        const uint32_t input = (code == 0x0a || code == 0x0b) ?
                               0xabcdef : 0x654321;
        const uint32_t expected = (code == 0x0a || code == 0x0b) ?
                                  0x0000ef : input & 0xffff;
        selectors.push_back({data_names[i], code, input, expected});
    }
    for (unsigned bank = 0; bank < 2; ++bank)
        for (unsigned i = 0; i < 8; ++i) {
            static const char *prefix[] = {"R", "N"};
            static char names[2][8][4];
            std::snprintf(names[bank][i], sizeof(names[bank][i]), "%s%u",
                          prefix[bank], i);
            selectors.push_back({names[bank][i], uint8_t(0x10 + bank * 8 + i),
                                 0xabcdef, 0x00cdef});
        }
    for (unsigned i = 0; i < 8; ++i) {
        static char names[8][4];
        std::snprintf(names[i], sizeof(names[i]), "M%u", i);
        selectors.push_back({names[i], uint8_t(0x20 + i),
                             0xabcdef, 0x00cdef});
    }
    const SelectorCase controls[] = {
        {"SR",  0x39, 0x000035, 0x000035},
        {"OMR", 0x3a, 0x12ab55, 0x000055},
        {"SP",  0x3b, 0x000003, 0x000003},
        {"SSH", 0x3c, 0xabcdef, 0x00cdef},
        {"SSL", 0x3d, 0xabcdef, 0x00cdef},
        {"LA",  0x3e, 0xabcdef, 0x00cdef},
        {"LC",  0x3f, 0xabcdef, 0x00cdef},
    };
    for (const auto &test : controls) selectors.push_back(test);
    unsigned selector_cases = 0;
    for (const auto &test : selectors) {
        failures += !run_source_selector(test);
        failures += !run_destination_selector(test);
        selector_cases += 2;
    }
    std::printf("MOVEC REGISTER SUMMARY cases=%zu failures=%u "
                "control-matrix=%zu selector-directions=%u\n",
                sizeof(cases)/sizeof(cases[0]) + selector_cases, failures,
                sizeof(cases)/sizeof(cases[0]), selector_cases);
    return failures == 0 ? 0 : 1;
}
