// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 on-chip X/Y data-ROM oracle
//
// Appendix C/D of Motorola DSP56001/D gives the exact X:$0100-$01ff
// positive Mu-law/A-law expansion tables and Y:$0100-$01ff four-quadrant
// sine table. Exercise the synthesizable memory block through its public
// registered read ports; zero-filled or approximate tables must fail.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_mem.h"
#include "verilated.h"

namespace {

constexpr std::array<uint32_t, 65> kSineQuarter = {
    0x000000, 0x03242b, 0x0647d9, 0x096a90, 0x0c8bd3, 0x0fab27,
    0x12c810, 0x15e214, 0x18f8b8, 0x1c0b82, 0x1f19f9, 0x2223a5,
    0x25280c, 0x2826b9, 0x2b1f35, 0x2e110a, 0x30fbc5, 0x33def3,
    0x36ba20, 0x398cdd, 0x3c56ba, 0x3f174a, 0x41ce1e, 0x447acd,
    0x471ced, 0x49b415, 0x4c3fe0, 0x4ebfe9, 0x5133cd, 0x539b2b,
    0x55f5a5, 0x5842dd, 0x5a827a, 0x5cb421, 0x5ed77d, 0x60ec38,
    0x62f202, 0x64e889, 0x66cf81, 0x68a69f, 0x6a6d99, 0x6c2429,
    0x6dca0d, 0x6f5f03, 0x70e2cc, 0x72552d, 0x73b5ec, 0x7504d3,
    0x7641af, 0x776c4f, 0x788484, 0x798a24, 0x7a7d05, 0x7b5d04,
    0x7c29fc, 0x7ce3cf, 0x7d8a5f, 0x7e1d94, 0x7e9d56, 0x7f0992,
    0x7f6237, 0x7fa737, 0x7fd888, 0x7ff622, 0x7fffff,
};

uint32_t expected_mulaw(uint8_t code) {
    const uint8_t decoded = static_cast<uint8_t>(~code) & 0x7f;
    const uint32_t segment = (decoded >> 4) & 7;
    const uint32_t quantum = decoded & 0x0f;
    const uint32_t linear = (((quantum << 3) + 132) << segment) - 132;
    return (linear << 8) & 0x00ffffff;
}

uint32_t expected_alaw(uint8_t code) {
    const uint8_t decoded = code ^ 0x55;
    const uint32_t segment = (decoded >> 4) & 7;
    const uint32_t quantum = decoded & 0x0f;
    uint32_t linear = quantum << 4;
    if (segment == 0) {
        linear += 8;
    } else {
        linear += 0x108;
        if (segment > 1)
            linear <<= segment - 1;
    }
    return (linear << 8) & 0x00ffffff;
}

uint32_t expected_sine(uint8_t index) {
    if (index <= 0x40)
        return kSineQuarter[index];
    if (index < 0x80)
        return kSineQuarter[0x80 - index];
    if (index == 0x80)
        return 0;
    if (index < 0xc0)
        return (-kSineQuarter[index - 0x80]) & 0x00ffffff;
    if (index == 0xc0)
        return 0x800000;
    return (-kSineQuarter[0x100 - index]) & 0x00ffffff;
}

class MemHarness {
public:
    Vdsp56001_mem top;

    MemHarness() {
        top.clk = 0;
        top.rst_n = 0;
        top.data_rom_enable = 1;
        top.p_addr = 0;
        top.p_rd_en = 0;
        top.p_wr_en = 0;
        top.p_wdata = 0;
        top.x_addr = 0;
        top.x_rd_en = 0;
        top.x_wr_en = 0;
        top.x_wdata = 0;
        top.y_addr = 0;
        top.y_rd_en = 0;
        top.y_wr_en = 0;
        top.y_wdata = 0;
        top.ext_rdata = 0;
        top.ext_ready = 1;
        top.xperiph_rdata = 0;
        top.yperiph_rdata = 0;
        top.yrtc_rdata = 0;
        top.dbg_wr_en = 0;
        top.dbg_space = 0;
        top.dbg_addr = 0;
        top.dbg_wdata = 0;
        tick();
        top.rst_n = 1;
        tick();
    }

    void tick() {
        top.clk = 0;
        top.eval();
        top.clk = 1;
        top.eval();
        top.clk = 0;
        top.eval();
    }

    uint32_t read_x(uint16_t addr) {
        top.x_addr = addr;
        top.x_rd_en = 1;
        tick();
        top.x_rd_en = 0;
        top.eval();
        return top.x_rdata & 0x00ffffff;
    }

    uint32_t read_y(uint16_t addr) {
        top.y_addr = addr;
        top.y_rd_en = 1;
        tick();
        top.y_rd_en = 0;
        top.eval();
        return top.y_rdata & 0x00ffffff;
    }
};

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    MemHarness h;
    unsigned failures = 0;

    for (unsigned i = 0; i < 256; ++i) {
        const uint32_t expected = i < 128 ? expected_mulaw(i) : expected_alaw(i);
        const uint32_t actual = h.read_x(0x0100 + i);
        if (actual != expected) {
            if (failures < 12)
                std::printf("FAIL X:%04x actual=%06x expected=%06x\n",
                            0x0100 + i, actual, expected);
            ++failures;
        }
    }

    for (unsigned i = 0; i < 256; ++i) {
        const uint32_t expected = expected_sine(i);
        const uint32_t actual = h.read_y(0x0100 + i);
        if (actual != expected) {
            if (failures < 12)
                std::printf("FAIL Y:%04x actual=%06x expected=%06x\n",
                            0x0100 + i, actual, expected);
            ++failures;
        }
    }

    // OMR.DE resets clear on the real DSP56001. With DE clear, the same
    // address window is ordinary external X/Y memory, not a hidden ROM read.
    h.top.data_rom_enable = 0;
    h.top.ext_rdata = 0xabc123;
    h.top.x_addr = 0x0100;
    h.top.x_rd_en = 1;
    h.top.eval();
    if (!h.top.ext_rd_en || h.top.ext_space != 1 || h.top.ext_addr != 0x4100) {
        std::printf("FAIL DE=0 X external route rd=%u space=%u addr=%04x\n",
                    h.top.ext_rd_en, h.top.ext_space, h.top.ext_addr);
        ++failures;
    }
    h.tick();
    if ((h.top.x_rdata & 0x00ffffff) != 0xabc123) {
        std::printf("FAIL DE=0 X external data actual=%06x expected=abc123\n",
                    h.top.x_rdata & 0x00ffffff);
        ++failures;
    }
    h.top.x_rd_en = 0;

    h.top.ext_rdata = 0x654321;
    h.top.y_addr = 0x0100;
    h.top.y_rd_en = 1;
    h.top.eval();
    if (!h.top.ext_rd_en || h.top.ext_space != 2 || h.top.ext_addr != 0x0100) {
        std::printf("FAIL DE=0 Y external route rd=%u space=%u addr=%04x\n",
                    h.top.ext_rd_en, h.top.ext_space, h.top.ext_addr);
        ++failures;
    }
    h.tick();
    if ((h.top.y_rdata & 0x00ffffff) != 0x654321) {
        std::printf("FAIL DE=0 Y external data actual=%06x expected=654321\n",
                    h.top.y_rdata & 0x00ffffff);
        ++failures;
    }
    h.top.y_rd_en = 0;

    h.top.data_rom_enable = 1;
    h.top.x_addr = 0x0100;
    h.top.x_rd_en = 1;
    h.top.eval();
    if (h.top.ext_rd_en) {
        std::printf("FAIL DE=1 X ROM leaked an external request\n");
        ++failures;
    }
    h.top.x_rd_en = 0;
    h.top.y_addr = 0x0100;
    h.top.y_rd_en = 1;
    h.top.eval();
    if (h.top.ext_rd_en) {
        std::printf("FAIL DE=1 Y ROM leaked an external request\n");
        ++failures;
    }
    h.top.y_rd_en = 0;

    if (failures) {
        std::printf("FAIL DSP56001 data ROM tables mismatches=%u/512\n", failures);
        return 1;
    }
    std::printf("PASS exact DSP56001 X/Y data ROM tables entries=512\n");
    return 0;
}
