// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 Class-I R:Y parallel data-move fail-before oracle
//
// DSP56001UMA2 A-140--A-143 defines one accumulator-to-X0/X1 transfer in
// parallel with a Y-memory transfer.  These exact direct-(R0) encodings use a
// NOP ALU byte so the two data moves are isolated:
//
//     MOVE A,X0 Y0,Y:(R0)      $106000
//     MOVE A,X0 Y:(R0),Y0      $10e000
//
// The sibling X:R family uses bit 14 clear.  R:Y sets bit 14, and must touch
// only the Y bus while preserving the independent register transfer.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint8_t kStateWritebackFetch = 4;
constexpr uint32_t kSetupR0 = 0x60f400; // MOVE #>$1234,R0
constexpr uint32_t kSetupN0 = 0x380300; // MOVE #$03,N0
constexpr uint32_t kClass1Store = 0x106000;
constexpr uint32_t kClass1Load = 0x10e000;
constexpr uint32_t kReadX0ToB = 0x208f00;
constexpr uint32_t kReadY0ToB = 0x20cf00;

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 16> p{};
    unsigned x_reads = 0;
    unsigned x_writes = 0;
    unsigned y_reads = 0;
    unsigned y_writes = 0;
    unsigned p_reads = 0;
    unsigned p_writes = 0;
    uint16_t last_y_read_addr = 0;
    uint16_t last_y_write_addr = 0;
    uint32_t last_y_write_data = 0;
    std::array<uint16_t, 4> y_read_addr{};
    std::array<uint16_t, 4> y_write_addr{};
    std::array<uint32_t, 4> y_write_data{};

    Harness() {
        p[0] = kSetupR0;
        p[1] = 0x001234;
        p[2] = 0x000000; // Settle explicit R0 write before AGU use.
        p[3] = kClass1Store;
        p[4] = kReadX0ToB;
        p[5] = 0x0af080; // JMP >$0005
        p[6] = 0x000005;

        core.clk = 0;
        core.rst_n = 0;
        core.clock_enable = 1;
        core.halt = 0;
        core.run = 0;
        core.p_fetch_data = 0;
        core.p_fetch_ready = 1;
        core.x_rdata = 0;
        core.x_ready = 1;
        core.y_rdata = 0xabcdef;
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
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        if (core.x_rd_en) ++x_reads;
        if (core.x_wr_en) ++x_writes;
        if (core.y_rd_en) {
            if (y_reads < y_read_addr.size())
                y_read_addr[y_reads] = core.y_addr;
            ++y_reads;
            last_y_read_addr = core.y_addr;
        }
        if (core.y_wr_en) {
            if (y_writes < y_write_addr.size()) {
                y_write_addr[y_writes] = core.y_addr;
                y_write_data[y_writes] = core.y_wdata & 0x00ffffff;
            }
            ++y_writes;
            last_y_write_addr = core.y_addr;
            last_y_write_data = core.y_wdata & 0x00ffffff;
        }
        if (core.p_data_rd_en) ++p_reads;
        if (core.p_data_wr_en) ++p_writes;
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
};

constexpr uint64_t kA = 0x00123456000000ULL;

bool direct_store() {
    Harness h;
    h.debug_write(0x04, 0x111111); // X0 sentinel
    h.debug_write(0x06, 0x654321); // Y0 primary source
    h.debug_write(0x0e, kA);       // independent A -> X0 source
    h.core.run = 1;

    const bool setup = h.retire(kSetupR0);
    const bool moved = h.retire(kClass1Store);
    const bool readback = h.retire(kReadX0ToB);
    const uint32_t x0 = (h.core.acc_b_out >> 24) & 0x00ffffff;
    const bool pass = setup && moved && readback &&
        h.x_reads == 0 && h.x_writes == 0 &&
        h.y_reads == 0 && h.y_writes == 1 &&
        h.last_y_write_addr == 0x1234 &&
        h.last_y_write_data == 0x654321 &&
        h.p_reads == 0 && h.p_writes == 0 && x0 == 0x123456;
    std::printf("%s R:Y Class-I store 106000: completed=%u/%u/%u "
                "X=%uR/%uW Y=%uR/%uW@%04x=%06x X0=%06x/123456\n",
                pass ? "PASS" : "FAIL", setup, moved, readback,
                h.x_reads, h.x_writes, h.y_reads, h.y_writes,
                h.last_y_write_addr, h.last_y_write_data, x0);
    return pass;
}

bool direct_load() {
    Harness h;
    h.p[3] = kClass1Load;
    h.p[4] = kReadX0ToB;
    h.p[5] = kReadY0ToB;
    h.p[6] = 0x0af080; // JMP >$0006
    h.p[7] = 0x000006;
    h.debug_write(0x04, 0x111111); // X0 sentinel
    h.debug_write(0x06, 0x654321); // Y0 sentinel
    h.debug_write(0x0e, kA);       // independent A -> X0 source
    h.core.run = 1;

    const bool setup = h.retire(kSetupR0);
    const bool moved = h.retire(kClass1Load);
    const bool xread = h.retire(kReadX0ToB);
    const uint32_t x0 = (h.core.acc_b_out >> 24) & 0x00ffffff;
    const bool yread = h.retire(kReadY0ToB);
    const uint32_t y0 = (h.core.acc_b_out >> 24) & 0x00ffffff;
    const bool pass = setup && moved && xread && yread &&
        h.x_reads == 0 && h.x_writes == 0 &&
        h.y_reads == 1 && h.y_writes == 0 &&
        h.last_y_read_addr == 0x1234 &&
        h.p_reads == 0 && h.p_writes == 0 &&
        x0 == 0x123456 && y0 == 0xabcdef;
    std::printf("%s R:Y Class-I load 10e000: completed=%u/%u/%u/%u "
                "X=%uR/%uW Y=%uR@%04x/%uW X0=%06x/123456 "
                "Y0=%06x/abcdef\n",
                pass ? "PASS" : "FAIL", setup, moved, xread, yread,
                h.x_reads, h.x_writes, h.y_reads, h.last_y_read_addr,
                h.y_writes, x0, y0);
    return pass;
}

struct AddressCase {
    uint32_t store_opcode;
    uint32_t load_opcode;
    bool setup_n;
    uint16_t access_addr;
    uint16_t final_r0;
    const char *name;
};

bool address_modes() {
    constexpr std::array<AddressCase, 7> cases{{
        {0x104000, 0x10c000, true,  0x1234, 0x1231, "postdec-n"},
        {0x104800, 0x10c800, true,  0x1234, 0x1237, "postinc-n"},
        {0x105000, 0x10d000, false, 0x1234, 0x1233, "postdec"},
        {0x105800, 0x10d800, false, 0x1234, 0x1235, "postinc"},
        {0x106000, 0x10e000, false, 0x1234, 0x1234, "direct"},
        {0x106800, 0x10e800, true,  0x1237, 0x1234, "indexed"},
        {0x107800, 0x10f800, false, 0x1233, 0x1233, "predec"},
    }};
    bool all = true;

    for (const auto &c : cases) {
        Harness h;
        h.p.fill(0);
        unsigned i = 0;
        h.p[i++] = kSetupR0;
        h.p[i++] = 0x001234;
        h.p[i++] = 0x000000; // Settle explicit R0 write before AGU use.
        if (c.setup_n) {
            h.p[i++] = kSetupN0;
            // Address-register writes have a one-instruction pipeline delay.
            h.p[i++] = 0x000000;
        }
        h.p[i++] = c.store_opcode;
        h.p[i++] = kClass1Store;
        h.p[i++] = kReadX0ToB;
        const unsigned loop = i;
        h.p[i++] = 0x0af080;
        h.p[i++] = loop;
        h.debug_write(0x04, 0x111111);
        h.debug_write(0x06, 0x654321);
        h.debug_write(0x0e, kA);
        h.core.run = 1;
        const bool setup_r = h.retire(kSetupR0);
        const bool setup_n = !c.setup_n || h.retire(kSetupN0);
        const bool moved = h.retire(c.store_opcode);
        const bool control = h.retire(kClass1Store);
        const bool readback = h.retire(kReadX0ToB);
        const uint32_t x0 = (h.core.acc_b_out >> 24) & 0x00ffffff;
        const bool pass = setup_r && setup_n && moved && control && readback &&
            h.x_reads == 0 && h.x_writes == 0 && h.y_reads == 0 &&
            h.y_writes == 2 && h.y_write_addr[0] == c.access_addr &&
            h.y_write_addr[1] == c.final_r0 &&
            h.y_write_data[0] == 0x654321 &&
            h.y_write_data[1] == 0x654321 && x0 == 0x123456;
        std::printf("%s R:Y Class-I store %-9s %06x: "
                    "Y=%uW[%04x,%04x] X=%uR/%uW X0=%06x\n",
                    pass ? "PASS" : "FAIL", c.name, c.store_opcode,
                    h.y_writes, h.y_write_addr[0], h.y_write_addr[1],
                    h.x_reads, h.x_writes, x0);
        all &= pass;
    }

    for (const auto &c : cases) {
        Harness h;
        h.p.fill(0);
        unsigned i = 0;
        h.p[i++] = kSetupR0;
        h.p[i++] = 0x001234;
        h.p[i++] = 0x000000; // Settle explicit R0 write before AGU use.
        if (c.setup_n) {
            h.p[i++] = kSetupN0;
            // Address-register writes have a one-instruction pipeline delay.
            h.p[i++] = 0x000000;
        }
        h.p[i++] = c.load_opcode;
        h.p[i++] = kClass1Store;
        h.p[i++] = kReadX0ToB;
        const unsigned loop = i;
        h.p[i++] = 0x0af080;
        h.p[i++] = loop;
        h.debug_write(0x04, 0x111111);
        h.debug_write(0x06, 0x654321);
        h.debug_write(0x0e, kA);
        h.core.run = 1;
        const bool setup_r = h.retire(kSetupR0);
        const bool setup_n = !c.setup_n || h.retire(kSetupN0);
        const bool moved = h.retire(c.load_opcode);
        const bool control = h.retire(kClass1Store);
        const bool readback = h.retire(kReadX0ToB);
        const uint32_t x0 = (h.core.acc_b_out >> 24) & 0x00ffffff;
        const bool pass = setup_r && setup_n && moved && control && readback &&
            h.x_reads == 0 && h.x_writes == 0 && h.y_reads == 1 &&
            h.y_read_addr[0] == c.access_addr && h.y_writes == 1 &&
            h.y_write_addr[0] == c.final_r0 &&
            h.y_write_data[0] == 0xabcdef && x0 == 0x123456;
        std::printf("%s R:Y Class-I load  %-9s %06x: "
                    "Y=%uR@%04x/%uW@%04x=%06x X=%uR/%uW X0=%06x\n",
                    pass ? "PASS" : "FAIL", c.name, c.load_opcode,
                    h.y_reads, h.y_read_addr[0], h.y_writes,
                    h.y_write_addr[0], h.y_write_data[0],
                    h.x_reads, h.x_writes, x0);
        all &= pass;
    }
    return all;
}

bool extension_forms() {
    bool all = true;
    struct ExtensionCase {
        uint32_t opcode;
        uint32_t extension;
        bool load;
        bool immediate;
        const char *name;
    };
    constexpr std::array<ExtensionCase, 3> cases{{
        {0x107000, 0x005678, false, false, "absolute-store"},
        {0x10f000, 0x005678, true,  false, "absolute-load"},
        {0x10f400, 0x654321, true,  true,  "immediate-load"},
    }};
    for (const auto &c : cases) {
        Harness h;
        h.p.fill(0);
        h.p[0] = kSetupR0;
        h.p[1] = 0x001234;
        h.p[2] = 0x000000; // Settle explicit R0 write before AGU use.
        h.p[3] = c.opcode;
        h.p[4] = c.extension;
        h.p[5] = kClass1Store;
        h.p[6] = kReadX0ToB;
        h.p[7] = 0x0af080;
        h.p[8] = 0x000007;
        h.debug_write(0x04, 0x111111);
        h.debug_write(0x06, 0x654321);
        h.debug_write(0x0e, kA);
        h.core.run = 1;
        const bool setup = h.retire(kSetupR0);
        const bool moved = h.retire(c.opcode);
        const bool control = h.retire(kClass1Store);
        const bool readback = h.retire(kReadX0ToB);
        const uint32_t x0 = (h.core.acc_b_out >> 24) & 0x00ffffff;
        const unsigned expected_reads = c.load && !c.immediate ? 1 : 0;
        const unsigned expected_writes = c.load ? 1 : 2;
        const uint16_t first_write_addr = c.load ? 0x1234 : 0x5678;
        const uint32_t first_write_data = c.load ?
            (c.immediate ? c.extension : 0xabcdef) : 0x654321;
        const bool pass = setup && moved && control && readback &&
            h.x_reads == 0 && h.x_writes == 0 &&
            h.y_reads == expected_reads &&
            (!expected_reads || h.y_read_addr[0] == 0x5678) &&
            h.y_writes == expected_writes &&
            h.y_write_addr[0] == first_write_addr &&
            h.y_write_data[0] == first_write_data && x0 == 0x123456;
        std::printf("%s R:Y Class-I %-14s %06x/%06x: "
                    "Y=%uR@%04x/%uW@%04x=%06x X=%uR/%uW X0=%06x\n",
                    pass ? "PASS" : "FAIL", c.name, c.opcode, c.extension,
                    h.y_reads, h.y_read_addr[0], h.y_writes,
                    h.y_write_addr[0], h.y_write_data[0],
                    h.x_reads, h.x_writes, x0);
        all &= pass;
    }
    return all;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const bool store = direct_store();
    const bool load = direct_load();
    const bool modes = address_modes();
    const bool extensions = extension_forms();
    return store && load && modes && extensions ? 0 : 1;
}
