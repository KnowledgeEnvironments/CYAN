// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 Class-II X:R and R:Y parallel data-move oracle
//
// DSP56001UMA2 A-135 and A-144 define one accumulator-to-memory transfer in
// parallel with the corresponding input-register-to-accumulator transfer:
//
//     MOVE A,X:(R0) X0,A      $082000
//     MOVE B,X:(R0) X0,B      $092000
//     MOVE A,Y:(R0) Y0,A      $08a000
//     MOVE B,Y:(R0) Y0,B      $09a000
//
// The low byte is an independent ALU opcode; zero isolates the parallel move.
// MOVEP shares the 0x08/0x09 prefix but has bit 14 set, so an exact legal
// immediate-to-peripheral MOVEP is retained as an overlap control.
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
constexpr uint32_t kClass2AX = 0x082000;
constexpr uint32_t kClass2BX = 0x092000;
constexpr uint32_t kClass2AY = 0x08a000;
constexpr uint32_t kClass2BY = 0x09a000;
constexpr uint32_t kMovepImmediateX = 0x08f4be;

constexpr uint32_t kInitialSr = 0x000035;
constexpr uint32_t kX0 = 0x314159;
constexpr uint32_t kY0 = 0xa71828;
constexpr uint64_t kA = 0x0112345689abcdULL;
constexpr uint64_t kB = 0xfe12345689abcdULL;
constexpr uint64_t kX0AsAcc = 0x00314159000000ULL;
constexpr uint64_t kY0AsAcc = 0xffa71828000000ULL;

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
    uint16_t last_x_read_addr = 0;
    uint16_t last_x_write_addr = 0;
    uint32_t last_x_write_data = 0;
    uint16_t last_y_read_addr = 0;
    uint16_t last_y_write_addr = 0;
    uint32_t last_y_write_data = 0;
    std::array<uint16_t, 4> x_write_addr{};
    std::array<uint32_t, 4> x_write_data{};
    std::array<uint16_t, 4> y_write_addr{};
    std::array<uint32_t, 4> y_write_data{};

    Harness() {
        p[0] = kSetupR0;
        p[1] = 0x001234;
        p[2] = 0x000000; // Address-register write pipeline gap.
        p[3] = kClass2AX;
        p[4] = 0x0af080; // JMP >$0004
        p[5] = 0x000004;

        core.clk = 0;
        core.rst_n = 0;
        core.clock_enable = 1;
        core.halt = 0;
        core.run = 0;
        core.p_fetch_data = 0;
        core.p_fetch_ready = 1;
        core.x_rdata = 0x55aa11;
        core.x_ready = 1;
        core.y_rdata = 0xaa55ee;
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
        if (core.x_rd_en) {
            ++x_reads;
            last_x_read_addr = core.x_addr;
        }
        if (core.x_wr_en) {
            if (x_writes < x_write_addr.size()) {
                x_write_addr[x_writes] = core.x_addr;
                x_write_data[x_writes] = core.x_wdata & 0x00ffffff;
            }
            ++x_writes;
            last_x_write_addr = core.x_addr;
            last_x_write_data = core.x_wdata & 0x00ffffff;
        }
        if (core.y_rd_en) {
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

bool test_class2(uint32_t opcode, bool y_space, bool select_b) {
    Harness h;
    h.p[3] = opcode;
    h.debug_write(0x39, kInitialSr);
    h.debug_write(0x04, kX0);
    h.debug_write(0x06, kY0);
    h.debug_write(0x0e, kA);
    h.debug_write(0x0f, kB);
    h.core.run = 1;

    const bool setup = h.retire(kSetupR0);
    const bool moved = h.retire(opcode);
    const uint64_t expected_a = !select_b ?
        (y_space ? kY0AsAcc : kX0AsAcc) : kA;
    const uint64_t expected_b = select_b ?
        (y_space ? kY0AsAcc : kX0AsAcc) : kB;
    const uint32_t expected_store = select_b ? 0x800000 : 0x7fffff;
    const bool x_contract = y_space ?
        (h.x_reads == 0 && h.x_writes == 0) :
        (h.x_reads == 0 && h.x_writes == 1 &&
         h.last_x_write_addr == 0x1234 &&
         h.last_x_write_data == expected_store);
    const bool y_contract = y_space ?
        (h.y_reads == 0 && h.y_writes == 1 &&
         h.last_y_write_addr == 0x1234 &&
         h.last_y_write_data == expected_store) :
        (h.y_reads == 0 && h.y_writes == 0);
    const bool pass = setup && moved && x_contract && y_contract &&
        h.p_reads == 0 && h.p_writes == 0 &&
        h.core.acc_a_out == expected_a &&
        h.core.acc_b_out == expected_b &&
        h.core.sr_out == (kInitialSr | 0x40);

    std::printf("%s Class-II %06x %s:%c completed=%u/%u "
                "X=%uR@%04x/%uW@%04x=%06x "
                "Y=%uR@%04x/%uW@%04x=%06x P=%uR/%uW "
                "A=%014llx/%014llx B=%014llx/%014llx SR=%06x/000075\n",
                pass ? "PASS" : "FAIL", opcode, y_space ? "R:Y" : "X:R",
                select_b ? 'B' : 'A', setup, moved,
                h.x_reads, h.last_x_read_addr, h.x_writes,
                h.last_x_write_addr, h.last_x_write_data,
                h.y_reads, h.last_y_read_addr, h.y_writes,
                h.last_y_write_addr, h.last_y_write_data,
                h.p_reads, h.p_writes,
                static_cast<unsigned long long>(h.core.acc_a_out),
                static_cast<unsigned long long>(expected_a),
                static_cast<unsigned long long>(h.core.acc_b_out),
                static_cast<unsigned long long>(expected_b), h.core.sr_out);
    return pass;
}

struct AddressModeCase {
    const char *name;
    uint8_t mmm;
    bool setup_n;
    uint16_t effective;
    uint16_t final_r;
};

constexpr std::array<AddressModeCase, 7> kAddressModes{{
    {"postdec-n", 0, true,  0x1234, 0x1231},
    {"postinc-n", 1, true,  0x1234, 0x1237},
    {"postdec",   2, false, 0x1234, 0x1233},
    {"postinc",   3, false, 0x1234, 0x1235},
    {"direct",    4, false, 0x1234, 0x1234},
    {"indexed",   5, true,  0x1237, 0x1234},
    {"predec",    7, false, 0x1233, 0x1233},
}};

bool test_class2_mode(uint32_t direct_opcode, bool y_space, bool select_b,
                      const AddressModeCase &mode) {
    Harness h;
    const uint32_t opcode =
        (direct_opcode & ~(uint32_t{7} << 11)) |
        (static_cast<uint32_t>(mode.mmm) << 11);
    unsigned pc = 3;
    if (mode.setup_n) {
        h.p[pc++] = kSetupN0;
        // Address-register writes have a one-instruction pipeline delay.
        h.p[pc++] = 0x000000;
    }
    h.p[pc++] = opcode;
    h.p[pc++] = direct_opcode;
    const unsigned terminal = pc;
    h.p[pc++] = 0x0af080; // JMP >terminal
    h.p[pc] = terminal;

    h.debug_write(0x39, kInitialSr);
    h.debug_write(0x04, kX0);
    h.debug_write(0x06, kY0);
    h.debug_write(0x0e, kA);
    h.debug_write(0x0f, kB);
    h.core.run = 1;

    const bool setup_r = h.retire(kSetupR0);
    const bool setup_n = !mode.setup_n || h.retire(kSetupN0);
    const bool moved = h.retire(opcode);
    const bool followed = h.retire(direct_opcode);
    const uint32_t first_data = select_b ? 0x800000 : 0x7fffff;
    const uint32_t second_data = y_space ? kY0 : kX0;
    const bool x_contract = y_space ?
        (h.x_reads == 0 && h.x_writes == 0) :
        (h.x_reads == 0 && h.x_writes == 2 &&
         h.x_write_addr[0] == mode.effective &&
         h.x_write_data[0] == first_data &&
         h.x_write_addr[1] == mode.final_r &&
         h.x_write_data[1] == second_data);
    const bool y_contract = y_space ?
        (h.y_reads == 0 && h.y_writes == 2 &&
         h.y_write_addr[0] == mode.effective &&
         h.y_write_data[0] == first_data &&
         h.y_write_addr[1] == mode.final_r &&
         h.y_write_data[1] == second_data) :
        (h.y_reads == 0 && h.y_writes == 0);
    const uint64_t expected_a = !select_b ?
        (y_space ? kY0AsAcc : kX0AsAcc) : kA;
    const uint64_t expected_b = select_b ?
        (y_space ? kY0AsAcc : kX0AsAcc) : kB;
    const bool pass = setup_r && setup_n && moved && followed &&
        x_contract && y_contract && h.p_reads == 0 && h.p_writes == 0 &&
        h.core.acc_a_out == expected_a && h.core.acc_b_out == expected_b &&
        h.core.sr_out == (kInitialSr | 0x40);

    const auto &write_addr = y_space ? h.y_write_addr : h.x_write_addr;
    const auto &write_data = y_space ? h.y_write_data : h.x_write_data;
    const unsigned writes = y_space ? h.y_writes : h.x_writes;
    std::printf("%s Class-II mode %-9s %06x %s:%c completed=%u/%u/%u/%u "
                "writes=%u [%04x=%06x,%04x=%06x] "
                "A=%014llx/%014llx B=%014llx/%014llx SR=%06x/000075\n",
                pass ? "PASS" : "FAIL", mode.name, opcode,
                y_space ? "R:Y" : "X:R", select_b ? 'B' : 'A',
                setup_r, setup_n, moved, followed, writes,
                write_addr[0], write_data[0], write_addr[1], write_data[1],
                static_cast<unsigned long long>(h.core.acc_a_out),
                static_cast<unsigned long long>(expected_a),
                static_cast<unsigned long long>(h.core.acc_b_out),
                static_cast<unsigned long long>(expected_b), h.core.sr_out);
    return pass;
}

bool test_movep_overlap() {
    Harness h;
    h.p[0] = kMovepImmediateX;
    h.p[1] = 0x001113;
    h.p[2] = 0x0af080; // JMP >$0002
    h.p[3] = 0x000002;
    h.core.run = 1;

    const bool moved = h.retire(kMovepImmediateX);
    const bool pass = moved && h.x_reads == 0 && h.x_writes == 1 &&
        h.last_x_write_addr == 0xfffe && h.last_x_write_data == 0x001113 &&
        h.y_reads == 0 && h.y_writes == 0 &&
        h.p_reads == 0 && h.p_writes == 0;
    std::printf("%s MOVEP overlap %06x/001113 completed=%u "
                "X=%uR/%uW@%04x=%06x Y=%uR/%uW P=%uR/%uW\n",
                pass ? "PASS" : "FAIL", kMovepImmediateX, moved,
                h.x_reads, h.x_writes, h.last_x_write_addr,
                h.last_x_write_data, h.y_reads, h.y_writes,
                h.p_reads, h.p_writes);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    bool pass = true;
    pass &= test_class2(kClass2AX, false, false);
    pass &= test_class2(kClass2BX, false, true);
    pass &= test_class2(kClass2AY, true, false);
    pass &= test_class2(kClass2BY, true, true);
    for (const auto &mode : kAddressModes) {
        pass &= test_class2_mode(kClass2AX, false, false, mode);
        pass &= test_class2_mode(kClass2BX, false, true, mode);
        pass &= test_class2_mode(kClass2AY, true, false, mode);
        pass &= test_class2_mode(kClass2BY, true, true, mode);
    }
    pass &= test_movep_overlap();
    return pass ? 0 : 1;
}
