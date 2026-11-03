// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 X:Y dual-memory parallel MOVE oracle
//
// DSP56001UMA2 A-149--A-151 defines two independent memory transfers in one
// instruction.  One effective address uses the low R-register bank and the
// other uses the high bank.  This fixture starts with the manual's example:
//
//   MPYR X1,Y0,A X1,X:(R0)+ Y0,Y:(R4)+N4   $D498E1
//
// and follows it with the same sources through direct (R0)/(R4) addressing:
//
//   MPYR X1,Y0,A X1,X:(R0)  Y0,Y:(R4)      $C480E1
//
// The second pair exposes both committed address-register updates without a
// debug-only architectural read port.  Additional instructions exercise the
// load/load and both mixed-direction forms; later stores make every loaded
// register value externally observable.  The first exact expected writes are:
//
//   X:$1000=$123123, Y:$0080=$456456,
//   X:$1001=$123123, Y:$00A3=$456456.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint8_t kStateWritebackFetch = 4;
constexpr uint32_t kSetupR0 = 0x60f400; // MOVE #>$1000,R0
constexpr uint32_t kSetupR4 = 0x64f400; // MOVE #>immediate,R4
constexpr uint32_t kSetupN4 = 0x74f400; // MOVE #>$0023,N4
constexpr uint32_t kManualMove = 0xD498E1;
constexpr uint32_t kDirectFollowup = 0xC480E1;
constexpr uint32_t kBothLoad = 0x8000E1;
constexpr uint32_t kBothStore = 0xC080E1;
constexpr uint32_t kXStoreYLoad = 0xC400E1;
constexpr uint32_t kXLoadYStore = 0x8180E1;

uint32_t short_memory_move(unsigned reg, bool load, unsigned address) {
    return ((0x40u | ((reg >> 3) << 4) | (reg & 7u)) << 16) |
           (load ? 0x8000u : 0u) | ((address & 0x3fu) << 8);
}

uint32_t register_move(unsigned source, unsigned destination) {
    return 0x200000u | (source << 13) | (destination << 8);
}

uint32_t memory_move(unsigned reg, bool load, unsigned mmm, unsigned rrr) {
    return ((0x40u | ((reg >> 3) << 4) | (reg & 7u)) << 16) |
           (load ? 0x8000u : 0u) | 0x4000u | (mmm << 11) | (rrr << 8);
}

uint32_t xy_move(bool x_store, bool y_store, unsigned x_mode,
                 unsigned y_mode, unsigned x_reg, unsigned y_reg,
                 unsigned x_selector, unsigned y_selector) {
    return 0x800000u | (x_store ? 0x400000u : 0u) |
           ((y_mode & 3u) << 20) | ((x_selector & 3u) << 18) |
           ((y_selector & 3u) << 16) | (y_store ? 0x008000u : 0u) |
           ((y_reg & 3u) << 13) | ((x_mode & 3u) << 11) |
           ((x_reg & 7u) << 8);
}

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 32> p{};
    std::array<uint16_t, 8> x_write_addr{};
    std::array<uint32_t, 8> x_write_data{};
    std::array<uint16_t, 8> y_write_addr{};
    std::array<uint32_t, 8> y_write_data{};
    unsigned x_reads = 0;
    unsigned x_writes = 0;
    unsigned y_reads = 0;
    unsigned y_writes = 0;
    unsigned p_reads = 0;
    unsigned p_writes = 0;
    unsigned xy_dual_request_cycles = 0;

    Harness() {
        p[0] = kSetupR0;
        p[1] = 0x001000;
        p[2] = kSetupR4;
        // Keep Y in internal RAM while X is external.  This proves that the
        // independent buses retain same-cycle issue when no SRAM conflict
        // exists, including the N4 update on the manual addressing form.
        p[3] = 0x000080;
        p[4] = kSetupN4;
        p[5] = 0x000023;
        p[6] = 0x000000; // Address-register pipeline delay.
        p[7] = kManualMove;
        p[8] = kDirectFollowup;
        p[9] = kSetupR0;
        p[10] = 0x001100;
        p[11] = kSetupR4;
        p[12] = 0x000200;
        p[13] = 0x000000; // Address-register pipeline delay.
        p[14] = kBothLoad;
        p[15] = kBothStore;
        p[16] = kSetupR0;
        p[17] = 0x001200;
        p[18] = kSetupR4;
        p[19] = 0x000300;
        p[20] = 0x000000; // Address-register pipeline delay.
        p[21] = kXStoreYLoad;
        p[22] = kXLoadYStore;
        p[23] = kBothStore;
        p[24] = 0x0af080; // JMP >$0018
        p[25] = 0x000018;

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
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        if ((core.x_rd_en || core.x_wr_en) &&
            (core.y_rd_en || core.y_wr_en))
            ++xy_dual_request_cycles;
        if (core.x_rd_en) {
            ++x_reads;
            core.x_rdata = (core.x_addr == 0x1100) ? 0xaaa111 :
                           (core.x_addr == 0x1200) ? 0x555666 : 0;
        }
        if (core.x_wr_en) {
            if (x_writes < x_write_addr.size()) {
                x_write_addr[x_writes] = core.x_addr;
                x_write_data[x_writes] = core.x_wdata & 0x00ffffff;
            }
            ++x_writes;
        }
        if (core.y_rd_en) {
            ++y_reads;
            core.y_rdata = (core.y_addr == 0x0200) ? 0xbbb222 :
                           (core.y_addr == 0x0300) ? 0x777888 : 0;
        }
        if (core.y_wr_en) {
            if (y_writes < y_write_addr.size()) {
                y_write_addr[y_writes] = core.y_addr;
                y_write_data[y_writes] = core.y_wdata & 0x00ffffff;
            }
            ++y_writes;
        }
        if (core.p_data_rd_en)
            ++p_reads;
        if (core.p_data_wr_en)
            ++p_writes;
        core.clk = 1;
        core.eval();
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 0x1f] & 0x00ffffff;
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

class MatrixHarness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 32> p{};
    std::array<uint32_t, 65536> x{};
    std::array<uint32_t, 65536> y{};
    unsigned x_reads = 0;
    unsigned x_writes = 0;
    unsigned y_reads = 0;
    unsigned y_writes = 0;
    unsigned p_reads = 0;
    unsigned p_writes = 0;
    unsigned dual_request_cycles = 0;

    MatrixHarness() {
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
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        const bool xr = core.x_rd_en;
        const bool xw = core.x_wr_en;
        const bool yr = core.y_rd_en;
        const bool yw = core.y_wr_en;
        const uint16_t xa = core.x_addr;
        const uint16_t ya = core.y_addr;
        const uint32_t xd = core.x_wdata & 0x00ffffffu;
        const uint32_t yd = core.y_wdata & 0x00ffffffu;
        if ((xr || xw) && (yr || yw))
            ++dual_request_cycles;
        if (xr) {
            ++x_reads;
            core.x_rdata = x[xa];
        }
        if (xw) {
            ++x_writes;
            x[xa] = xd;
        }
        if (yr) {
            ++y_reads;
            core.y_rdata = y[ya];
        }
        if (yw) {
            ++y_writes;
            y[ya] = yd;
        }
        if (core.p_data_rd_en)
            ++p_reads;
        if (core.p_data_wr_en)
            ++p_writes;
        core.clk = 1;
        core.eval();
        if (fetch)
            core.p_fetch_data = p[fetch_addr & 0x1f] & 0x00ffffffu;
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

constexpr std::array<uint8_t, 4> kXRegisterCodes{{0x04, 0x05, 0x0e, 0x0f}};
constexpr std::array<uint8_t, 4> kYRegisterCodes{{0x06, 0x07, 0x0e, 0x0f}};
constexpr std::array<uint32_t, 4> kXRegisterData{{
    0x110001, 0x110002, 0x110003, 0x110004,
}};
constexpr std::array<uint32_t, 4> kYRegisterData{{
    0x120001, 0x120002, 0x110003, 0x110004,
}};

uint16_t updated_address(uint16_t initial, uint16_t offset, unsigned mode) {
    switch (mode & 3u) {
    case 1: return initial + offset;
    case 2: return initial - 1;
    case 3: return initial + 1;
    default: return initial;
    }
}

bool check_matrix_case(unsigned ordinal, bool x_store, bool y_store,
                       unsigned x_mode, unsigned y_mode, unsigned x_reg,
                       unsigned y_reg, unsigned x_selector,
                       unsigned y_selector, bool verbose) {
    MatrixHarness h;
    const uint16_t x_address = 0x20u + 4u * x_reg;
    const unsigned architectural_y_reg = ((~x_reg >> 2) & 1u) * 4u + y_reg;
    const uint16_t y_address = 0x60u + 4u * architectural_y_reg;
    const uint16_t x_offset = 3;
    const uint16_t y_offset = 5;
    const uint32_t x_input = 0x200000u | (ordinal & 0x0fffffu);
    const uint32_t y_input = 0x300000u | (ordinal & 0x0fffffu);
    const uint32_t opcode = xy_move(x_store, y_store, x_mode, y_mode,
                                    x_reg, y_reg, x_selector, y_selector);

    h.debug_write(0x04, kXRegisterData[0]);
    h.debug_write(0x05, kXRegisterData[1]);
    h.debug_write(0x06, kYRegisterData[0]);
    h.debug_write(0x07, kYRegisterData[1]);
    h.debug_write(0x0e, uint64_t{kXRegisterData[2]} << 24);
    h.debug_write(0x0f, uint64_t{kXRegisterData[3]} << 24);
    h.x[x_address] = x_input;
    h.y[y_address] = y_input;

    unsigned next = 0;
    const uint32_t setup_x_r = memory_move(0x10 + x_reg, true, 6, 4);
    const uint32_t setup_x_n = memory_move(0x18 + x_reg, true, 6, 4);
    const uint32_t setup_y_r = memory_move(0x10 + architectural_y_reg,
                                          true, 6, 4);
    const uint32_t setup_y_n = memory_move(0x18 + architectural_y_reg,
                                          true, 6, 4);
    h.p[next++] = setup_x_r;
    h.p[next++] = x_address;
    h.p[next++] = setup_x_n;
    h.p[next++] = x_offset;
    h.p[next++] = setup_y_r;
    h.p[next++] = y_address;
    h.p[next++] = setup_y_n;
    h.p[next++] = y_offset;
    h.p[next++] = 0x000000; // Settle explicit R/N writes before AGU use.
    h.p[next++] = opcode;
    if (!x_store)
        h.p[next++] = short_memory_move(kXRegisterCodes[x_selector], false,
                                        0x3e);
    if (!y_store)
        h.p[next++] = short_memory_move(kYRegisterCodes[y_selector], false,
                                        0x3f);
    h.p[next++] = register_move(0x10 + x_reg, 0x0c);
    h.p[next++] = register_move(0x10 + architectural_y_reg, 0x0d);
    const unsigned terminal = next;
    h.p[next++] = 0x0af080;
    h.p[next] = terminal;
    h.core.run = 1;

    const bool setup = h.retire(setup_x_r) && h.retire(setup_x_n) &&
                       h.retire(setup_y_r) && h.retire(setup_y_n) &&
                       h.retire(0x000000);
    h.x_reads = h.x_writes = h.y_reads = h.y_writes = 0;
    h.p_reads = h.p_writes = h.dual_request_cycles = 0;
    const bool moved = h.retire(opcode);
    const unsigned move_x_reads = h.x_reads;
    const unsigned move_x_writes = h.x_writes;
    const unsigned move_y_reads = h.y_reads;
    const unsigned move_y_writes = h.y_writes;
    const unsigned move_dual_cycles = h.dual_request_cycles;
    bool observed = true;
    if (!x_store)
        observed &= h.retire(short_memory_move(kXRegisterCodes[x_selector],
                                               false, 0x3e));
    if (!y_store)
        observed &= h.retire(short_memory_move(kYRegisterCodes[y_selector],
                                               false, 0x3f));
    observed &= h.retire(register_move(0x10 + x_reg, 0x0c));
    observed &= h.retire(register_move(0x10 + architectural_y_reg, 0x0d));

    const uint32_t actual_x = x_store ? h.x[x_address] : h.x[0x3e];
    const uint32_t actual_y = y_store ? h.y[y_address] : h.x[0x3f];
    const uint32_t expected_x = x_store ? kXRegisterData[x_selector] : x_input;
    const uint32_t expected_y = y_store ? kYRegisterData[y_selector] : y_input;
    const uint16_t actual_x_r = (h.core.acc_a_out >> 24) & 0xffffu;
    const uint16_t actual_y_r = (h.core.acc_b_out >> 24) & 0xffffu;
    const uint16_t expected_x_r = updated_address(x_address, x_offset, x_mode);
    const uint16_t expected_y_r = updated_address(y_address, y_offset, y_mode);
    const bool pass = setup && moved && observed && actual_x == expected_x &&
        actual_y == expected_y && actual_x_r == expected_x_r &&
        actual_y_r == expected_y_r &&
        move_x_reads == (x_store ? 0u : 1u) &&
        move_x_writes == (x_store ? 1u : 0u) &&
        move_y_reads == (y_store ? 0u : 1u) &&
        move_y_writes == (y_store ? 1u : 0u) &&
        move_dual_cycles == 1 && h.p_reads == 0 && h.p_writes == 0;
    if (verbose || !pass) {
        std::printf("%s X:Y matrix %04u dir=%u%u sel=%u/%u mode=%u/%u "
                    "R=%u/%u addr=%04x/%04x->%04x/%04x "
                    "data=%06x/%06x expected=%06x/%06x traffic=%u%u/%u%u dual=%u\n",
                    pass ? "PASS" : "FAIL", ordinal, x_store, y_store,
                    x_selector, y_selector, x_mode, y_mode, x_reg,
                    architectural_y_reg, x_address, y_address, actual_x_r,
                    actual_y_r, actual_x, actual_y, expected_x, expected_y,
                    move_x_reads, move_x_writes, move_y_reads, move_y_writes,
                    move_dual_cycles);
    }
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Harness h;

    h.debug_write(0x05, 0x123123); // X1
    h.debug_write(0x06, 0x456456); // Y0
    h.debug_write(0x07, 0x333444); // Y1
    h.core.run = 1;

    const bool setup_r0 = h.retire(kSetupR0);
    const bool setup_r4 = h.retire(kSetupR4);
    const bool setup_n4 = h.retire(kSetupN4);
    const bool manual = h.retire(kManualMove);
    const bool followup = h.retire(kDirectFollowup);
    const bool setup_load_r0 = h.retire(kSetupR0);
    const bool setup_load_r4 = h.retire(kSetupR4);
    const bool both_load = h.retire(kBothLoad);
    const bool load_observe = h.retire(kBothStore);
    const bool setup_mixed_r0 = h.retire(kSetupR0);
    const bool setup_mixed_r4 = h.retire(kSetupR4);
    const bool x_store_y_load = h.retire(kXStoreYLoad);
    const bool x_load_y_store = h.retire(kXLoadYStore);
    const bool mixed_observe = h.retire(kBothStore);
    const bool completed = setup_r0 && setup_r4 && setup_n4 && manual &&
        followup && setup_load_r0 && setup_load_r4 && both_load &&
        load_observe && setup_mixed_r0 && setup_mixed_r4 && x_store_y_load &&
        x_load_y_store && mixed_observe;
    const bool pass = completed && h.x_reads == 2 && h.x_writes == 5 &&
        h.x_write_addr[0] == 0x1000 && h.x_write_data[0] == 0x123123 &&
        h.x_write_addr[1] == 0x1001 && h.x_write_data[1] == 0x123123 &&
        h.x_write_addr[2] == 0x1100 && h.x_write_data[2] == 0xaaa111 &&
        h.x_write_addr[3] == 0x1200 && h.x_write_data[3] == 0x123123 &&
        h.x_write_addr[4] == 0x1200 && h.x_write_data[4] == 0x555666 &&
        h.y_reads == 2 && h.y_writes == 5 &&
        h.y_write_addr[0] == 0x0080 && h.y_write_data[0] == 0x456456 &&
        h.y_write_addr[1] == 0x00a3 && h.y_write_data[1] == 0x456456 &&
        h.y_write_addr[2] == 0x0200 && h.y_write_data[2] == 0xbbb222 &&
        h.y_write_addr[3] == 0x0300 && h.y_write_data[3] == 0x333444 &&
        h.y_write_addr[4] == 0x0300 && h.y_write_data[4] == 0x777888 &&
        h.p_reads == 0 && h.p_writes == 0 &&
        h.xy_dual_request_cycles == 2;

    std::printf("%s X:Y all-directions completed=%u "
                "X=%uR/%uW first=%04x:%06x last=%04x:%06x "
                "Y=%uR/%uW first=%04x:%06x last=%04x:%06x "
                "P=%uR/%uW dual_cycles=%u\n",
                pass ? "PASS" : "FAIL", completed,
                h.x_reads, h.x_writes, h.x_write_addr[0], h.x_write_data[0],
                h.x_write_addr[4], h.x_write_data[4],
                h.y_reads, h.y_writes, h.y_write_addr[0], h.y_write_data[0],
                h.y_write_addr[4], h.y_write_data[4],
                h.p_reads, h.p_writes, h.xy_dual_request_cycles);

    unsigned matrix_cases = 0;
    unsigned matrix_failures = 0;
    // Exhaust every legal selector combination in every direction.  A/A and
    // B/B dual loads are the manual's nonsensical same-destination forms and
    // are deliberately outside this legal-selector matrix.
    for (unsigned direction = 0; direction < 4; ++direction) {
        const bool x_store = direction & 2u;
        const bool y_store = direction & 1u;
        for (unsigned xs = 0; xs < 4; ++xs) {
            for (unsigned ys = 0; ys < 4; ++ys) {
                if (!x_store && !y_store && xs >= 2 && xs == ys)
                    continue;
                ++matrix_cases;
                matrix_failures += !check_matrix_case(
                    matrix_cases, x_store, y_store, 0, 0, 0, 0, xs, ys,
                    false);
            }
        }
    }
    // Independently exhaust both restricted AGUs, including both bank
    // orientations and every Rn subindex, in all four transfer directions.
    for (unsigned direction = 0; direction < 4; ++direction) {
        const bool x_store = direction & 2u;
        const bool y_store = direction & 1u;
        for (unsigned xm = 0; xm < 4; ++xm) {
            for (unsigned ym = 0; ym < 4; ++ym) {
                for (unsigned xr = 0; xr < 8; ++xr) {
                    for (unsigned yr = 0; yr < 4; ++yr) {
                        ++matrix_cases;
                        matrix_failures += !check_matrix_case(
                            matrix_cases, x_store, y_store, xm, ym, xr, yr,
                            0, 0, false);
                    }
                }
            }
        }
    }
    std::printf("%s X:Y MATRIX SUMMARY cases=%u failures=%u "
                "directions=4 selector-pairs=62 mode/register-cases=2048\n",
                matrix_failures ? "FAIL" : "PASS", matrix_cases,
                matrix_failures);
    return (pass && matrix_failures == 0) ? 0 : 1;
}
