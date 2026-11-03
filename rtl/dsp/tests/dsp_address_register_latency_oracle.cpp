// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 move-type address-register pipeline latency oracle
//
// DSP56001UMA3 section A.8.6 states that an R/N/M register changed by a
// move-type instruction is not available *as a pointer* to the immediately
// following instruction.  That follower must use the old address-register
// value; one intervening instruction makes the new value visible.
//
// Initialize R0 through the production instruction path with MOVE #>$0020,R0
// and a settling NOP.  Then exercise MOVE #>$0040,R0 followed by a Class-I
// MOVE X:(R0),X0.  The no-gap case must read X:$0020, while the NOP control
// must read X:$0040.  Distinct memory data is published through B so an
// address-only implementation or a timing-only no-op cannot pass.  Companion
// cases cover N0 and M0 pointer effects and prove the documented exclusions:
// ordinary register-data reads see the new value immediately, and an R0
// update performed by an addressing mode is visible to the next instruction.
// The manual also states that the delay applies across a DO-loop boundary.
// A loop-tail MOVE to R0 must therefore leave the old pointer visible at the
// loop top while making the new pointer visible after the final iteration.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint8_t kWritebackState = 4;
constexpr uint32_t kMoveImmediateR0 = 0x60f400;
constexpr uint32_t kMoveShortImmediateR0New = 0x304000;
constexpr uint32_t kMoveImmediateN0Old = 0x380300;
constexpr uint32_t kMoveImmediateN0New = 0x380500;
constexpr uint32_t kMovecImmediateM0 = 0x05f420;
constexpr uint32_t kMoveXAtR0ToX0 = 0x10a000;
constexpr uint32_t kMoveXAtR5ToX0 = 0x10a500;
constexpr uint32_t kMoveXAtR0PlusN0ToX0 = 0x10a800;
constexpr uint32_t kMoveXAtR0PostincN0ToX0 = 0x108800;
constexpr uint32_t kMoveXAtR0PostincToX0 = 0x109800;
constexpr uint32_t kPublishX0ToB = 0x208f00;
constexpr uint32_t kPublishR0ToA1 = 0x220c00;
constexpr uint32_t kNop = 0x000000;
constexpr uint32_t kJump = 0x0af080;
constexpr uint32_t kDoImmediate2 = 0x060280;
constexpr uint32_t kTgtDualR2R5 = 0x03720d;
constexpr uint32_t kMovemP20ToR0 = 0x07a010;
constexpr uint32_t kMovepX2cToR0 = 0x08502c;
constexpr uint32_t kParallelTstAR2ToR0 = 0x225003;
constexpr uint16_t kOldR0 = 0x0020;
constexpr uint16_t kNewR0 = 0x0040;
constexpr uint32_t kOldData = 0x13579b;
constexpr uint32_t kNewData = 0x2468ac;

enum class Scenario {
    RPointer,
    NPointer,
    MPointer,
    RData,
    AddressModeUpdate,
    DoWrap,
    TccPointer,
    MovemPointer,
    MovepPointer,
    ParallelPointer,
};

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 65536> p{};
    std::array<uint32_t, 65536> x{};
    unsigned x_reads = 0;
    unsigned x_writes = 0;
    unsigned y_reads = 0;
    unsigned y_writes = 0;
    unsigned p_reads = 0;
    unsigned p_writes = 0;
    uint16_t first_x_read_addr = 0xffff;
    std::array<uint16_t, 4> x_read_addr{};

    Harness(Scenario scenario, bool insert_nop) {
        unsigned pc = 0;
        auto emit_r = [&](unsigned reg, uint16_t value) {
            p[pc++] = kMoveImmediateR0 + (reg << 16);
            p[pc++] = value;
        };
        auto emit_r0 = [&](uint16_t value) { emit_r(0, value); };
        auto emit_m0 = [&](uint16_t value) {
            p[pc++] = kMovecImmediateM0;
            p[pc++] = value;
        };

        if (scenario == Scenario::RPointer) {
            emit_r0(kOldR0);
            p[pc++] = kNop;
            emit_r0(kNewR0);
            if (insert_nop)
                p[pc++] = kNop;
            p[pc++] = kMoveXAtR0ToX0;
            p[pc++] = kPublishX0ToB;
        } else if (scenario == Scenario::NPointer) {
            emit_r0(kOldR0);
            p[pc++] = kMoveImmediateN0Old;
            p[pc++] = kNop;
            p[pc++] = kMoveImmediateN0New;
            if (insert_nop)
                p[pc++] = kNop;
            p[pc++] = kMoveXAtR0PlusN0ToX0;
        } else if (scenario == Scenario::MPointer) {
            emit_r0(0x0007);
            p[pc++] = 0x380100; // MOVE #$01,N0.
            emit_m0(0xffff);
            p[pc++] = kNop;
            emit_m0(0x0007);
            if (insert_nop)
                p[pc++] = kNop;
            p[pc++] = kMoveXAtR0PostincN0ToX0;
            p[pc++] = kPublishR0ToA1;
        } else if (scenario == Scenario::RData) {
            emit_r0(kOldR0);
            p[pc++] = kNop;
            emit_r0(kNewR0);
            p[pc++] = kPublishR0ToA1;
        } else if (scenario == Scenario::AddressModeUpdate) {
            emit_r0(kOldR0);
            p[pc++] = kNop;
            p[pc++] = kMoveXAtR0PostincToX0;
            p[pc++] = kMoveXAtR0ToX0;
        } else if (scenario == Scenario::DoWrap) {
            emit_r0(kOldR0);
            p[pc++] = kNop;
            p[pc++] = kDoImmediate2;
            p[pc++] = 0x000006;
            p[pc++] = kMoveXAtR0ToX0;
            p[pc++] = kMoveShortImmediateR0New;
            p[pc++] = kMoveXAtR0ToX0;
        } else if (scenario == Scenario::TccPointer) {
            emit_r(5, kOldR0);
            emit_r(2, kNewR0);
            p[pc++] = kNop;
            p[pc++] = kTgtDualR2R5;
            if (insert_nop)
                p[pc++] = kNop;
            p[pc++] = kMoveXAtR5ToX0;
        } else if (scenario == Scenario::MovemPointer) {
            emit_r0(kOldR0);
            p[pc++] = kNop;
            p[pc++] = kMovemP20ToR0;
            if (insert_nop)
                p[pc++] = kNop;
            p[pc++] = kMoveXAtR0ToX0;
            p[0x20] = kNewR0;
        } else if (scenario == Scenario::MovepPointer) {
            emit_r0(kOldR0);
            p[pc++] = kNop;
            p[pc++] = kMovepX2cToR0;
            if (insert_nop)
                p[pc++] = kNop;
            p[pc++] = kMoveXAtR0ToX0;
            x[0xffec] = kNewR0;
        } else {
            emit_r0(kOldR0);
            emit_r(2, kNewR0);
            p[pc++] = kNop;
            p[pc++] = kParallelTstAR2ToR0;
            if (insert_nop)
                p[pc++] = kNop;
            p[pc++] = kMoveXAtR0ToX0;
        }
        const unsigned terminal = pc;
        p[pc++] = kJump;
        p[pc] = terminal;
        x[kOldR0] = kOldData;
        x[kNewR0] = kNewData;
        x[kOldR0 + 3] = kOldData;
        x[kOldR0 + 5] = kNewData;

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
        core.run = 1;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        const bool x_read = core.x_rd_en;
        const bool x_write = core.x_wr_en;
        const bool y_read = core.y_rd_en;
        const bool y_write = core.y_wr_en;
        const bool p_read = core.p_data_rd_en;
        const bool p_write = core.p_data_wr_en;
        const uint16_t x_addr = core.x_addr;
        const uint16_t y_addr = core.y_addr;
        const uint16_t p_addr = core.p_data_addr;
        const uint32_t x_wdata = core.x_wdata;
        const uint32_t y_wdata = core.y_wdata;
        const uint32_t p_wdata = core.p_data_wdata;

        if (x_read) {
            if (x_reads == 0)
                first_x_read_addr = x_addr;
            if (x_reads < x_read_addr.size())
                x_read_addr[x_reads] = x_addr;
            ++x_reads;
        }
        x_writes += x_write;
        y_reads += y_read;
        y_writes += y_write;
        p_reads += p_read;
        p_writes += p_write;

        core.clk = 1;
        core.eval();
        if (fetch)
            core.p_fetch_data = p[fetch_addr] & 0x00ffffff;
        if (p_read)
            core.p_data_rdata = p[p_addr] & 0x00ffffff;
        if (p_write)
            p[p_addr] = p_wdata & 0x00ffffff;
        if (x_read)
            core.x_rdata = x[x_addr] & 0x00ffffff;
        if (x_write)
            x[x_addr] = x_wdata & 0x00ffffff;
        if (y_read)
            core.y_rdata = 0;
        if (y_write)
            (void)y_wdata;
    }

    bool retire(uint32_t instruction) {
        for (unsigned i = 0; i < 100; ++i) {
            if (core.state_out == kWritebackState &&
                core.cur_inst_out == instruction) {
                tick();
                return true;
            }
            tick();
        }
        return false;
    }
};

bool run_case(bool insert_nop) {
    Harness h(Scenario::RPointer, insert_nop);
    const bool initialized = h.retire(kMoveImmediateR0);
    const bool init_gap = h.retire(kNop);
    const bool setup = h.retire(kMoveImmediateR0);
    const bool spacing = !insert_nop || h.retire(kNop);
    const bool moved = h.retire(kMoveXAtR0ToX0);
    const bool published = h.retire(kPublishX0ToB);
    const uint16_t expected_addr = insert_nop ? kNewR0 : kOldR0;
    const uint32_t expected_data = insert_nop ? kNewData : kOldData;
    const uint32_t actual_data =
        static_cast<uint32_t>((h.core.acc_b_out >> 24) & 0x00ffffff);
    const bool pass = initialized && init_gap && setup && spacing && moved && published &&
        h.x_reads == 1 && h.x_writes == 0 && h.y_reads == 0 &&
        h.y_writes == 0 && h.first_x_read_addr == expected_addr &&
        actual_data == expected_data;
    std::printf(
        "%s ADDRESS_REGISTER_LATENCY gap=%s retire=%u/%u/%u/%u/%u/%u "
        "X=%uR/%uW@%04x/%04x Y=%uR/%uW data=%06x/%06x\n",
        pass ? "PASS" : "FAIL", insert_nop ? "NOP" : "none", initialized,
        init_gap, setup, spacing, moved, published, h.x_reads, h.x_writes,
        h.first_x_read_addr, expected_addr, h.y_reads, h.y_writes,
        actual_data, expected_data);
    return pass;
}

bool run_n_case(bool insert_nop) {
    Harness h(Scenario::NPointer, insert_nop);
    const bool setup_r = h.retire(kMoveImmediateR0);
    const bool setup_old_n = h.retire(kMoveImmediateN0Old);
    const bool init_gap = h.retire(kNop);
    const bool setup_new_n = h.retire(kMoveImmediateN0New);
    const bool spacing = !insert_nop || h.retire(kNop);
    const bool moved = h.retire(kMoveXAtR0PlusN0ToX0);
    const uint16_t expected_addr = kOldR0 + (insert_nop ? 5 : 3);
    const bool pass = setup_r && setup_old_n && init_gap && setup_new_n &&
        spacing && moved && h.x_reads == 1 && h.x_writes == 0 &&
        h.y_reads == 0 && h.y_writes == 0 &&
        h.first_x_read_addr == expected_addr;
    std::printf(
        "%s ADDRESS_REGISTER_N_POINTER gap=%s retire=%u/%u/%u/%u/%u/%u "
        "X=%uR@%04x/%04x\n",
        pass ? "PASS" : "FAIL", insert_nop ? "NOP" : "none", setup_r,
        setup_old_n, init_gap, setup_new_n, spacing, moved, h.x_reads,
        h.first_x_read_addr, expected_addr);
    return pass;
}

bool run_m_case(bool insert_nop) {
    Harness h(Scenario::MPointer, insert_nop);
    const bool setup_r = h.retire(kMoveImmediateR0);
    const bool setup_n = h.retire(0x380100);
    const bool setup_old_m = h.retire(kMovecImmediateM0);
    const bool init_gap = h.retire(kNop);
    const bool setup_new_m = h.retire(kMovecImmediateM0);
    const bool spacing = !insert_nop || h.retire(kNop);
    const bool moved = h.retire(kMoveXAtR0PostincN0ToX0);
    const bool published = h.retire(kPublishR0ToA1);
    const uint16_t expected_r0 = insert_nop ? 0x0000 : 0x0008;
    const uint16_t actual_r0 =
        static_cast<uint16_t>((h.core.acc_a_out >> 24) & 0xffff);
    const bool pass = setup_r && setup_n && setup_old_m && init_gap &&
        setup_new_m && spacing && moved && published && h.x_reads == 1 &&
        h.first_x_read_addr == 0x0007 && actual_r0 == expected_r0;
    std::printf(
        "%s ADDRESS_REGISTER_M_POINTER gap=%s retire=%u/%u/%u/%u/%u/%u/%u/%u "
        "X=%uR@%04x R0=%04x/%04x\n",
        pass ? "PASS" : "FAIL", insert_nop ? "NOP" : "none", setup_r,
        setup_n, setup_old_m, init_gap, setup_new_m, spacing, moved, published,
        h.x_reads, h.first_x_read_addr, actual_r0, expected_r0);
    return pass;
}

bool run_data_visibility_case() {
    Harness h(Scenario::RData, false);
    const bool setup_old = h.retire(kMoveImmediateR0);
    const bool init_gap = h.retire(kNop);
    const bool setup_new = h.retire(kMoveImmediateR0);
    const bool published = h.retire(kPublishR0ToA1);
    const uint16_t actual_r0 =
        static_cast<uint16_t>((h.core.acc_a_out >> 24) & 0xffff);
    const bool pass = setup_old && init_gap && setup_new && published &&
        actual_r0 == kNewR0 && h.x_reads == 0 && h.x_writes == 0;
    std::printf(
        "%s ADDRESS_REGISTER_DATA_IMMEDIATE retire=%u/%u/%u/%u "
        "R0=%04x/%04x\n",
        pass ? "PASS" : "FAIL", setup_old, init_gap, setup_new, published,
        actual_r0, kNewR0);
    return pass;
}

bool run_address_update_case() {
    Harness h(Scenario::AddressModeUpdate, false);
    const bool setup = h.retire(kMoveImmediateR0);
    const bool init_gap = h.retire(kNop);
    const bool updated = h.retire(kMoveXAtR0PostincToX0);
    const bool followed = h.retire(kMoveXAtR0ToX0);
    const bool pass = setup && init_gap && updated && followed &&
        h.x_reads == 2 && h.x_read_addr[0] == kOldR0 &&
        h.x_read_addr[1] == kOldR0 + 1;
    std::printf(
        "%s ADDRESS_MODE_UPDATE_IMMEDIATE retire=%u/%u/%u/%u "
        "X=%uR@%04x,%04x/%04x,%04x\n",
        pass ? "PASS" : "FAIL", setup, init_gap, updated, followed,
        h.x_reads, h.x_read_addr[0], h.x_read_addr[1], kOldR0, kOldR0 + 1);
    return pass;
}

bool run_do_wrap_case() {
    Harness h(Scenario::DoWrap, false);
    const bool setup = h.retire(kMoveImmediateR0);
    const bool init_gap = h.retire(kNop);
    const bool loop_setup = h.retire(kDoImmediate2);
    const bool first_read = h.retire(kMoveXAtR0ToX0);
    const bool first_write = h.retire(kMoveShortImmediateR0New);
    const bool wrap_read = h.retire(kMoveXAtR0ToX0);
    const bool final_write = h.retire(kMoveShortImmediateR0New);
    const bool exit_read = h.retire(kMoveXAtR0ToX0);
    const bool pass = setup && init_gap && loop_setup && first_read &&
        first_write && wrap_read && final_write && exit_read &&
        h.x_reads == 3 && h.x_writes == 0 && h.y_reads == 0 &&
        h.y_writes == 0 && h.x_read_addr[0] == kOldR0 &&
        h.x_read_addr[1] == kOldR0 && h.x_read_addr[2] == kNewR0;
    std::printf(
        "%s ADDRESS_REGISTER_DO_WRAP retire=%u/%u/%u/%u/%u/%u/%u/%u "
        "X=%uR/%uW@%04x,%04x,%04x/%04x,%04x,%04x Y=%uR/%uW\n",
        pass ? "PASS" : "FAIL", setup, init_gap, loop_setup, first_read,
        first_write, wrap_read, final_write, exit_read, h.x_reads,
        h.x_writes, h.x_read_addr[0], h.x_read_addr[1], h.x_read_addr[2],
        kOldR0, kOldR0, kNewR0, h.y_reads, h.y_writes);
    return pass;
}

bool run_move_type_writer_case(Scenario scenario, const char *name,
                               uint32_t writer, uint32_t pointer,
                               bool insert_nop) {
    Harness h(scenario, insert_nop);
    const bool tcc = scenario == Scenario::TccPointer;
    const bool parallel = scenario == Scenario::ParallelPointer;
    const uint32_t old_setup = tcc ?
        kMoveImmediateR0 + (5u << 16) : kMoveImmediateR0;
    const bool setup_old = h.retire(old_setup);
    const bool setup_new = !(tcc || parallel) ||
        h.retire(kMoveImmediateR0 + (2u << 16));
    const bool init_gap = h.retire(kNop);
    const bool wrote = h.retire(writer);
    const bool spacing = !insert_nop || h.retire(kNop);
    const bool moved = h.retire(pointer);
    const uint16_t expected_addr = insert_nop ? kNewR0 : kOldR0;
    const unsigned pointer_read_index =
        scenario == Scenario::MovepPointer ? 1u : 0u;
    const bool traffic =
        scenario == Scenario::MovemPointer ?
            h.x_reads == 1 && h.p_reads == 1 :
        scenario == Scenario::MovepPointer ?
            h.x_reads == 2 && h.x_read_addr[0] == 0xffec && h.p_reads == 0 :
            h.x_reads == 1 && h.p_reads == 0;
    const bool pass = setup_old && setup_new && init_gap && wrote && spacing &&
        moved && traffic && h.x_read_addr[pointer_read_index] == expected_addr &&
        h.x_writes == 0 && h.y_reads == 0 && h.y_writes == 0 &&
        h.p_writes == 0;
    std::printf(
        "%s ADDRESS_REGISTER_WRITER %-8s gap=%s "
        "retire=%u/%u/%u/%u/%u/%u X=%uR/%uW@%04x/%04x "
        "P=%uR/%uW Y=%uR/%uW\n",
        pass ? "PASS" : "FAIL", name, insert_nop ? "NOP" : "none",
        setup_old, setup_new, init_gap, wrote, spacing, moved, h.x_reads,
        h.x_writes, h.x_read_addr[pointer_read_index], expected_addr,
        h.p_reads, h.p_writes, h.y_reads, h.y_writes);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const bool immediate_follower = run_case(false);
    const bool one_instruction_gap = run_case(true);
    const bool n_immediate_follower = run_n_case(false);
    const bool n_one_instruction_gap = run_n_case(true);
    const bool m_immediate_follower = run_m_case(false);
    const bool m_one_instruction_gap = run_m_case(true);
    const bool data_immediate = run_data_visibility_case();
    const bool address_update_immediate = run_address_update_case();
    const bool do_wrap = run_do_wrap_case();
    const bool tcc_immediate = run_move_type_writer_case(
        Scenario::TccPointer, "Tcc", kTgtDualR2R5, kMoveXAtR5ToX0, false);
    const bool tcc_gap = run_move_type_writer_case(
        Scenario::TccPointer, "Tcc", kTgtDualR2R5, kMoveXAtR5ToX0, true);
    const bool movem_immediate = run_move_type_writer_case(
        Scenario::MovemPointer, "MOVEM", kMovemP20ToR0,
        kMoveXAtR0ToX0, false);
    const bool movem_gap = run_move_type_writer_case(
        Scenario::MovemPointer, "MOVEM", kMovemP20ToR0,
        kMoveXAtR0ToX0, true);
    const bool movep_immediate = run_move_type_writer_case(
        Scenario::MovepPointer, "MOVEP", kMovepX2cToR0,
        kMoveXAtR0ToX0, false);
    const bool movep_gap = run_move_type_writer_case(
        Scenario::MovepPointer, "MOVEP", kMovepX2cToR0,
        kMoveXAtR0ToX0, true);
    const bool parallel_immediate = run_move_type_writer_case(
        Scenario::ParallelPointer, "parallel", kParallelTstAR2ToR0,
        kMoveXAtR0ToX0, false);
    const bool parallel_gap = run_move_type_writer_case(
        Scenario::ParallelPointer, "parallel", kParallelTstAR2ToR0,
        kMoveXAtR0ToX0, true);
    return immediate_follower && one_instruction_gap &&
        n_immediate_follower && n_one_instruction_gap &&
        m_immediate_follower && m_one_instruction_gap && data_immediate &&
        address_update_immediate && do_wrap && tcc_immediate && tcc_gap &&
        movem_immediate && movem_gap && movep_immediate && movep_gap &&
        parallel_immediate && parallel_gap ? 0 : 1;
}
