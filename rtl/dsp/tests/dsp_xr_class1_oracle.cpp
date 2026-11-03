// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 Class-I X:R parallel data-move oracle
//
// DSP56001UMA2 A-131--A-134 defines one X-memory transfer in parallel with an
// accumulator-to-input-register transfer.  This fixture uses the exact
// direct-(R0) encodings with a NOP ALU byte so both data moves are isolated:
//
//     MOVE X0,X:(R0) A,Y0      $102000
//     MOVE X:(R0),X0 A,Y0      $10a000
//     MOVE X0,X:(R0)+ A,Y0     $101800
//     MOVE X0,X:(R0)- A,Y0     $101000
//     MOVE X0,X:(R0)-N0 A,Y0   $100000
//     MOVE X0,X:(R0)+N0 A,Y0   $100800
//     MOVE X0,X:(R0+N0) A,Y0   $102800
//     MOVE X0,X:-(R0) A,Y0     $103800
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
constexpr uint32_t kClass1Store = 0x102000;
constexpr uint32_t kClass1StoreALimitBToY0 = 0x1a2000;
constexpr uint32_t kClass1Load = 0x10a000;
constexpr uint32_t kClass1StorePostinc = 0x101800;
constexpr uint32_t kClass1StorePostdec = 0x101000;
constexpr uint32_t kClass1StorePostdecN = 0x100000;
constexpr uint32_t kClass1StorePostincN = 0x100800;
constexpr uint32_t kClass1StoreIndexed = 0x102800;
constexpr uint32_t kClass1StorePredec = 0x103800;
constexpr uint32_t kClass1LoadPostdecN = 0x108000;
constexpr uint32_t kClass1LoadPostincN = 0x108800;
constexpr uint32_t kClass1LoadPostdec = 0x109000;
constexpr uint32_t kClass1LoadPostinc = 0x109800;
constexpr uint32_t kClass1LoadIndexed = 0x10a800;
constexpr uint32_t kClass1LoadPredec = 0x10b800;
constexpr uint32_t kClass1StoreAbsolute = 0x103000;
constexpr uint32_t kClass1LoadAbsolute = 0x10b000;
constexpr uint32_t kClass1LoadImmediate = 0x10b400;
constexpr uint32_t kReadX0ToB = 0x208f00;
constexpr uint32_t kReadY0ToB = 0x20cf00;

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
    unsigned p_reads = 0;
    unsigned p_writes = 0;
    uint16_t last_x_read_addr = 0;
    uint16_t first_x_addr = 0;
    uint16_t last_x_addr = 0;
    uint32_t last_x_data = 0;

    Harness() {
        p[0] = kSetupR0;
        p[1] = 0x001234;
        p[2] = 0x000000; // Settle explicit R0 write before AGU use.
        p[3] = kClass1Store;
        p[4] = kReadY0ToB;
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
        if (core.x_rd_en) {
            ++x_reads;
            last_x_read_addr = core.x_addr;
        }
        if (core.x_wr_en) {
            if (x_writes == 0)
                first_x_addr = core.x_addr;
            ++x_writes;
            last_x_addr = core.x_addr;
            last_x_data = core.x_wdata & 0x00ffffff;
        }
        if (core.y_rd_en) ++y_reads;
        if (core.y_wr_en) ++y_writes;
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

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Harness h;
    constexpr uint32_t kInitialSr = 0x000035;
    constexpr uint32_t kX0 = 0x2468ac;
    constexpr uint64_t kA = 0x0112345689abcdULL;
    constexpr uint64_t kB = 0x002468ac13579bULL;
    h.debug_write(0x39, kInitialSr);
    h.debug_write(0x04, kX0);
    h.debug_write(0x06, 0x13579b); // Y0 sentinel
    h.debug_write(0x0e, kA);
    h.debug_write(0x0f, kB);
    h.core.run = 1;

    const bool setup = h.retire(kSetupR0);
    const bool moved = h.retire(kClass1Store);
    const bool readback = h.retire(kReadY0ToB);
    const uint32_t y0_word = (h.core.acc_b_out >> 24) & 0x00ffffff;
    const bool store_pass = setup && moved && readback &&
        h.x_reads == 0 && h.x_writes == 1 &&
        h.last_x_addr == 0x1234 && h.last_x_data == kX0 &&
        h.y_reads == 0 && h.y_writes == 0 &&
        h.p_reads == 0 && h.p_writes == 0 &&
        y0_word == 0x7fffff && h.core.acc_a_out == kA &&
        h.core.sr_out == (kInitialSr | 0x40);
    std::printf("%s X:R Class-I 102000: completed=%u/%u/%u "
                "X=%uR/%uW@%04x=%06x Y=%uR/%uW P=%uR/%uW "
                "Y0=%06x/7fffff A=%014llx/%014llx SR=%06x/000075\n",
                store_pass ? "PASS" : "FAIL", setup, moved, readback,
                h.x_reads, h.x_writes, h.last_x_addr, h.last_x_data,
                h.y_reads, h.y_writes, h.p_reads, h.p_writes, y0_word,
                static_cast<unsigned long long>(h.core.acc_a_out),
                static_cast<unsigned long long>(kA), h.core.sr_out);

    Harness s;
    s.p[3] = kClass1StoreALimitBToY0;
    s.p[4] = kReadY0ToB;
    s.p[5] = 0x0af080; // JMP >$0005
    s.p[6] = 0x000005;
    s.debug_write(0x39, kInitialSr);
    s.debug_write(0x06, 0x13579b); // Y0 sentinel
    s.debug_write(0x0e, kA);       // positive limiting primary source
    s.debug_write(0x0f, kB);       // in-range independent source
    s.core.run = 1;

    const bool selector_setup = s.retire(kSetupR0);
    const bool selector_moved = s.retire(kClass1StoreALimitBToY0);
    const uint64_t selector_a_after = s.core.acc_a_out;
    const uint64_t selector_b_after = s.core.acc_b_out;
    const uint32_t selector_sr_after = s.core.sr_out;
    const bool selector_readback = s.retire(kReadY0ToB);
    const uint32_t selector_y0_word =
        (s.core.acc_b_out >> 24) & 0x00ffffff;
    const bool selector_limit_pass = selector_setup && selector_moved &&
        selector_readback && s.x_reads == 0 && s.x_writes == 1 &&
        s.last_x_addr == 0x1234 && s.last_x_data == 0x7fffff &&
        s.y_reads == 0 && s.y_writes == 0 &&
        s.p_reads == 0 && s.p_writes == 0 &&
        selector_y0_word == 0x2468ac && selector_a_after == kA &&
        selector_b_after == kB && selector_sr_after == 0x000075 &&
        s.core.sr_out == 0x000075;
    std::printf("%s X:R Class-I 1a2000 primary-limit: completed=%u/%u/%u "
                "X=%uR/%uW@%04x=%06x Y=%uR/%uW P=%uR/%uW "
                "Y0=%06x/2468ac A=%014llx/%014llx B=%014llx/%014llx "
                "SR=%06x/000075\n",
                selector_limit_pass ? "PASS" : "FAIL", selector_setup,
                selector_moved, selector_readback, s.x_reads, s.x_writes,
                s.last_x_addr, s.last_x_data, s.y_reads, s.y_writes,
                s.p_reads, s.p_writes, selector_y0_word,
                static_cast<unsigned long long>(selector_a_after),
                static_cast<unsigned long long>(kA),
                static_cast<unsigned long long>(selector_b_after),
                static_cast<unsigned long long>(kB), selector_sr_after);
    Harness l;
    l.p[3] = kClass1Load;
    l.p[4] = kReadX0ToB;
    l.p[5] = kReadY0ToB;
    l.p[6] = 0x0af080; // JMP >$0006
    l.p[7] = 0x000006;
    l.core.x_rdata = 0x314159;
    l.debug_write(0x39, kInitialSr);
    l.debug_write(0x04, 0xabcdef); // X0 sentinel
    l.debug_write(0x06, 0x13579b); // Y0 sentinel
    l.debug_write(0x0e, kA);
    l.debug_write(0x0f, kB);
    l.core.run = 1;

    const bool load_setup = l.retire(kSetupR0);
    const bool loaded = l.retire(kClass1Load);
    const bool x0_readback = l.retire(kReadX0ToB);
    const uint32_t x0_word = (l.core.acc_b_out >> 24) & 0x00ffffff;
    const bool y0_readback = l.retire(kReadY0ToB);
    const uint32_t load_y0_word = (l.core.acc_b_out >> 24) & 0x00ffffff;
    const bool load_pass = load_setup && loaded && x0_readback && y0_readback &&
        l.x_reads == 1 && l.x_writes == 0 &&
        l.last_x_read_addr == 0x1234 && x0_word == 0x314159 &&
        l.y_reads == 0 && l.y_writes == 0 &&
        l.p_reads == 0 && l.p_writes == 0 &&
        load_y0_word == 0x7fffff && l.core.acc_a_out == kA &&
        l.core.sr_out == (kInitialSr | 0x40);
    std::printf("%s X:R Class-I 10a000: completed=%u/%u/%u/%u "
                "X=%uR@%04x/%uW X0=%06x/314159 Y=%uR/%uW P=%uR/%uW "
                "Y0=%06x/7fffff A=%014llx/%014llx SR=%06x/000075\n",
                load_pass ? "PASS" : "FAIL", load_setup, loaded,
                x0_readback, y0_readback, l.x_reads, l.last_x_read_addr,
                l.x_writes, x0_word, l.y_reads, l.y_writes,
                l.p_reads, l.p_writes, load_y0_word,
                static_cast<unsigned long long>(l.core.acc_a_out),
                static_cast<unsigned long long>(kA), l.core.sr_out);

    const auto test_load_mode = [&](uint32_t opcode, bool setup_n,
                                    uint16_t expected_read_addr,
                                    uint16_t expected_final_r) {
        Harness m;
        unsigned instruction_index = 3;
        if (setup_n) {
            m.p[instruction_index++] = kSetupN0;
            // Address-register writes have a one-instruction pipeline delay.
            m.p[instruction_index++] = 0x000000;
        }
        m.p[instruction_index++] = opcode;
        m.p[instruction_index++] = kClass1Store;
        m.p[instruction_index++] = kReadX0ToB;
        m.p[instruction_index++] = kReadY0ToB;
        const unsigned loop_index = instruction_index;
        m.p[instruction_index++] = 0x0af080; // JMP >loop_index
        m.p[instruction_index++] = loop_index;
        m.core.x_rdata = 0x314159;
        m.debug_write(0x39, kInitialSr);
        m.debug_write(0x04, 0xabcdef); // X0 sentinel
        m.debug_write(0x06, 0x13579b); // Y0 sentinel
        m.debug_write(0x0e, kA);
        m.debug_write(0x0f, kB);
        m.core.run = 1;

        const bool setup_r_done = m.retire(kSetupR0);
        const bool setup_n_done = !setup_n || m.retire(kSetupN0);
        const bool loaded_mode = m.retire(opcode);
        const bool stored_after = m.retire(kClass1Store);
        const bool x0_mode_readback = m.retire(kReadX0ToB);
        const uint32_t x0_mode_word =
            (m.core.acc_b_out >> 24) & 0x00ffffff;
        const bool y0_mode_readback = m.retire(kReadY0ToB);
        const uint32_t y0_mode_word =
            (m.core.acc_b_out >> 24) & 0x00ffffff;
        const bool pass = setup_r_done && setup_n_done && loaded_mode &&
            stored_after && x0_mode_readback && y0_mode_readback &&
            m.x_reads == 1 && m.x_writes == 1 &&
            m.last_x_read_addr == expected_read_addr &&
            m.last_x_addr == expected_final_r && m.last_x_data == 0x314159 &&
            m.y_reads == 0 && m.y_writes == 0 &&
            m.p_reads == 0 && m.p_writes == 0 &&
            x0_mode_word == 0x314159 && y0_mode_word == 0x7fffff &&
            m.core.acc_a_out == kA && m.core.sr_out == (kInitialSr | 0x40);
        std::printf("%s X:R Class-I %06x->102000: completed=%u/%u/%u/%u/%u/%u "
                    "X=%uR@%04x/%uW@%04x=%06x Y=%uR/%uW P=%uR/%uW "
                    "X0=%06x/314159 Y0=%06x/7fffff "
                    "A=%014llx/%014llx SR=%06x/000075\n",
                    pass ? "PASS" : "FAIL", opcode, setup_r_done,
                    setup_n_done, loaded_mode, stored_after, x0_mode_readback,
                    y0_mode_readback, m.x_reads, m.last_x_read_addr,
                    m.x_writes, m.last_x_addr, m.last_x_data,
                    m.y_reads, m.y_writes, m.p_reads, m.p_writes,
                    x0_mode_word, y0_mode_word,
                    static_cast<unsigned long long>(m.core.acc_a_out),
                    static_cast<unsigned long long>(kA), m.core.sr_out);
        return pass;
    };
    const bool load_modes_pass =
        test_load_mode(kClass1LoadPostdecN, true, 0x1234, 0x1231) &&
        test_load_mode(kClass1LoadPostincN, true, 0x1234, 0x1237) &&
        test_load_mode(kClass1LoadPostdec, false, 0x1234, 0x1233) &&
        test_load_mode(kClass1LoadPostinc, false, 0x1234, 0x1235) &&
        test_load_mode(kClass1LoadIndexed, true, 0x1237, 0x1234) &&
        test_load_mode(kClass1LoadPredec, false, 0x1233, 0x1233);

    // DSP56001UMA2 A-131--A-134 assigns MMMRRR=110000 to a long absolute
    // X-memory address and 110100 to 24-bit immediate data.  Both consume the
    // following program word.  The immediate form has no X-memory transfer,
    // while the independent accumulator-to-Y transfer still occurs.
    const auto test_extension_form = [&](uint32_t opcode,
                                         uint32_t extension,
                                         bool load, bool immediate) {
        Harness m;
        m.p[3] = opcode;
        m.p[4] = extension;
        m.p[5] = kReadX0ToB;
        m.p[6] = kReadY0ToB;
        m.p[7] = 0x0af080; // JMP >$0007
        m.p[8] = 0x000007;
        m.core.x_rdata = 0x314159;
        m.debug_write(0x39, kInitialSr);
        m.debug_write(0x04, kX0);
        m.debug_write(0x06, 0x13579b);
        m.debug_write(0x0e, kA);
        m.debug_write(0x0f, kB);
        m.core.run = 1;

        const bool setup_done = m.retire(kSetupR0);
        const bool moved = m.retire(opcode);
        const uint64_t a_after = m.core.acc_a_out;
        const uint64_t b_after = m.core.acc_b_out;
        const uint32_t sr_after = m.core.sr_out;
        const bool primary_read = m.retire(kReadX0ToB);
        const uint32_t primary_word =
            (m.core.acc_b_out >> 24) & 0x00ffffff;
        const bool secondary_read = m.retire(kReadY0ToB);
        const uint32_t secondary_word =
            (m.core.acc_b_out >> 24) & 0x00ffffff;

        const uint32_t expected_primary = immediate ? extension :
                                                  (load ? 0x314159 : kX0);
        const bool pass = setup_done && moved && primary_read &&
            secondary_read &&
            m.x_reads == ((load && !immediate) ? 1U : 0U) &&
            m.x_writes == ((!load && !immediate) ? 1U : 0U) &&
            (!load || immediate || m.last_x_read_addr ==
                                      (extension & 0xffff)) &&
            (load || immediate || (m.last_x_addr == (extension & 0xffff) &&
                                    m.last_x_data == kX0)) &&
            m.y_reads == 0 && m.y_writes == 0 &&
            m.p_reads == 0 && m.p_writes == 0 &&
            primary_word == expected_primary &&
            secondary_word == 0x7fffff &&
            a_after == kA && b_after == kB &&
            sr_after == (kInitialSr | 0x40) &&
            m.core.sr_out == (kInitialSr | 0x40);
        std::printf("%s X:R Class-I extension %06x/%06x %s "
                    "completed=%u/%u/%u/%u X=%uR@%04x/%uW@%04x=%06x "
                    "primary=%06x/%06x secondary=%06x/7fffff "
                    "A=%014llx/%014llx B=%014llx/%014llx "
                    "SR=%06x/000075\n",
                    pass ? "PASS" : "FAIL", opcode, extension,
                    immediate ? "immediate" : (load ? "load" : "store"),
                    setup_done, moved, primary_read, secondary_read,
                    m.x_reads, m.last_x_read_addr, m.x_writes,
                    m.last_x_addr, m.last_x_data,
                    primary_word, expected_primary, secondary_word,
                    static_cast<unsigned long long>(a_after),
                    static_cast<unsigned long long>(kA),
                    static_cast<unsigned long long>(b_after),
                    static_cast<unsigned long long>(kB), sr_after);
        return pass;
    };
    bool extension_forms_pass = true;
    extension_forms_pass &=
        test_extension_form(kClass1StoreAbsolute, 0x005678, false, false);
    extension_forms_pass &=
        test_extension_form(kClass1LoadAbsolute, 0x005678, true, false);
    extension_forms_pass &=
        test_extension_form(kClass1LoadImmediate, 0x654321, true, true);

    Harness u;
    u.p[3] = kClass1StorePostinc;
    u.p[4] = kClass1Store;
    u.p[5] = kReadY0ToB;
    u.p[6] = 0x0af080; // JMP >$0006
    u.p[7] = 0x000006;
    u.debug_write(0x39, kInitialSr);
    u.debug_write(0x04, kX0);
    u.debug_write(0x06, 0x13579b); // Y0 sentinel
    u.debug_write(0x0e, kA);
    u.debug_write(0x0f, kB);
    u.core.run = 1;

    const bool update_setup = u.retire(kSetupR0);
    const bool updated = u.retire(kClass1StorePostinc);
    const bool direct_after_update = u.retire(kClass1Store);
    const bool update_readback = u.retire(kReadY0ToB);
    const uint32_t update_y0_word = (u.core.acc_b_out >> 24) & 0x00ffffff;
    const bool update_pass = update_setup && updated && direct_after_update &&
        update_readback && u.x_reads == 0 && u.x_writes == 2 &&
        u.first_x_addr == 0x1234 && u.last_x_addr == 0x1235 &&
        u.last_x_data == kX0 && u.y_reads == 0 && u.y_writes == 0 &&
        u.p_reads == 0 && u.p_writes == 0 &&
        update_y0_word == 0x7fffff && u.core.acc_a_out == kA &&
        u.core.sr_out == (kInitialSr | 0x40);
    std::printf("%s X:R Class-I 101800->102000: completed=%u/%u/%u/%u "
                "X=%uR/%uW@%04x->%04x=%06x Y=%uR/%uW P=%uR/%uW "
                "Y0=%06x/7fffff A=%014llx/%014llx SR=%06x/000075\n",
                update_pass ? "PASS" : "FAIL", update_setup, updated,
                direct_after_update, update_readback, u.x_reads, u.x_writes,
                u.first_x_addr, u.last_x_addr, u.last_x_data,
                u.y_reads, u.y_writes, u.p_reads, u.p_writes,
                update_y0_word,
                static_cast<unsigned long long>(u.core.acc_a_out),
                static_cast<unsigned long long>(kA), u.core.sr_out);

    Harness d;
    d.p[3] = kClass1StorePostdec;
    d.p[4] = kClass1Store;
    d.p[5] = kReadY0ToB;
    d.p[6] = 0x0af080; // JMP >$0006
    d.p[7] = 0x000006;
    d.debug_write(0x39, kInitialSr);
    d.debug_write(0x04, kX0);
    d.debug_write(0x06, 0x13579b); // Y0 sentinel
    d.debug_write(0x0e, kA);
    d.debug_write(0x0f, kB);
    d.core.run = 1;

    const bool decrement_setup = d.retire(kSetupR0);
    const bool decremented = d.retire(kClass1StorePostdec);
    const bool direct_after_decrement = d.retire(kClass1Store);
    const bool decrement_readback = d.retire(kReadY0ToB);
    const uint32_t decrement_y0_word =
        (d.core.acc_b_out >> 24) & 0x00ffffff;
    const bool decrement_pass = decrement_setup && decremented &&
        direct_after_decrement && decrement_readback &&
        d.x_reads == 0 && d.x_writes == 2 &&
        d.first_x_addr == 0x1234 && d.last_x_addr == 0x1233 &&
        d.last_x_data == kX0 && d.y_reads == 0 && d.y_writes == 0 &&
        d.p_reads == 0 && d.p_writes == 0 &&
        decrement_y0_word == 0x7fffff && d.core.acc_a_out == kA &&
        d.core.sr_out == (kInitialSr | 0x40);
    std::printf("%s X:R Class-I 101000->102000: completed=%u/%u/%u/%u "
                "X=%uR/%uW@%04x->%04x=%06x Y=%uR/%uW P=%uR/%uW "
                "Y0=%06x/7fffff A=%014llx/%014llx SR=%06x/000075\n",
                decrement_pass ? "PASS" : "FAIL", decrement_setup,
                decremented, direct_after_decrement, decrement_readback,
                d.x_reads, d.x_writes, d.first_x_addr, d.last_x_addr,
                d.last_x_data, d.y_reads, d.y_writes, d.p_reads, d.p_writes,
                decrement_y0_word,
                static_cast<unsigned long long>(d.core.acc_a_out),
                static_cast<unsigned long long>(kA), d.core.sr_out);

    Harness n;
    n.p[3] = kSetupN0;
    n.p[4] = 0x000000; // Address-register pipeline delay.
    n.p[5] = kClass1StorePostdecN;
    n.p[6] = kClass1Store;
    n.p[7] = kReadY0ToB;
    n.p[8] = 0x0af080; // JMP >$0008
    n.p[9] = 0x000008;
    n.debug_write(0x39, kInitialSr);
    n.debug_write(0x04, kX0);
    n.debug_write(0x06, 0x13579b); // Y0 sentinel
    n.debug_write(0x0e, kA);
    n.debug_write(0x0f, kB);
    n.core.run = 1;

    const bool offset_decrement_setup_r = n.retire(kSetupR0);
    const bool offset_decrement_setup_n = n.retire(kSetupN0);
    const bool offset_decremented = n.retire(kClass1StorePostdecN);
    const bool direct_after_offset_decrement = n.retire(kClass1Store);
    const bool offset_decrement_readback = n.retire(kReadY0ToB);
    const uint32_t offset_decrement_y0_word =
        (n.core.acc_b_out >> 24) & 0x00ffffff;
    const bool offset_decrement_pass = offset_decrement_setup_r &&
        offset_decrement_setup_n && offset_decremented &&
        direct_after_offset_decrement && offset_decrement_readback &&
        n.x_reads == 0 && n.x_writes == 2 &&
        n.first_x_addr == 0x1234 && n.last_x_addr == 0x1231 &&
        n.last_x_data == kX0 && n.y_reads == 0 && n.y_writes == 0 &&
        n.p_reads == 0 && n.p_writes == 0 &&
        offset_decrement_y0_word == 0x7fffff && n.core.acc_a_out == kA &&
        n.core.sr_out == (kInitialSr | 0x40);
    std::printf("%s X:R Class-I 100000->102000: completed=%u/%u/%u/%u/%u "
                "X=%uR/%uW@%04x->%04x=%06x Y=%uR/%uW P=%uR/%uW "
                "Y0=%06x/7fffff A=%014llx/%014llx SR=%06x/000075\n",
                offset_decrement_pass ? "PASS" : "FAIL",
                offset_decrement_setup_r, offset_decrement_setup_n,
                offset_decremented, direct_after_offset_decrement,
                offset_decrement_readback, n.x_reads, n.x_writes,
                n.first_x_addr, n.last_x_addr, n.last_x_data,
                n.y_reads, n.y_writes, n.p_reads, n.p_writes,
                offset_decrement_y0_word,
                static_cast<unsigned long long>(n.core.acc_a_out),
                static_cast<unsigned long long>(kA), n.core.sr_out);

    Harness a;
    a.p[3] = kSetupN0;
    a.p[4] = 0x000000; // Address-register pipeline delay.
    a.p[5] = kClass1StorePostincN;
    a.p[6] = kClass1Store;
    a.p[7] = kReadY0ToB;
    a.p[8] = 0x0af080; // JMP >$0008
    a.p[9] = 0x000008;
    a.debug_write(0x39, kInitialSr);
    a.debug_write(0x04, kX0);
    a.debug_write(0x06, 0x13579b); // Y0 sentinel
    a.debug_write(0x0e, kA);
    a.debug_write(0x0f, kB);
    a.core.run = 1;

    const bool offset_increment_setup_r = a.retire(kSetupR0);
    const bool offset_increment_setup_n = a.retire(kSetupN0);
    const bool offset_incremented = a.retire(kClass1StorePostincN);
    const bool direct_after_offset_increment = a.retire(kClass1Store);
    const bool offset_increment_readback = a.retire(kReadY0ToB);
    const uint32_t offset_increment_y0_word =
        (a.core.acc_b_out >> 24) & 0x00ffffff;
    const bool offset_increment_pass = offset_increment_setup_r &&
        offset_increment_setup_n && offset_incremented &&
        direct_after_offset_increment && offset_increment_readback &&
        a.x_reads == 0 && a.x_writes == 2 &&
        a.first_x_addr == 0x1234 && a.last_x_addr == 0x1237 &&
        a.last_x_data == kX0 && a.y_reads == 0 && a.y_writes == 0 &&
        a.p_reads == 0 && a.p_writes == 0 &&
        offset_increment_y0_word == 0x7fffff && a.core.acc_a_out == kA &&
        a.core.sr_out == (kInitialSr | 0x40);
    std::printf("%s X:R Class-I 100800->102000: completed=%u/%u/%u/%u/%u "
                "X=%uR/%uW@%04x->%04x=%06x Y=%uR/%uW P=%uR/%uW "
                "Y0=%06x/7fffff A=%014llx/%014llx SR=%06x/000075\n",
                offset_increment_pass ? "PASS" : "FAIL",
                offset_increment_setup_r, offset_increment_setup_n,
                offset_incremented, direct_after_offset_increment,
                offset_increment_readback, a.x_reads, a.x_writes,
                a.first_x_addr, a.last_x_addr, a.last_x_data,
                a.y_reads, a.y_writes, a.p_reads, a.p_writes,
                offset_increment_y0_word,
                static_cast<unsigned long long>(a.core.acc_a_out),
                static_cast<unsigned long long>(kA), a.core.sr_out);

    Harness p;
    p.p[3] = kClass1StorePredec;
    p.p[4] = kClass1Store;
    p.p[5] = kReadY0ToB;
    p.p[6] = 0x0af080; // JMP >$0006
    p.p[7] = 0x000006;
    p.debug_write(0x39, kInitialSr);
    p.debug_write(0x04, kX0);
    p.debug_write(0x06, 0x13579b); // Y0 sentinel
    p.debug_write(0x0e, kA);
    p.debug_write(0x0f, kB);
    p.core.run = 1;

    const bool predecrement_setup = p.retire(kSetupR0);
    const bool predecremented = p.retire(kClass1StorePredec);
    const bool direct_after_predecrement = p.retire(kClass1Store);
    const bool predecrement_readback = p.retire(kReadY0ToB);
    const uint32_t predecrement_y0_word =
        (p.core.acc_b_out >> 24) & 0x00ffffff;
    const bool predecrement_pass = predecrement_setup && predecremented &&
        direct_after_predecrement && predecrement_readback &&
        p.x_reads == 0 && p.x_writes == 2 &&
        p.first_x_addr == 0x1233 && p.last_x_addr == 0x1233 &&
        p.last_x_data == kX0 && p.y_reads == 0 && p.y_writes == 0 &&
        p.p_reads == 0 && p.p_writes == 0 &&
        predecrement_y0_word == 0x7fffff && p.core.acc_a_out == kA &&
        p.core.sr_out == (kInitialSr | 0x40);
    std::printf("%s X:R Class-I 103800->102000: completed=%u/%u/%u/%u "
                "X=%uR/%uW@%04x->%04x=%06x Y=%uR/%uW P=%uR/%uW "
                "Y0=%06x/7fffff A=%014llx/%014llx SR=%06x/000075\n",
                predecrement_pass ? "PASS" : "FAIL", predecrement_setup,
                predecremented, direct_after_predecrement,
                predecrement_readback, p.x_reads, p.x_writes,
                p.first_x_addr, p.last_x_addr, p.last_x_data,
                p.y_reads, p.y_writes, p.p_reads, p.p_writes,
                predecrement_y0_word,
                static_cast<unsigned long long>(p.core.acc_a_out),
                static_cast<unsigned long long>(kA), p.core.sr_out);

    Harness i;
    i.p[3] = kSetupN0;
    i.p[4] = 0x000000; // Address-register pipeline delay.
    i.p[5] = kClass1StoreIndexed;
    i.p[6] = kClass1Store;
    i.p[7] = kReadY0ToB;
    i.p[8] = 0x0af080; // JMP >$0008
    i.p[9] = 0x000008;
    i.debug_write(0x39, kInitialSr);
    i.debug_write(0x04, kX0);
    i.debug_write(0x06, 0x13579b); // Y0 sentinel
    i.debug_write(0x0e, kA);
    i.debug_write(0x0f, kB);
    i.core.run = 1;

    const bool indexed_setup_r = i.retire(kSetupR0);
    const bool indexed_setup_n = i.retire(kSetupN0);
    const bool indexed = i.retire(kClass1StoreIndexed);
    const bool direct_after_indexed = i.retire(kClass1Store);
    const bool indexed_readback = i.retire(kReadY0ToB);
    const uint32_t indexed_y0_word = (i.core.acc_b_out >> 24) & 0x00ffffff;
    const bool indexed_pass = indexed_setup_r && indexed_setup_n && indexed &&
        direct_after_indexed && indexed_readback &&
        i.x_reads == 0 && i.x_writes == 2 &&
        i.first_x_addr == 0x1237 && i.last_x_addr == 0x1234 &&
        i.last_x_data == kX0 && i.y_reads == 0 && i.y_writes == 0 &&
        i.p_reads == 0 && i.p_writes == 0 &&
        indexed_y0_word == 0x7fffff && i.core.acc_a_out == kA &&
        i.core.sr_out == (kInitialSr | 0x40);
    std::printf("%s X:R Class-I 102800->102000: completed=%u/%u/%u/%u/%u "
                "X=%uR/%uW@%04x->%04x=%06x Y=%uR/%uW P=%uR/%uW "
                "Y0=%06x/7fffff A=%014llx/%014llx SR=%06x/000075\n",
                indexed_pass ? "PASS" : "FAIL", indexed_setup_r,
                indexed_setup_n, indexed, direct_after_indexed,
                indexed_readback, i.x_reads, i.x_writes,
                i.first_x_addr, i.last_x_addr, i.last_x_data,
                i.y_reads, i.y_writes, i.p_reads, i.p_writes,
                indexed_y0_word,
                static_cast<unsigned long long>(i.core.acc_a_out),
                static_cast<unsigned long long>(kA), i.core.sr_out);

    // DSP56001UMA2 A-131--A-134 defines four primary S1 selections
    // (X0/X1/A/B) and four independent S2 selections (A/B to Y0/Y1).  Sweep
    // every combination in both X-memory directions.  The load cases in
    // which S1 and S2 name the same accumulator deliberately require S2 to
    // observe the pre-load value: both parallel sources are sampled before
    // writeback.
    constexpr std::array<uint8_t, 4> kPrimaryCodes = {
        0x04, 0x05, 0x0e, 0x0f
    };
    constexpr std::array<uint32_t, 4> kPrimaryWords = {
        0x102030, 0x405060, 0x112233, 0x8899aa
    };
    constexpr uint64_t kMatrixA = 0x00112233445566ULL;
    constexpr uint64_t kMatrixB = 0xff8899aabbccddULL;
    constexpr uint32_t kMatrixMemory = 0x314159;
    constexpr uint32_t kMatrixSr = 0x000035;

    const auto test_selector = [&](bool load, unsigned primary,
                                   unsigned secondary) {
        Harness m;
        const uint32_t opcode = (load ? kClass1Load : kClass1Store) |
            (primary << 18) | ((secondary & 2U) << 16) |
            ((secondary & 1U) << 16);
        const uint8_t primary_code = kPrimaryCodes[primary];
        const uint8_t y_code = (secondary & 1U) ? 0x07 : 0x06;
        const uint32_t secondary_word = (secondary & 2U) ?
                                        kPrimaryWords[3] : kPrimaryWords[2];
        m.p[3] = opcode;
        m.p[4] = register_to_b(primary_code);
        m.p[5] = register_to_b(y_code);
        m.p[6] = 0x0af080; // JMP >$0006
        m.p[7] = 0x000006;
        m.core.x_rdata = kMatrixMemory;
        m.debug_write(0x39, kMatrixSr);
        m.debug_write(0x04, kPrimaryWords[0]);
        m.debug_write(0x05, kPrimaryWords[1]);
        m.debug_write(0x06, 0x0a0b0c);
        m.debug_write(0x07, 0x0d0e0f);
        m.debug_write(0x0e, kMatrixA);
        m.debug_write(0x0f, kMatrixB);
        m.core.run = 1;

        const bool setup_done = m.retire(kSetupR0);
        const bool moved_selector = m.retire(opcode);
        const uint64_t a_after = m.core.acc_a_out;
        const uint64_t b_after = m.core.acc_b_out;
        const uint32_t sr_after = m.core.sr_out;
        const bool primary_read = m.retire(m.p[4]);
        const uint32_t primary_word =
            (m.core.acc_b_out >> 24) & 0x00ffffff;
        const bool secondary_read = m.retire(m.p[5]);
        const uint32_t y_word =
            (m.core.acc_b_out >> 24) & 0x00ffffff;

        uint64_t expected_a = kMatrixA;
        uint64_t expected_b = kMatrixB;
        if (load && primary == 2)
            expected_a = 0x00314159000000ULL;
        if (load && primary == 3)
            expected_b = 0x00314159000000ULL;
        const uint32_t expected_primary = load ? kMatrixMemory :
                                                kPrimaryWords[primary];
        const bool pass = setup_done && moved_selector && primary_read &&
            secondary_read && m.x_reads == (load ? 1U : 0U) &&
            m.x_writes == (load ? 0U : 1U) &&
            m.last_x_read_addr == (load ? 0x1234 : 0) &&
            (!load ? (m.last_x_addr == 0x1234 &&
                      m.last_x_data == kPrimaryWords[primary]) : true) &&
            m.y_reads == 0 && m.y_writes == 0 &&
            m.p_reads == 0 && m.p_writes == 0 &&
            primary_word == expected_primary && y_word == secondary_word &&
            a_after == expected_a && b_after == expected_b &&
            sr_after == kMatrixSr && m.core.sr_out == kMatrixSr;
        std::printf("%s X:R Class-I selector %06x %s "
                    "S1=%u/%02x S2=%u->Y%u "
                    "X=%uR/%uW@%04x=%06x primary=%06x/%06x "
                    "secondary=%06x/%06x A=%014llx/%014llx "
                    "B=%014llx/%014llx SR=%06x/%06x\n",
                    pass ? "PASS" : "FAIL", opcode,
                    load ? "load" : "store", primary, primary_code,
                    secondary >> 1, secondary & 1U, m.x_reads, m.x_writes,
                    load ? m.last_x_read_addr : m.last_x_addr,
                    m.last_x_data, primary_word, expected_primary,
                    y_word, secondary_word,
                    static_cast<unsigned long long>(a_after),
                    static_cast<unsigned long long>(expected_a),
                    static_cast<unsigned long long>(b_after),
                    static_cast<unsigned long long>(expected_b),
                    sr_after, kMatrixSr);
        return pass;
    };

    bool selector_matrix_pass = true;
    for (unsigned load = 0; load < 2; ++load) {
        for (unsigned primary = 0; primary < 4; ++primary) {
            for (unsigned secondary = 0; secondary < 4; ++secondary)
                selector_matrix_pass &=
                    test_selector(load != 0, primary, secondary);
        }
    }

    return store_pass && selector_limit_pass && load_pass && load_modes_pass &&
           update_pass &&
           decrement_pass &&
           offset_decrement_pass && offset_increment_pass &&
           predecrement_pass && indexed_pass && extension_forms_pass &&
           selector_matrix_pass ? 0 : 1;
}
