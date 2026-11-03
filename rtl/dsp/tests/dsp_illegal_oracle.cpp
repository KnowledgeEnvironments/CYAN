// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 illegal-instruction exception boundary oracle
//
// The architectural ILLEGAL opcode is $000005.  Motorola specifies that it
// executes as a NOP and then raises the nonmaskable IPL-3 exception through
// P:$003e/$003f.  A long-vector JSR stacks the address of the illegal
// instruction, so the deliberately-invalid RTI in this oracle must return to
// $0000 and take the exception again rather than retire the successor.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint16_t kIllegalVector = 0x003e;
constexpr uint16_t kHandler = 0x0060;
constexpr uint32_t kInitialSr = 0x000055;

class IllegalHarness {
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
    bool handler_mask_was_ipl3 = false;
    uint16_t successor_addr = 0x0001;

    explicit IllegalHarness(bool repeated = false,
                            uint32_t illegal_opcode = 0x000005) {
        if (repeated) {
            p[0x00] = 0x0603a0; // REP #3.
            p[0x01] = illegal_opcode; // Repeated III acts as repeated NOP.
            p[0x02] = 0x0af080; // JMP >$0002 after deferred exception.
            p[0x03] = 0x000002;
            successor_addr = 0x0002;
        } else {
            p[0x00] = illegal_opcode;
            p[0x01] = 0x0af080; // JMP >$0001 if ILLEGAL is wrongly retired.
            p[0x02] = 0x000001;
        }
        p[0x08] = 0x0bf080; // IRQA long vector: JSR >$0070.
        p[0x09] = 0x000070;
        p[kIllegalVector] = 0x0bf080; // ILLEGAL long vector: JSR >$0060.
        p[kIllegalVector + 1] = kHandler;
        p[kHandler] = 0x000004; // RTI only to prove the stacked fault PC.
        p[0x70] = 0x000004;

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

        if (fetch_en) {
            switch (fetch_addr) {
            case kIllegalVector:
                if (first_vector == 0xffff)
                    first_vector = fetch_addr;
                ++vector_first_fetches;
                break;
            case kIllegalVector + 1:
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
            default:
                if (fetch_addr == successor_addr)
                    ++successor_fetches;
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
        }
    }

    void run_cycles(unsigned count) {
        for (unsigned i = 0; i < count; ++i)
            tick();
    }
};

bool check_reserved_encoding(const char *name, uint32_t opcode) {
    IllegalHarness h(false, opcode);
    h.run_cycles(140);
    const bool vectors = h.first_vector == kIllegalVector &&
                         h.vector_first_fetches >= 2 &&
                         h.vector_first_fetches == h.vector_second_fetches &&
                         h.handler_fetches >= 2 &&
                         h.successor_fetches == 0;
    std::printf("%s %s III boundary: opcode=$%06x first=$%04x "
                "vector=%u/%u handler=%u successor-fetch=%u\n",
                vectors ? "PASS" : "FAIL", name, opcode, h.first_vector,
                h.vector_first_fetches, h.vector_second_fetches,
                h.handler_fetches, h.successor_fetches);
    return vectors;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);

    IllegalHarness ordinary;
    ordinary.run_cycles(140);
    const bool long_vector = ordinary.first_vector == kIllegalVector &&
                             ordinary.vector_first_fetches >= 2 &&
                             ordinary.vector_first_fetches ==
                                 ordinary.vector_second_fetches &&
                             ordinary.handler_fetches >= 2 &&
                             ordinary.handler_mask_was_ipl3;
    std::printf("%s ILLEGAL long-vector boundary: first=$%04x vector=%u/%u "
                "handler=%u successor-fetch=%u mask3=%u\n",
                long_vector ? "PASS" : "FAIL", ordinary.first_vector,
                ordinary.vector_first_fetches,
                ordinary.vector_second_fetches, ordinary.handler_fetches,
                ordinary.successor_fetches, ordinary.handler_mask_was_ipl3);

    // ILLEGAL is above every maskable source.  A coincident level-mode IRQA
    // remains pending and can only win after the fatal handler's test-only RTI.
    IllegalHarness coincident;
    coincident.core.irq_priority = 0x03;
    for (unsigned i = 0; i < 40 &&
         !(coincident.core.state_out == 4 &&
           coincident.core.cur_inst_out == 0x000005); ++i)
        coincident.tick();
    coincident.core.irq_a = 1;
    coincident.run_cycles(180);
    const bool priority = coincident.first_vector == kIllegalVector &&
                          coincident.vector_first_fetches != 0;
    std::printf("%s coincident ILLEGAL/IRQA priority: first=$%04x "
                "illegal=%u/%u irqa=%u/%u\n", priority ? "PASS" : "FAIL",
                coincident.first_vector, coincident.vector_first_fetches,
                coincident.vector_second_fetches,
                coincident.irqa_vector_fetches,
                coincident.irqa_second_fetches);

    // REP is uninterruptible.  Its ILLEGAL follower executes as a repeated
    // NOP; only after all repetitions does III vector, and a test-only RTI
    // resumes two words after the illegal instruction at the successor.
    IllegalHarness repeated(true);
    repeated.run_cycles(180);
    const bool rep_deferred = repeated.first_vector == kIllegalVector &&
                              repeated.vector_first_fetches == 1 &&
                              repeated.vector_second_fetches == 1 &&
                              repeated.handler_fetches == 1 &&
                              repeated.successor_fetches != 0;
    std::printf("%s REP/ILLEGAL deferred boundary: first=$%04x "
                "vector=%u/%u handler=%u successor-fetch=%u\n",
                rep_deferred ? "PASS" : "FAIL", repeated.first_vector,
                repeated.vector_first_fetches,
                repeated.vector_second_fetches, repeated.handler_fetches,
                repeated.successor_fetches);

    // Undefined instruction encodings enter the same III exception as the
    // explicit $000005 ILLEGAL opcode rather than retiring as accidental NOPs.
    IllegalHarness undefined(false, 0x000007);
    undefined.run_cycles(140);
    const bool undefined_vectors = undefined.first_vector == kIllegalVector &&
                                   undefined.vector_first_fetches >= 2 &&
                                   undefined.vector_first_fetches ==
                                       undefined.vector_second_fetches &&
                                   undefined.handler_fetches >= 2 &&
                                   undefined.successor_fetches == 0;
    std::printf("%s undefined-opcode III boundary: first=$%04x vector=%u/%u "
                "handler=%u successor-fetch=%u\n",
                undefined_vectors ? "PASS" : "FAIL", undefined.first_vector,
                undefined.vector_first_fetches,
                undefined.vector_second_fetches, undefined.handler_fetches,
                undefined.successor_fetches);

    // DIV fixes bits 2:0 at zero.  A low-bit neighbor must take III rather
    // than enter an over-broad DIV decode and retire as an ALU instruction.
    IllegalHarness div_neighbor(false, 0x018041);
    div_neighbor.run_cycles(140);
    const bool div_neighbor_vectors =
        div_neighbor.first_vector == kIllegalVector &&
        div_neighbor.vector_first_fetches >= 2 &&
        div_neighbor.vector_first_fetches ==
            div_neighbor.vector_second_fetches &&
        div_neighbor.handler_fetches >= 2 &&
        div_neighbor.successor_fetches == 0;
    std::printf("%s reserved-DIV-neighbor III boundary: first=$%04x "
                "vector=%u/%u handler=%u successor-fetch=%u\n",
                div_neighbor_vectors ? "PASS" : "FAIL",
                div_neighbor.first_vector, div_neighbor.vector_first_fetches,
                div_neighbor.vector_second_fetches,
                div_neighbor.handler_fetches,
                div_neighbor.successor_fetches);

    // ANDI/ORI encode the control-register destination in EE.  The manual
    // defines 00=MR, 01=CCR, and 10=OMR; EE=11 is reserved and must take the
    // same III path as every other undefined instruction encoding.
    IllegalHarness andi_reserved(false, 0x005abb); // ANDI #$5a,reserved-EE=3
    andi_reserved.run_cycles(140);
    const bool andi_reserved_vectors =
        andi_reserved.first_vector == kIllegalVector &&
        andi_reserved.vector_first_fetches >= 2 &&
        andi_reserved.vector_first_fetches ==
            andi_reserved.vector_second_fetches &&
        andi_reserved.handler_fetches >= 2 &&
        andi_reserved.successor_fetches == 0;
    std::printf("%s reserved-ANDI-destination III boundary: first=$%04x "
                "vector=%u/%u handler=%u successor-fetch=%u\n",
                andi_reserved_vectors ? "PASS" : "FAIL",
                andi_reserved.first_vector,
                andi_reserved.vector_first_fetches,
                andi_reserved.vector_second_fetches,
                andi_reserved.handler_fetches,
                andi_reserved.successor_fetches);

    IllegalHarness ori_reserved(false, 0x00a5fb); // ORI #$a5,reserved-EE=3
    ori_reserved.run_cycles(140);
    const bool ori_reserved_vectors =
        ori_reserved.first_vector == kIllegalVector &&
        ori_reserved.vector_first_fetches >= 2 &&
        ori_reserved.vector_first_fetches ==
            ori_reserved.vector_second_fetches &&
        ori_reserved.handler_fetches >= 2 &&
        ori_reserved.successor_fetches == 0;
    std::printf("%s reserved-ORI-destination III boundary: first=$%04x "
                "vector=%u/%u handler=%u successor-fetch=%u\n",
                ori_reserved_vectors ? "PASS" : "FAIL",
                ori_reserved.first_vector, ori_reserved.vector_first_fetches,
                ori_reserved.vector_second_fetches,
                ori_reserved.handler_fetches,
                ori_reserved.successor_fetches);

    // 101100EE is not an ANDI alias.  It was accepted by the historical
    // decoder because the product and its oracle shared the same transcribed
    // opcode error.  All three apparent destinations must take III.
    const bool false_andi_mr = check_reserved_encoding(
        "reserved-false-ANDI-MR", 0x005ab0);
    const bool false_andi_ccr = check_reserved_encoding(
        "reserved-false-ANDI-CCR", 0x005ab1);
    const bool false_andi_omr = check_reserved_encoding(
        "reserved-false-ANDI-OMR", 0x005ab2);

    // The JMP/JSR/Jcc/JScc effective-address formats fix their unused low
    // bits, and MMM=110 admits only RRR=000 for the absolute extension-word
    // form.  The old broad decoder treated every neighbor below as a real
    // branch instead of taking III.
    const bool jmp_low_reserved = check_reserved_encoding(
        "reserved-JMP-low-field", 0x0af081);
    const bool jsr_low_reserved = check_reserved_encoding(
        "reserved-JSR-low-field", 0x0bf081);
    const bool jcc_low_reserved = check_reserved_encoding(
        "reserved-Jcc-low-field", 0x0af0b0);
    const bool jscc_low_reserved = check_reserved_encoding(
        "reserved-JScc-low-field", 0x0bf0b0);
    const bool jmp_abs_reserved = check_reserved_encoding(
        "reserved-JMP-absolute-RRR", 0x0af180);
    const bool jsr_abs_reserved = check_reserved_encoding(
        "reserved-JSR-absolute-RRR", 0x0bf180);
    const bool jcc_abs_reserved = check_reserved_encoding(
        "reserved-Jcc-absolute-RRR", 0x0af1a0);
    const bool jscc_abs_reserved = check_reserved_encoding(
        "reserved-JScc-absolute-RRR", 0x0bf1a0);

    // DO and REP admit only the documented source-register map, and their
    // effective-address forms omit MMM=110 entirely.  These encodings match
    // the outer opcode shape but name no instruction; they must not retire as
    // silent NOPs or consume the following word as a DO extension.
    const bool do_source_reserved = check_reserved_encoding(
        "reserved-DO-source", 0x06c000);
    const bool rep_source_reserved = check_reserved_encoding(
        "reserved-REP-source", 0x06c020);
    const bool do_ea_reserved = check_reserved_encoding(
        "reserved-DO-effective-address", 0x067000);
    const bool rep_ea_reserved = check_reserved_encoding(
        "reserved-REP-effective-address", 0x067020);

    // Motorola tables A-29/A-30 reserve five nonmultiply ALU bytes.  Pair
    // them with the architectural parallel-move NOP field so the complete
    // instruction must enter III rather than retire as a plain MOVE/NOP.
    const bool parallel_move_d1_reserved = check_reserved_encoding(
        "reserved-parallel-MOVE-D1", 0x200008);
    const bool parallel_j0_k4_d0_reserved = check_reserved_encoding(
        "reserved-parallel-J0-K4-D0", 0x200004);
    const bool parallel_j0_k4_d1_reserved = check_reserved_encoding(
        "reserved-parallel-J0-K4-D1", 0x20000c);
    const bool parallel_j1_k5_d0_reserved = check_reserved_encoding(
        "reserved-parallel-J1-K5-D0", 0x200015);
    const bool parallel_j1_k5_d1_reserved = check_reserved_encoding(
        "reserved-parallel-J1-K5-D1", 0x20001d);

    // DSP56001UMA3 section A.9.3 reserves bbbbb=11bbb for every bit-class
    // instruction.  Exercise the first reserved bit number (#24) in all
    // eight opcode families.  These must take III rather than aliasing bits
    // 24--31 of a register or memory operand.
    const bool bclr_bit24_reserved = check_reserved_encoding(
        "reserved-BCLR-bit24", 0x0ac458);
    const bool bset_bit24_reserved = check_reserved_encoding(
        "reserved-BSET-bit24", 0x0ac478);
    const bool bchg_bit24_reserved = check_reserved_encoding(
        "reserved-BCHG-bit24", 0x0bc458);
    const bool btst_bit24_reserved = check_reserved_encoding(
        "reserved-BTST-bit24", 0x0bc478);
    const bool jclr_bit24_reserved = check_reserved_encoding(
        "reserved-JCLR-bit24", 0x0a0098);
    const bool jset_bit24_reserved = check_reserved_encoding(
        "reserved-JSET-bit24", 0x0a00b8);
    const bool jsclr_bit24_reserved = check_reserved_encoding(
        "reserved-JSCLR-bit24", 0x0b0098);
    const bool jsset_bit24_reserved = check_reserved_encoding(
        "reserved-JSSET-bit24", 0x0b00b8);

    // DSP56001UMA1 pp. A-81--A-82 define JCLR's effective-address source as
    // exactly the seven address-register-indirect modes 000, 001, 010, 011,
    // 100, 101, and 111.  JSET/JSCLR/JSSET share that encoding.  MMM=110 is
    // absent because every Jbit instruction is already exactly two program
    // words and its sole extension word is the absolute jump target; there
    // is no second word available for a long-absolute source address.  Check
    // both memory spaces, all four Jbit operations, and every RRR neighbor.
    bool jbit_mmm110_reserved = true;
    for (const uint32_t call_family :
         {uint32_t{0x0a0000}, uint32_t{0x0b0000}}) {
        for (const uint32_t set_variant :
             {uint32_t{0}, uint32_t{0x000020}}) {
            for (const uint32_t memory_space :
                 {uint32_t{0}, uint32_t{0x000040}}) {
                for (unsigned rrr = 0; rrr < 8; ++rrr) {
                    char name[80];
                    std::snprintf(
                        name, sizeof(name), "reserved-%s%s-%c-MMM110-RRR%u",
                        call_family == 0x0b0000 ? "JS" : "J",
                        set_variant ? "SET" : "CLR",
                        memory_space ? 'Y' : 'X', rrr);
                    jbit_mmm110_reserved &= check_reserved_encoding(
                        name, call_family | 0x007080 | set_variant |
                              memory_space | (rrr << 8));
                }
            }
        }
    }

    // The Class-II X:R/R:Y format omits MMM=110 entirely.  The four
    // accumulator/space combinations must all take III rather than degrade
    // into a parallel ALU instruction with a suppressed move.
    const bool class2_ax_mmm110_reserved = check_reserved_encoding(
        "reserved-Class-II-A-X-MMM110", 0x083000);
    const bool class2_bx_mmm110_reserved = check_reserved_encoding(
        "reserved-Class-II-B-X-MMM110", 0x093000);
    const bool class2_ay_mmm110_reserved = check_reserved_encoding(
        "reserved-Class-II-A-Y-MMM110", 0x08b000);
    const bool class2_by_mmm110_reserved = check_reserved_encoding(
        "reserved-Class-II-B-Y-MMM110", 0x09b000);

    // In Class-I, MMM=110 admits RRR=000 as long absolute and, for reads,
    // RRR=100 as immediate data.  Check every other RRR value for both
    // directions and both X:R/R:Y spaces.
    bool class1_reserved = true;
    constexpr std::array<unsigned, 7> store_reserved_rrr{{1, 2, 3, 4, 5, 6, 7}};
    constexpr std::array<unsigned, 6> load_reserved_rrr{{1, 2, 3, 5, 6, 7}};
    for (const uint32_t space : {uint32_t{0}, uint32_t{0x004000}}) {
        for (const unsigned rrr : store_reserved_rrr) {
            char name[64];
            std::snprintf(name, sizeof(name),
                          "reserved-Class-I-%s-store-RRR%u",
                          space ? "R-Y" : "X-R", rrr);
            class1_reserved &= check_reserved_encoding(
                name, 0x103000 | space | (rrr << 8));
        }
        for (const unsigned rrr : load_reserved_rrr) {
            char name[64];
            std::snprintf(name, sizeof(name),
                          "reserved-Class-I-%s-load-RRR%u",
                          space ? "R-Y" : "X-R", rrr);
            class1_reserved &= check_reserved_encoding(
                name, 0x10b000 | space | (rrr << 8));
        }
    }

    // Table A-20 reserves five-bit register codes 0--3.  The dedicated
    // parallel-move NOP/address-update encodings remain legal, but an R:R
    // transfer naming one of these codes must take III.
    bool register_move_reserved = true;
    // Source code 2 is the separately defined U (address-update) format.
    // Source code 0 also contains the exact all-zero parallel-move NOP, but
    // pairing it with legal destination X0 below remains a reserved R:R word.
    constexpr std::array<unsigned, 3> source_reserved_codes{{0, 1, 3}};
    for (const unsigned code : source_reserved_codes) {
        char source_name[64];
        std::snprintf(source_name, sizeof(source_name),
                      "reserved-register-MOVE-source-%u", code);
        register_move_reserved &= check_reserved_encoding(
            source_name, 0x200400 | (code << 13));
    }

    for (unsigned code = 0; code < 4; ++code) {
        char destination_name[64];
        std::snprintf(destination_name, sizeof(destination_name),
                      "reserved-register-MOVE-destination-%u", code);
        register_move_reserved &= check_reserved_encoding(
            destination_name, 0x208000 | (code << 8));
    }

    // Ordinary X/Y memory moves use MMMRRR=110000 for long absolute data and
    // additionally admit 110100 only for memory-to-register immediate data.
    // Long L: moves admit only 110000.  Every other selector is reserved and
    // must not consume the following program word as an extension.
    bool memory_move_reserved = true;
    constexpr std::array<unsigned, 7> memory_store_reserved_rrr{{1, 2, 3, 4, 5, 6, 7}};
    constexpr std::array<unsigned, 6> memory_load_reserved_rrr{{1, 2, 3, 5, 6, 7}};
    for (const uint32_t space : {uint32_t{0}, uint32_t{0x080000}}) {
        for (const unsigned rrr : memory_store_reserved_rrr) {
            char name[64];
            std::snprintf(name, sizeof(name),
                          "reserved-%s-memory-store-RRR%u",
                          space ? "Y" : "X", rrr);
            memory_move_reserved &= check_reserved_encoding(
                name, 0x447000 | space | (rrr << 8));
        }
        for (const unsigned rrr : memory_load_reserved_rrr) {
            char name[64];
            std::snprintf(name, sizeof(name),
                          "reserved-%s-memory-load-RRR%u",
                          space ? "Y" : "X", rrr);
            memory_move_reserved &= check_reserved_encoding(
                name, 0x44f000 | space | (rrr << 8));
        }
    }

    bool long_move_reserved = true;
    for (unsigned rrr = 1; rrr < 8; ++rrr) {
        char store_name[64];
        std::snprintf(store_name, sizeof(store_name),
                      "reserved-L-memory-store-RRR%u", rrr);
        long_move_reserved &= check_reserved_encoding(
            store_name, 0x427000 | (rrr << 8));

        char load_name[64];
        std::snprintf(load_name, sizeof(load_name),
                      "reserved-L-memory-load-RRR%u", rrr);
        long_move_reserved &= check_reserved_encoding(
            load_name, 0x42f000 | (rrr << 8));
    }

    // DSP56001UMA2 pp. A-155--A-158 define exactly three $05 MOVEC
    // subformats in bits 7:5: X/Y memory (0s1), register (101), and short
    // immediate (101).  The memory and immediate forms select only M0--M7
    // or SR--LC in their five-bit control-register field.  The separate
    // $04 register-to-register form fixes bits 7:6 at 10.  W selects which
    // six-bit field uses the control-register subset and which uses the full
    // architectural register map.  Exercise every reserved selector instead of
    // allowing the broad opcode-family decode to retire it as a move.
    bool movec_reserved = true;
    constexpr std::array<unsigned, 5> movec_reserved_forms{{0, 2, 4, 6, 7}};
    for (const unsigned form : movec_reserved_forms) {
        char name[64];
        std::snprintf(name, sizeof(name),
                      "reserved-MOVEC-$05-form-%u", form);
        movec_reserved &= check_reserved_encoding(
            name, 0x050000 | (form << 5));
    }

    for (unsigned code = 8; code <= 24; ++code) {
        char name[64];
        std::snprintf(name, sizeof(name),
                      "reserved-MOVEC-control-%u", code);
        movec_reserved &= check_reserved_encoding(
            name, 0x0500a0 | code);
    }

    // MMM=110 admits only RRR=000 (long absolute), plus RRR=100 for the
    // memory-to-control long-immediate form.  A control-to-memory move has
    // no immediate-data encoding.
    for (unsigned rrr = 1; rrr < 8; ++rrr) {
        char name[64];
        std::snprintf(name, sizeof(name),
                      "reserved-MOVEC-memory-write-RRR%u", rrr);
        movec_reserved &= check_reserved_encoding(
            name, 0x057020 | (rrr << 8));
        if (rrr != 4) {
            std::snprintf(name, sizeof(name),
                          "reserved-MOVEC-memory-read-RRR%u", rrr);
            movec_reserved &= check_reserved_encoding(
                name, 0x05f020 | (rrr << 8));
        }
    }

    for (unsigned code = 0; code < 64; ++code) {
        if ((code >= 32 && code <= 39) || code >= 57)
            continue;
        char name[64];
        std::snprintf(name, sizeof(name),
                      "reserved-MOVEC-register-W0-control-%u", code);
        movec_reserved &= check_reserved_encoding(
            name, 0x044080 | code);
    }

    for (unsigned code = 0; code < 64; ++code) {
        if ((code >= 4 && code <= 39) || code >= 57)
            continue;
        char name[72];
        std::snprintf(name, sizeof(name),
                      "reserved-MOVEC-register-W0-general-%u", code);
        movec_reserved &= check_reserved_encoding(
            name, 0x0440a0 | (code << 8));
        std::snprintf(name, sizeof(name),
                      "reserved-MOVEC-register-W1-general-%u", code);
        movec_reserved &= check_reserved_encoding(
            name, 0x04f980 | code);
    }

    for (unsigned code = 0; code < 64; ++code) {
        if ((code >= 32 && code <= 39) || code >= 57)
            continue;
        char name[72];
        std::snprintf(name, sizeof(name),
                      "reserved-MOVEC-register-W1-control-%u", code);
        movec_reserved &= check_reserved_encoding(
            name, 0x04c08e | (code << 8));
    }

    // DSP56001UMA2 pp. A-161--A-163 define only two MOVEM layouts: short
    // absolute has bit 14=0 and bits 7:6=00; effective-address has bit 14=1
    // and bits 7:6=10.  Both use the architectural six-bit register map, and
    // the EA layout admits only RRR=000 when MMM=110.
    bool movem_reserved = true;
    for (unsigned ea = 0; ea < 2; ++ea) {
        for (unsigned form = 0; form < 4; ++form) {
            if ((!ea && form == 0) || (ea && form == 2))
                continue;
            char name[64];
            std::snprintf(name, sizeof(name),
                          "reserved-MOVEM-form-ea%u-%u", ea, form);
            movem_reserved &= check_reserved_encoding(
                name, 0x070004 | (ea << 14) | (form << 6));
        }
    }
    for (unsigned code = 0; code < 64; ++code) {
        if ((code >= 4 && code <= 39) || code >= 57)
            continue;
        for (const uint32_t direction : {uint32_t{0}, uint32_t{0x008000}}) {
            char name[72];
            std::snprintf(name, sizeof(name),
                          "reserved-MOVEM-register-%s-%u",
                          direction ? "destination" : "source", code);
            movem_reserved &= check_reserved_encoding(
                name, 0x070000 | direction | code);
        }
    }
    for (unsigned rrr = 1; rrr < 8; ++rrr) {
        for (const uint32_t direction : {uint32_t{0}, uint32_t{0x008000}}) {
            char name[72];
            std::snprintf(name, sizeof(name),
                          "reserved-MOVEM-EA-%s-RRR%u",
                          direction ? "read" : "write", rrr);
            movem_reserved &= check_reserved_encoding(
                name, 0x077084 | direction | (rrr << 8));
        }
    }

    // DSP56001UMA2 pp. A-167--A-169 split MOVEP by bits 7:6.  Register
    // references use the same six-bit register map.  P references admit
    // only absolute RRR=000 within MMM=110.  X/Y references additionally
    // admit immediate RRR=100, but only while writing the peripheral.
    bool movep_reserved = true;
    for (unsigned code = 0; code < 64; ++code) {
        if ((code >= 4 && code <= 39) || code >= 57)
            continue;
        for (const uint32_t direction : {uint32_t{0}, uint32_t{0x008000}}) {
            char name[72];
            std::snprintf(name, sizeof(name),
                          "reserved-MOVEP-register-%s-%u",
                          direction ? "to-peripheral" : "from-peripheral",
                          code);
            movep_reserved &= check_reserved_encoding(
                name, 0x084000 | direction | (code << 8));
        }
    }
    for (unsigned rrr = 1; rrr < 8; ++rrr) {
        for (const uint32_t direction : {uint32_t{0}, uint32_t{0x008000}}) {
            char name[72];
            std::snprintf(name, sizeof(name),
                          "reserved-MOVEP-P-%s-RRR%u",
                          direction ? "to-peripheral" : "from-peripheral",
                          rrr);
            movep_reserved &= check_reserved_encoding(
                name, 0x087040 | direction | (rrr << 8));
        }
    }
    for (const uint32_t peripheral_space :
         {uint32_t{0}, uint32_t{0x010000}}) {
        for (const uint32_t memory_space :
             {uint32_t{0}, uint32_t{0x000040}}) {
            for (unsigned rrr = 1; rrr < 8; ++rrr) {
                char name[80];
                std::snprintf(name, sizeof(name),
                              "reserved-MOVEP-%c-%c-read-RRR%u",
                              peripheral_space ? 'Y' : 'X',
                              memory_space ? 'Y' : 'X', rrr);
                movep_reserved &= check_reserved_encoding(
                    name, 0x087080 | peripheral_space | memory_space |
                          (rrr << 8));
                if (rrr != 4) {
                    std::snprintf(name, sizeof(name),
                                  "reserved-MOVEP-%c-%c-write-RRR%u",
                                  peripheral_space ? 'Y' : 'X',
                                  memory_space ? 'Y' : 'X', rrr);
                    movep_reserved &= check_reserved_encoding(
                        name, 0x08f080 | peripheral_space | memory_space |
                              (rrr << 8));
                }
            }
        }
    }

    return long_vector && priority && rep_deferred && undefined_vectors &&
           div_neighbor_vectors && andi_reserved_vectors &&
           ori_reserved_vectors && false_andi_mr && false_andi_ccr &&
           false_andi_omr && jmp_low_reserved && jsr_low_reserved &&
           jcc_low_reserved && jscc_low_reserved && jmp_abs_reserved &&
           jsr_abs_reserved && jcc_abs_reserved && jscc_abs_reserved &&
           do_source_reserved && rep_source_reserved && do_ea_reserved &&
           rep_ea_reserved && parallel_move_d1_reserved &&
           parallel_j0_k4_d0_reserved && parallel_j0_k4_d1_reserved &&
           parallel_j1_k5_d0_reserved && parallel_j1_k5_d1_reserved &&
           bclr_bit24_reserved && bset_bit24_reserved &&
           bchg_bit24_reserved && btst_bit24_reserved &&
           jclr_bit24_reserved && jset_bit24_reserved &&
           jsclr_bit24_reserved && jsset_bit24_reserved &&
           jbit_mmm110_reserved &&
           class2_ax_mmm110_reserved && class2_bx_mmm110_reserved &&
           class2_ay_mmm110_reserved && class2_by_mmm110_reserved &&
           class1_reserved && register_move_reserved &&
           memory_move_reserved && long_move_reserved && movec_reserved &&
           movem_reserved && movep_reserved ? 0 : 1;
}
