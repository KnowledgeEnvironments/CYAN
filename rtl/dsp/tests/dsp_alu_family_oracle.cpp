// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 production-core nonmultiply ALU opcode-family matrix
//
// Exercise every legal nonmultiply parallel-ALU encoding through the product
// decoder and core, for both accumulator destinations.  The direct ALU CCR
// oracle independently grades flag edge cases; this oracle grades the exact
// instruction-to-operation/source/destination composition, result writeback,
// opposite-accumulator preservation, and absence of unintended X/Y traffic.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint8_t kStateWritebackFetch = 4;
constexpr uint64_t kMask56 = 0x00ffffffffffffffULL;
constexpr uint64_t kA1Mask = 0x0000ffffff000000ULL;
constexpr uint64_t kInitialA = 0x00000003000000ULL;
constexpr uint64_t kInitialB = 0x00000001000000ULL;
constexpr uint32_t kX0 = 0x000002;
constexpr uint32_t kX1 = 0x000008;
constexpr uint32_t kY0 = 0x000004;
constexpr uint32_t kY1 = 0x000010;

enum class Operation {
    Abs,
    Adc,
    Add,
    Addl,
    Addr,
    And,
    Asl,
    Asr,
    Clr,
    Cmp,
    Cmpm,
    Eor,
    Lsl,
    Lsr,
    Neg,
    Not,
    Or,
    Rnd,
    Rol,
    Ror,
    Sbc,
    Sub,
    Subl,
    Subr,
    Tfr,
    Tst,
};

struct Encoding {
    uint8_t low;
    const char *name;
    Operation operation;
    uint8_t source_jjj;
    bool writes_result;
};

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 8> p{};
    uint32_t target_instruction;
    unsigned cycles = 0;
    int64_t decode_cycle = -1;
    int64_t writeback_cycle = -1;
    unsigned x_reads = 0;
    unsigned x_writes = 0;
    unsigned y_reads = 0;
    unsigned y_writes = 0;

    explicit Harness(uint32_t instruction) : target_instruction(instruction) {
        p[0] = instruction;
        p[1] = 0x0af080; // JMP >$0001.
        p[2] = 0x000001;

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
        core.irq_sci_rx_exception = 0;
        core.irq_sci_idle = 0;
        core.irq_sci_timer = 0;
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
        if (core.state_out == 2 &&
            (core.p_fetch_data & 0x00ffffff) == target_instruction &&
            decode_cycle < 0)
            decode_cycle = cycles;
        if (core.state_out == kStateWritebackFetch &&
            core.cur_inst_out == target_instruction && writeback_cycle < 0)
            writeback_cycle = cycles;
        x_reads += core.x_rd_en;
        x_writes += core.x_wr_en;
        y_reads += core.y_rd_en;
        y_writes += core.y_wr_en;
        core.clk = 1;
        core.eval();
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 7] & 0x00ffffff;
        ++cycles;
    }

    void debug_write(uint8_t address, uint64_t value) {
        core.dbg_reg_wr_addr = address;
        core.dbg_reg_wr_data = value;
        core.dbg_reg_wr_en = 1;
        tick();
        core.dbg_reg_wr_en = 0;
    }

    bool retire(uint32_t instruction) {
        core.run = 1;
        for (unsigned i = 0; i < 80; ++i) {
            if (core.state_out == kStateWritebackFetch &&
                core.cur_inst_out == instruction) {
                tick();
                return true;
            }
            tick();
        }
        return false;
    }

    int64_t instruction_clocks() const {
        return decode_cycle >= 0 && writeback_cycle >= 0 ?
            writeback_cycle - decode_cycle + 1 : -1;
    }
};

uint64_t source_for(uint8_t jjj, bool dest_b) {
    const uint64_t other = dest_b ? kInitialA : kInitialB;
    switch (jjj) {
    case 0:
    case 1:
        return other;
    case 2:
        return (static_cast<uint64_t>(kX1) << 24) | kX0;
    case 3:
        return (static_cast<uint64_t>(kY1) << 24) | kY0;
    case 4:
        return static_cast<uint64_t>(kX0) << 24;
    case 5:
        return static_cast<uint64_t>(kY0) << 24;
    case 6:
        return static_cast<uint64_t>(kX1) << 24;
    default:
        return static_cast<uint64_t>(kY1) << 24;
    }
}

uint64_t arithmetic_shift_right(uint64_t value) {
    return ((value >> 1) | (value & (1ULL << 55))) & kMask56;
}

uint64_t a1_result(uint64_t destination, uint32_t a1) {
    return (destination & ~kA1Mask) |
           ((static_cast<uint64_t>(a1) << 24) & kA1Mask);
}

uint64_t expected_result(Operation operation, uint64_t destination,
                         uint64_t source) {
    const uint32_t d1 = (destination >> 24) & 0x00ffffff;
    const uint32_t s1 = (source >> 24) & 0x00ffffff;
    switch (operation) {
    case Operation::Abs:
        return (destination & (1ULL << 55))
                   ? ((~destination + 1) & kMask56)
                   : destination;
    case Operation::Adc:
        return (destination + source + 1) & kMask56;
    case Operation::Add:
        return (destination + source) & kMask56;
    case Operation::Addl:
        return ((destination << 1) + source) & kMask56;
    case Operation::Addr:
        return (arithmetic_shift_right(destination) + source) & kMask56;
    case Operation::And:
        return a1_result(destination, d1 & s1);
    case Operation::Asl:
        return (destination << 1) & kMask56;
    case Operation::Asr:
        return arithmetic_shift_right(destination);
    case Operation::Clr:
        return 0;
    case Operation::Cmp:
    case Operation::Cmpm:
    case Operation::Tst:
        return destination;
    case Operation::Eor:
        return a1_result(destination, d1 ^ s1);
    case Operation::Lsl:
        return a1_result(destination, (d1 << 1) & 0x00ffffff);
    case Operation::Lsr:
        return a1_result(destination, d1 >> 1);
    case Operation::Neg:
        return (~destination + 1) & kMask56;
    case Operation::Not:
        return a1_result(destination, (~d1) & 0x00ffffff);
    case Operation::Or:
        return a1_result(destination, d1 | s1);
    case Operation::Rnd:
        // Both fixture accumulators are exact 24-bit fractions, so convergent
        // rounding at the default scale is intentionally an identity.
        return destination;
    case Operation::Rol:
        return a1_result(destination, ((d1 << 1) | 1) & 0x00ffffff);
    case Operation::Ror:
        return a1_result(destination, (1U << 23) | (d1 >> 1));
    case Operation::Sbc:
        return (destination - source - 1) & kMask56;
    case Operation::Sub:
        return (destination - source) & kMask56;
    case Operation::Subl:
        return ((destination << 1) - source) & kMask56;
    case Operation::Subr:
        return (arithmetic_shift_right(destination) - source) & kMask56;
    case Operation::Tfr:
        return source;
    }
    return 0;
}

bool run_case(const Encoding &encoding, bool dest_b) {
    const uint8_t low = encoding.low | (dest_b ? 0x08 : 0);
    const uint32_t instruction = 0x200000 | low;
    Harness h(instruction);
    h.debug_write(0x04, kX0);
    h.debug_write(0x05, kX1);
    h.debug_write(0x06, kY0);
    h.debug_write(0x07, kY1);
    h.debug_write(0x0e, kInitialA);
    h.debug_write(0x0f, kInitialB);
    h.debug_write(0x39, 0x000001); // Carry in is one for ADC/SBC/ROL/ROR.
    h.x_reads = h.x_writes = h.y_reads = h.y_writes = 0;

    const bool completed = h.retire(instruction);
    const uint64_t initial_destination = dest_b ? kInitialB : kInitialA;
    const uint64_t initial_other = dest_b ? kInitialA : kInitialB;
    const uint64_t source = source_for(encoding.source_jjj, dest_b);
    const uint64_t computed = expected_result(
        encoding.operation, initial_destination, source);
    const uint64_t expected_destination =
        encoding.writes_result ? computed : initial_destination;
    const uint64_t actual_destination =
        dest_b ? h.core.acc_b_out : h.core.acc_a_out;
    const uint64_t actual_other = dest_b ? h.core.acc_a_out : h.core.acc_b_out;
    const bool bus_quiet = h.x_reads == 0 && h.x_writes == 0 &&
                           h.y_reads == 0 && h.y_writes == 0;
    const bool pass = completed && actual_destination == expected_destination &&
                      actual_other == initial_other && bus_quiet &&
                      h.instruction_clocks() == 2;
    std::printf(
        "%s ALU_FAMILY low=%02x op=%s dest=%c src=%u "
        "result=%014llx/%014llx other=%014llx/%014llx "
        "bus=X%u/%u,Y%u/%u timing=%lld/2 cycles=%u\n",
        pass ? "PASS" : "FAIL", low, encoding.name, dest_b ? 'B' : 'A',
        encoding.source_jjj,
        static_cast<unsigned long long>(actual_destination),
        static_cast<unsigned long long>(expected_destination),
        static_cast<unsigned long long>(actual_other),
        static_cast<unsigned long long>(initial_other), h.x_reads, h.x_writes,
        h.y_reads, h.y_writes,
        static_cast<long long>(h.instruction_clocks()), h.cycles);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);

    const std::vector<Encoding> encodings = {
        {0x01, "TFR", Operation::Tfr, 0, true},
        {0x02, "ADDR", Operation::Addr, 0, true},
        {0x03, "TST", Operation::Tst, 0, false},
        {0x05, "CMP", Operation::Cmp, 0, false},
        {0x06, "SUBR", Operation::Subr, 0, true},
        {0x07, "CMPM", Operation::Cmpm, 0, false},
        {0x10, "ADD", Operation::Add, 1, true},
        {0x11, "RND", Operation::Rnd, 1, true},
        {0x12, "ADDL", Operation::Addl, 1, true},
        {0x13, "CLR", Operation::Clr, 1, true},
        {0x14, "SUB", Operation::Sub, 1, true},
        {0x16, "SUBL", Operation::Subl, 1, true},
        {0x17, "NOT", Operation::Not, 1, true},
        {0x20, "ADD", Operation::Add, 2, true},
        {0x21, "ADC", Operation::Adc, 2, true},
        {0x22, "ASR", Operation::Asr, 2, true},
        {0x23, "LSR", Operation::Lsr, 2, true},
        {0x24, "SUB", Operation::Sub, 2, true},
        {0x25, "SBC", Operation::Sbc, 2, true},
        {0x26, "ABS", Operation::Abs, 2, true},
        {0x27, "ROR", Operation::Ror, 2, true},
        {0x30, "ADD", Operation::Add, 3, true},
        {0x31, "ADC", Operation::Adc, 3, true},
        {0x32, "ASL", Operation::Asl, 3, true},
        {0x33, "LSL", Operation::Lsl, 3, true},
        {0x34, "SUB", Operation::Sub, 3, true},
        {0x35, "SBC", Operation::Sbc, 3, true},
        {0x36, "NEG", Operation::Neg, 3, true},
        {0x37, "ROL", Operation::Rol, 3, true},
    };

    std::vector<Encoding> all = encodings;
    for (uint8_t jjj = 4; jjj < 8; ++jjj) {
        const uint8_t base = jjj << 4;
        all.push_back({static_cast<uint8_t>(base | 0), "ADD", Operation::Add,
                       jjj, true});
        all.push_back({static_cast<uint8_t>(base | 1), "TFR", Operation::Tfr,
                       jjj, true});
        all.push_back({static_cast<uint8_t>(base | 2), "OR", Operation::Or,
                       jjj, true});
        all.push_back({static_cast<uint8_t>(base | 3), "EOR", Operation::Eor,
                       jjj, true});
        all.push_back({static_cast<uint8_t>(base | 4), "SUB", Operation::Sub,
                       jjj, true});
        all.push_back({static_cast<uint8_t>(base | 5), "CMP", Operation::Cmp,
                       jjj, false});
        all.push_back({static_cast<uint8_t>(base | 6), "AND", Operation::And,
                       jjj, true});
        all.push_back({static_cast<uint8_t>(base | 7), "CMPM", Operation::Cmpm,
                       jjj, false});
    }

    unsigned failures = 0;
    unsigned cases = 0;
    for (const auto &encoding : all) {
        for (unsigned destination = 0; destination < 2; ++destination) {
            ++cases;
            if (!run_case(encoding, destination != 0))
                ++failures;
        }
    }
    std::printf("ALU FAMILY SUMMARY cases=%u failures=%u\n", cases, failures);
    return failures == 0 ? 0 : 1;
}
