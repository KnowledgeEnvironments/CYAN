// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 data-ALU deterministic differential oracle
//
// Exercise the synthesizable ALU directly over every nonmultiply operation,
// both destinations, every JJJ source selector, both carry inputs, structured
// boundary operands, and a fixed pseudo-random stream.  Expected results are
// computed with a small integer model rather than copied from RTL equations.
// Decode/retirement composition is covered separately by dsp_alu_family_oracle.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_alu.h"
#include "verilated.h"

namespace {

constexpr uint64_t kMask56 = 0x00ffffffffffffffULL;
constexpr uint64_t kSign56 = 0x0080000000000000ULL;
constexpr uint64_t kA1Mask = 0x0000ffffff000000ULL;

enum : uint8_t {
    kAbs = 1, kAdd = 2, kSub = 3, kAnd = 4, kOr = 5, kEor = 6,
    kNot = 7, kNeg = 8, kClr = 9, kTst = 10, kCmp = 11, kTfr = 12,
    kAsl = 13, kAsr = 14, kLsl = 15, kLsr = 16, kRnd = 17,
    kCmpm = 22, kAddl = 23, kAddr = 24, kSubl = 25, kRol = 26,
    kRor = 27, kAdc = 31, kSbc = 32, kNorm = 33, kSubr = 34,
};

struct Operation {
    uint8_t opcode;
    const char *name;
};

constexpr std::array<Operation, 27> kOperations{{
    {kAbs, "ABS"}, {kAdd, "ADD"}, {kSub, "SUB"}, {kAnd, "AND"},
    {kOr, "OR"}, {kEor, "EOR"}, {kNot, "NOT"}, {kNeg, "NEG"},
    {kClr, "CLR"}, {kTst, "TST"}, {kCmp, "CMP"}, {kTfr, "TFR"},
    {kAsl, "ASL"}, {kAsr, "ASR"}, {kLsl, "LSL"}, {kLsr, "LSR"},
    {kRnd, "RND"}, {kCmpm, "CMPM"}, {kAddl, "ADDL"},
    {kAddr, "ADDR"}, {kSubl, "SUBL"}, {kRol, "ROL"},
    {kRor, "ROR"}, {kAdc, "ADC"}, {kSbc, "SBC"},
    {kNorm, "NORM"}, {kSubr, "SUBR"},
}};

uint64_t next_random(uint64_t &state) {
    state ^= state << 13;
    state ^= state >> 7;
    state ^= state << 17;
    return state & kMask56;
}

uint64_t operand(unsigned ordinal, uint64_t &state) {
    constexpr std::array<uint64_t, 16> edges{{
        0, 1, 2, 3, kMask56, kMask56 - 1, kSign56, kSign56 - 1,
        kSign56 + 1, 0x007fffffffffffffULL, 0x00000000ffffffULL,
        0x00000001000000ULL, 0x00000080000000ULL, 0x00ffffff000000ULL,
        0x00555555555555ULL, 0x00aaaaaaaaaaaaULL,
    }};
    return ordinal < edges.size() ? edges[ordinal] : next_random(state);
}

uint64_t sign_extend_24(uint32_t value) {
    const uint64_t v = value & 0x00ffffffU;
    return (v | ((v & 0x00800000U) ? 0x00ffffffff000000ULL : 0)) &
           kMask56;
}

uint64_t source_for(uint8_t selector, bool dest_b, uint64_t acc_a,
                    uint64_t acc_b, uint32_t x0, uint32_t x1,
                    uint32_t y0, uint32_t y1) {
    const uint64_t other = dest_b ? acc_a : acc_b;
    switch (selector) {
    case 0:
    case 1:
        return other;
    case 2:
        return ((sign_extend_24(x1) << 24) & kMask56) | x0;
    case 3:
        return ((sign_extend_24(y1) << 24) & kMask56) | y0;
    case 4:
        return (sign_extend_24(x0) << 24) & kMask56;
    case 5:
        return (sign_extend_24(y0) << 24) & kMask56;
    case 6:
        return (sign_extend_24(x1) << 24) & kMask56;
    default:
        return (sign_extend_24(y1) << 24) & kMask56;
    }
}

uint64_t replace_a1(uint64_t destination, uint32_t a1) {
    return (destination & ~kA1Mask) |
           ((static_cast<uint64_t>(a1 & 0x00ffffffU) << 24) & kA1Mask);
}

uint64_t shift_right_signed(uint64_t value) {
    const int64_t signed_value = static_cast<int64_t>(value << 8) >> 8;
    return static_cast<uint64_t>(signed_value >> 1) & kMask56;
}

uint64_t round_convergent(uint64_t value, uint32_t sr) {
    const unsigned scale = (sr >> 10) & 3;
    const uint64_t half = scale == 2 ? (1ULL << 22) :
                          scale == 1 ? (1ULL << 24) : (1ULL << 23);
    const uint64_t quantum = half << 1;
    const uint64_t fraction = value & (quantum - 1);
    uint64_t rounded = value & ~(quantum - 1);
    if (fraction > half || (fraction == half && (rounded & quantum)))
        rounded += quantum;
    return rounded & kMask56;
}

uint64_t expected_result(uint8_t opcode, uint64_t destination,
                         uint64_t source, uint32_t sr) {
    const uint32_t d1 = (destination >> 24) & 0x00ffffffU;
    const uint32_t s1 = (source >> 24) & 0x00ffffffU;
    switch (opcode) {
    case kAbs:
        return destination & kSign56 ? (-destination) & kMask56 : destination;
    case kAdd: return (destination + source) & kMask56;
    case kSub: return (destination - source) & kMask56;
    case kAnd: return replace_a1(destination, d1 & s1);
    case kOr: return replace_a1(destination, d1 | s1);
    case kEor: return replace_a1(destination, d1 ^ s1);
    case kNot: return replace_a1(destination, ~d1);
    case kNeg: return (-destination) & kMask56;
    case kClr: return 0;
    case kTst:
    case kCmp:
    case kCmpm:
        return destination;
    case kTfr: return source;
    case kAsl: return (destination << 1) & kMask56;
    case kAsr: return shift_right_signed(destination);
    case kLsl: return replace_a1(destination, d1 << 1);
    case kLsr: return replace_a1(destination, d1 >> 1);
    case kRnd: return round_convergent(destination, sr);
    case kAddl: return ((destination << 1) + source) & kMask56;
    case kAddr: return (shift_right_signed(destination) + source) & kMask56;
    case kSubl: return ((destination << 1) - source) & kMask56;
    case kRol: return replace_a1(destination, (d1 << 1) | (sr & 1));
    case kRor:
        return replace_a1(destination, ((sr & 1) << 23) | (d1 >> 1));
    case kAdc: return (destination + source + (sr & 1)) & kMask56;
    case kSbc: return (destination - source - (sr & 1)) & kMask56;
    case kNorm:
        if ((sr & 0x20) != 0)
            return shift_right_signed(destination);
        if ((sr & 0x10) != 0 && (sr & 0x04) == 0)
            return (destination << 1) & kMask56;
        return destination;
    case kSubr: return (shift_right_signed(destination) - source) & kMask56;
    default: return destination;
    }
}

bool expected_result_valid(uint8_t opcode) {
    return opcode != kTst && opcode != kCmp && opcode != kCmpm;
}

bool expected_ccr_valid(uint8_t opcode) {
    return opcode != kTfr;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Vdsp56001_alu alu;
    uint64_t random_state = 0x00560a1d1ffee123ULL;
    unsigned cases = 0;
    unsigned failures = 0;

    for (const auto &operation : kOperations) {
        for (unsigned dest_b = 0; dest_b < 2; ++dest_b) {
            for (unsigned selector = 0; selector < 8; ++selector) {
                for (unsigned sample = 0; sample < 128; ++sample) {
                    const uint64_t acc_a = operand(sample, random_state);
                    const uint64_t acc_b = operand(sample + 3, random_state);
                    // Verilator's public C++ members use a 32-bit carrier for
                    // 24-bit ports.  Mask here exactly as physical pins do;
                    // otherwise out-of-width host bits can leak into a model.
                    const uint32_t x0 = operand(sample + 5, random_state) &
                                        0x00ffffffU;
                    const uint32_t x1 = operand(sample + 7, random_state) &
                                        0x00ffffffU;
                    const uint32_t y0 = operand(sample + 9, random_state) &
                                        0x00ffffffU;
                    const uint32_t y1 = operand(sample + 11, random_state) &
                                        0x00ffffffU;
                    uint32_t sr = static_cast<uint32_t>(
                        operand(sample + 13, random_state)) & 0x0c75;
                    // S1:S0=11 is reserved by the DSP56001 manual and has no
                    // architectural rounding contract.  Keep the sweep to
                    // the three legal scaling modes rather than grading an
                    // implementation-defined state.
                    if ((sr & 0x0c00) == 0x0c00)
                        sr &= ~0x0c00U;
                    const uint64_t destination = dest_b ? acc_b : acc_a;
                    const uint64_t source = source_for(selector, dest_b,
                        acc_a, acc_b, x0, x1, y0, y1);
                    const uint64_t expected = expected_result(
                        operation.opcode, destination, source, sr);

                    alu.acc_a = acc_a;
                    alu.acc_b = acc_b;
                    alu.x0 = x0;
                    alu.x1 = x1;
                    alu.y0 = y0;
                    alu.y1 = y1;
                    alu.alu_opcode = operation.opcode;
                    alu.dest_sel = dest_b;
                    alu.src_jjj = selector;
                    alu.negate = 0;
                    alu.mpy_src_qq = 0;
                    alu.sr_in = sr;
                    alu.eval();
                    ++cases;

                    const bool pass = alu.result == expected &&
                        alu.result_valid == expected_result_valid(operation.opcode) &&
                        alu.ccr_valid == expected_ccr_valid(operation.opcode);
                    if (!pass && failures++ < 20) {
                        std::printf(
                            "FAIL ALU_DIFF op=%s dest=%c sel=%u sample=%u "
                            "d=%014llx s=%014llx sr=%06x result=%014llx/%014llx "
                            "valid=%u/%u ccr_valid=%u/%u\n",
                            operation.name, dest_b ? 'B' : 'A', selector,
                            sample, static_cast<unsigned long long>(destination),
                            static_cast<unsigned long long>(source), sr,
                            static_cast<unsigned long long>(alu.result),
                            static_cast<unsigned long long>(expected),
                            alu.result_valid,
                            expected_result_valid(operation.opcode),
                            alu.ccr_valid, expected_ccr_valid(operation.opcode));
                    }
                }
            }
        }
    }

    if (failures != 0) {
        std::printf("FAIL ALU differential sweep failures=%u cases=%u\n",
                    failures, cases);
        return 1;
    }
    std::printf("PASS ALU differential sweep cases=%u seed=560a1d1ffee123\n",
                cases);
    return 0;
}
