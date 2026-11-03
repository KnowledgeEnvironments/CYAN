// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 ALU condition-code oracle
//
// Motorola defines AND, OR, EOR, and NOT as 24-bit A1/B1 operations.  They
// update N from bit 47, Z from bits 47:24, always clear V, retain C/E/U, and
// retain sticky L unless a parallel move limits data.  A2/B2 and A0/B0 are
// not part of the logical result or its N/Z classification.
//
// LSL, LSR, ROL, and ROR have the same 24-bit result domain.  They retain E/U,
// update C from the shifted-out A1/B1 endpoint, clear V, and classify N/Z from
// A1/B1.  LSR necessarily clears N because it shifts zero into result bit 47.
// ABS, NEG, CLR, and TST classify the full accumulator but leave C unaffected.
// RND, MPY, MPYR, MAC, and MACR likewise leave C unaffected.
// ADDR computes S+D/2 and SUBR computes D/2-S.  Both derive C from the
// carry/borrow out of bit 55, and V from the post-shift add/subtract only.
// ADDL computes S+2*D and SUBL computes 2*D-S.  Their C is the final
// add/subtract carry/borrow; V also covers a sign change caused by the shift.
// ADD, ADC, SUB, SBC, and CMP derive V from signed 56-bit overflow.  CMPM
// subtracts unsigned magnitudes into a signed 56-bit condition-code result;
// the sole positive overflow endpoint is |D|-|S| = 2^55.
// NORM selects left, right, or no shift from the incoming E/U/Z flags rather
// than reclassifying the accumulator before the operation.  It preserves C,
// and a selected left shift reports a bit-55 sign change through V and L.
// Full-accumulator E/U classification follows SR scaling mode: no-scale uses
// bits 55:47 and 47/46, scale-down uses bits 55:48 and 48/47, and scale-up
// uses bits 55:46 and 46/45, respectively.
// ASL and ASR update the full-accumulator flags.  DIV updates only L/V/C and
// must preserve the incoming E/U/N/Z classification across each iteration.
// RND, MPYR, and MACR use the SR-selected rounding position (23, 24, or 22)
// and convergent ties-to-even rather than a fixed, positively biased round.
// RND and MACR also report V and sticky L when the rounding increment itself
// crosses the signed 56-bit accumulator boundary.
///////////////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <cstdio>

#include "Vdsp56001_alu.h"
#include "verilated.h"

namespace {

enum : uint8_t {
    kAbs = 1,
    kAdd = 2,
    kSub = 3,
    kAnd = 4,
    kOr = 5,
    kEor = 6,
    kNot = 7,
    kNeg = 8,
    kClr = 9,
    kTst = 10,
    kCmp = 11,
    kAsl = 13,
    kAsr = 14,
    kLsl = 15,
    kLsr = 16,
    kRnd = 17,
    kMpy = 20,
    kMac = 21,
    kCmpm = 22,
    kAddl = 23,
    kAddr = 24,
    kSubl = 25,
    kRol = 26,
    kRor = 27,
    kDiv = 28,
    kMpyr = 29,
    kMacr = 30,
    kAdc = 31,
    kSbc = 32,
    kNorm = 33,
    kSubr = 34,
};

struct Case {
    const char *name;
    uint8_t opcode;
    uint64_t accumulator;
    uint32_t source;
    uint32_t initial_sr;
    uint64_t expected_result;
    uint8_t expected_ccr;
    uint64_t other_accumulator = 0;
    uint8_t source_selector = 4;
    bool check_norm = false;
    bool expected_norm_shift = false;
    bool expected_norm_right = false;
};

bool run_case(const Case &test) {
    Vdsp56001_alu alu;
    alu.acc_a = test.accumulator;
    alu.acc_b = test.other_accumulator;
    alu.x0 = test.source;
    alu.x1 = 0;
    alu.y0 = 0;
    alu.y1 = 0;
    alu.alu_opcode = test.opcode;
    alu.dest_sel = 0;
    alu.src_jjj = test.source_selector;
    alu.negate = 0;
    alu.mpy_src_qq = 0;
    alu.sr_in = test.initial_sr;
    alu.eval();

    const uint64_t result = alu.result;
    const uint8_t ccr = alu.ccr_out;
    const bool expected_result_valid = test.opcode != kTst &&
                                       test.opcode != kCmp &&
                                       test.opcode != kCmpm;
    const bool norm_pass = !test.check_norm ||
                           (alu.norm_did_shift == test.expected_norm_shift &&
                            alu.norm_shift_dir == test.expected_norm_right);
    const bool pass = alu.result_valid == expected_result_valid &&
                      alu.ccr_valid &&
                      result == test.expected_result &&
                      ccr == test.expected_ccr && norm_pass;
    std::printf(
        "%s ALU_CCR %s result=%014llx/%014llx CCR=%02x/%02x "
        "result_valid=%u/%u ccr_valid=%u norm=%u,%u/%u,%u\n",
        pass ? "PASS" : "FAIL", test.name,
        static_cast<unsigned long long>(result),
        static_cast<unsigned long long>(test.expected_result),
        ccr, test.expected_ccr, alu.result_valid, expected_result_valid,
        alu.ccr_valid, alu.norm_did_shift, alu.norm_shift_dir,
        test.expected_norm_shift, test.expected_norm_right);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);

    // CCR order is {reserved,L,E,U,N,Z,V,C}.  These cases deliberately make
    // A2/A0 disagree with A1 so whole-accumulator classification cannot pass.
    const Case cases[] = {
        {"AND-zero-A1-preserve-EUC", kAnd,
         0x007fffff123456ULL, 0x000000, 0x31,
         0x00000000123456ULL, 0x35},
        {"OR-negative-A1-positive-A2", kOr,
         0x00000000000001ULL, 0x800000, 0x25,
         0x00800000000001ULL, 0x29},
        {"EOR-positive-A1-negative-A2", kEor,
         0x80ffffff654321ULL, 0xffffff, 0x51,
         0x80000000654321ULL, 0x55},
        {"NOT-zero-A1-nonzero-A0", kNot,
         0x00ffffffabcdefULL, 0x13579b, 0x71,
         0x00000000abcdefULL, 0x75},
        {"LSL-negative-result-positive-A2", kLsl,
         0x00400000123456ULL, 0x000000, 0x30,
         0x00800000123456ULL, 0x38},
        {"LSR-zero-A1-negative-A2", kLsr,
         0x80000001123456ULL, 0x000000, 0x50,
         0x80000000123456ULL, 0x55},
        {"ROL-carry-in-nonzero-A1-negative-A2", kRol,
         0x80000000123456ULL, 0x000000, 0x21,
         0x80000001123456ULL, 0x20},
        {"ROR-zero-A1-nonzero-outer-pieces", kRor,
         0x7f000001abcdefULL, 0x000000, 0x50,
         0x7f000000abcdefULL, 0x55},
        {"ABS-preserve-set-carry", kAbs,
         0xfffffffffffff2ULL, 0x000000, 0x41,
         0x0000000000000eULL, 0x51},
        {"NEG-preserve-clear-carry", kNeg,
         0x00000000000001ULL, 0x000000, 0x00,
         0xffffffffffffffULL, 0x18},
        {"CLR-preserve-set-carry", kClr,
         0x123456789abcdeULL, 0x000000, 0x41,
         0x00000000000000ULL, 0x55},
        {"TST-preserve-set-carry", kTst,
         0x01020304000000ULL, 0x000000, 0x41,
         0x01020304000000ULL, 0x71},
        {"RND-preserve-set-carry", kRnd,
         0x00000000000000ULL, 0x000000, 0x01,
         0x00000000000000ULL, 0x15},
        {"MPY-preserve-set-carry", kMpy,
         0x00000000000000ULL, 0x000000, 0x01,
         0x00000000000000ULL, 0x15},
        {"MPYR-preserve-set-carry", kMpyr,
         0x00000000000000ULL, 0x000000, 0x01,
         0x00000000000000ULL, 0x15},
        {"MAC-preserve-set-carry", kMac,
         0x00000000000000ULL, 0x000000, 0x01,
         0x00000000000000ULL, 0x15},
        {"MACR-preserve-set-carry", kMacr,
         0x00000000000000ULL, 0x000000, 0x01,
         0x00000000000000ULL, 0x15},
        {"ADDR-carry-out", kAddr,
         0x7fffffffffffffULL, 0x000000, 0x00,
         0x3ffffffffffffeULL, 0x31,
         0xffffffffffffffULL, 0},
        {"SUBR-operand-order", kSubr,
         0x00000000000002ULL, 0x000000, 0x00,
         0x00000000000001ULL, 0x10,
         0x00000000000000ULL, 0},
        {"SUBR-borrow-out", kSubr,
         0x00000000000000ULL, 0x000000, 0x00,
         0xffffffffffffffULL, 0x19,
         0x00000000000001ULL, 0},
        {"ADDR-post-shift-overflow", kAddr,
         0x7fffffffffffffULL, 0x000000, 0x00,
         0x80000000000000ULL, 0x7a,
         0x40000000000001ULL, 0},
        {"SUBR-post-shift-overflow", kSubr,
         0x7fffffffffffffULL, 0x000000, 0x00,
         0x80000000000000ULL, 0x7b,
         0xbfffffffffffffULL, 0},
        {"ADDL-final-carry", kAddl,
         0x00000000000001ULL, 0x000000, 0x00,
         0x00000000000000ULL, 0x15,
         0xfffffffffffffeULL, 0},
        {"SUBL-final-borrow", kSubl,
         0x00000000000001ULL, 0x000000, 0x00,
         0xffffffffffffffULL, 0x19,
         0x00000000000003ULL, 0},
        {"ADDL-shift-overflow", kAddl,
         0x40000000000000ULL, 0x000000, 0x00,
         0x80000000000000ULL, 0x7a,
         0x00000000000000ULL, 0},
        {"SUBL-shift-overflow", kSubl,
         0x40000000000000ULL, 0x000000, 0x00,
         0x80000000000000ULL, 0x7a,
         0x00000000000000ULL, 0},
        {"ADDL-add-overflow", kAddl,
         0x20000000000000ULL, 0x000000, 0x00,
         0x80000000000000ULL, 0x7a,
         0x40000000000000ULL, 0},
        {"SUBL-subtract-overflow", kSubl,
         0x20000000000000ULL, 0x000000, 0x00,
         0x80000000000000ULL, 0x7b,
         0xc0000000000000ULL, 0},
        {"ADD-signed-overflow", kAdd,
         0x7fffffffffffffULL, 0x000000, 0x00,
         0x80000000000000ULL, 0x7a,
         0x00000000000001ULL, 0},
        {"ADC-carry-in-overflow", kAdc,
         0x7fffffffffffffULL, 0x000000, 0x01,
         0x80000000000000ULL, 0x7a,
         0x00000000000000ULL, 0},
        {"SUB-signed-overflow", kSub,
         0x80000000000000ULL, 0x000000, 0x00,
         0x7fffffffffffffULL, 0x72,
         0x00000000000001ULL, 0},
        {"SBC-borrow-in-overflow", kSbc,
         0x80000000000000ULL, 0x000000, 0x01,
         0x7fffffffffffffULL, 0x72,
         0x00000000000000ULL, 0},
        {"CMP-signed-overflow-no-writeback", kCmp,
         0x80000000000000ULL, 0x000000, 0x00,
         0x80000000000000ULL, 0x72,
         0x00000000000001ULL, 0},
        {"CMPM-positive-magnitude-overflow", kCmpm,
         0x80000000000000ULL, 0x000000, 0x00,
         0x80000000000000ULL, 0x7a,
         0x00000000000000ULL, 0},
        {"CMPM-negative-limit-no-overflow", kCmpm,
         0x00000000000000ULL, 0x000000, 0x00,
         0x00000000000000ULL, 0x39,
         0x80000000000000ULL, 0},
        {"NORM-incoming-U-selects-left", kNorm,
         0x00400000000000ULL, 0x000000, 0x11,
         0x00800000000000ULL, 0x21,
         0x00000000000000ULL, 4, true, true, false},
        {"NORM-incoming-E-selects-right", kNorm,
         0x00000000000002ULL, 0x000000, 0x21,
         0x00000000000001ULL, 0x11,
         0x00000000000000ULL, 4, true, true, true},
        {"NORM-clear-EU-selects-nop", kNorm,
         0x00000000000002ULL, 0x000000, 0x01,
         0x00000000000002ULL, 0x11,
         0x00000000000000ULL, 4, true, false, false},
        {"NORM-incoming-Z-selects-nop", kNorm,
         0x00000000000002ULL, 0x000000, 0x15,
         0x00000000000002ULL, 0x11,
         0x00000000000000ULL, 4, true, false, false},
        {"NORM-left-sign-change-sets-VL", kNorm,
         0x40000000000000ULL, 0x000000, 0x11,
         0x80000000000000ULL, 0x7b,
         0x00000000000000ULL, 4, true, true, false},
        {"NORM-right-preserves-carry", kNorm,
         0x01000000000000ULL, 0x000000, 0x21,
         0x00800000000000ULL, 0x21,
         0x00000000000000ULL, 4, true, true, true},
        {"NORM-zero-nop-preserves-carry", kNorm,
         0x00000000000000ULL, 0x000000, 0x15,
         0x00000000000000ULL, 0x15,
         0x00000000000000ULL, 4, true, false, false},
        {"TST-scale-down-U-uses-bits-48-47", kTst,
         0x00400000000000ULL, 0x000000, 0x401,
         0x00400000000000ULL, 0x11},
        {"TST-scale-down-E-uses-bits-55-48", kTst,
         0x00800000000000ULL, 0x000000, 0x401,
         0x00800000000000ULL, 0x01},
        {"TST-scale-up-EU-uses-bits-46-45", kTst,
         0x00600000000000ULL, 0x000000, 0x801,
         0x00600000000000ULL, 0x31},
        {"ASL-full-accumulator-flags", kAsl,
         0xa5012345012345ULL, 0x000000, 0x300,
         0x4a02468a02468aULL, 0x73},
        {"ASR-full-accumulator-flags", kAsr,
         0xa8a86420a86421ULL, 0x000000, 0x300,
         0xd4543210543210ULL, 0x29},
        {"DIV-preserves-EUNZ", kDiv,
         0x00000000000001ULL, 0x000001, 0x3c,
         0xffffffff000002ULL, 0x3c},
        {"RND-no-scale-even-tie", kRnd,
         0x00123456800000ULL, 0x000000, 0x001,
         0x00123456000000ULL, 0x11},
        {"RND-no-scale-odd-tie", kRnd,
         0x00123455800000ULL, 0x000000, 0x001,
         0x00123456000000ULL, 0x11},
        {"MPYR-no-scale-even-tie", kMpyr,
         0x00000000000000ULL, 0x000800, 0x001,
         0x00000000000000ULL, 0x15},
        {"MPYR-scale-down-position-24", kMpyr,
         0x00000000000000ULL, 0x000800, 0x401,
         0x00000000000000ULL, 0x15},
        {"MPYR-scale-up-position-22", kMpyr,
         0x00000000000000ULL, 0x000600, 0x801,
         0x00000000800000ULL, 0x11},
        {"MACR-no-scale-even-tie", kMacr,
         0x00123456800000ULL, 0x000000, 0x001,
         0x00123456000000ULL, 0x11},
        {"MACR-scale-down-position-24", kMacr,
         0x00000000800000ULL, 0x000000, 0x401,
         0x00000000000000ULL, 0x15},
        {"MACR-scale-up-position-22", kMacr,
         0x00000000480000ULL, 0x000000, 0x801,
         0x00000000800000ULL, 0x11},
        {"RND-rounding-overflow", kRnd,
         0x7fffffffffffffULL, 0x000000, 0x001,
         0x80000000000000ULL, 0x7b},
        {"MACR-rounding-overflow", kMacr,
         0x7fffffffffffffULL, 0x000000, 0x001,
         0x80000000000000ULL, 0x7b},
    };

    bool pass = true;
    for (const Case &test : cases)
        pass &= run_case(test);
    return pass ? 0 : 1;
}
