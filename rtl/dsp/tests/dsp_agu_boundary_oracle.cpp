// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 address-generation-unit boundary oracle
//
// Motorola DSP56001UM05 section 5.3.2 specifies that every address-register
// indirect mode uses its Mn modifier.  It also requires legal modulo updates
// to remain in their circular buffer even when the 16-bit linear sum crosses
// $0000/$ffff.  Exercise those boundaries directly in the synthesizable AGU.
///////////////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <cstdio>

#include "Vdsp56001_agu.h"
#include "verilated.h"

namespace {

struct Case {
    const char *name;
    uint16_t r;
    uint16_t n;
    uint16_t m;
    uint8_t mode;
    uint16_t effective;
    uint16_t updated;
    bool write;
    bool valid = true;
};

bool run_case(const Case &test, uint8_t rrr) {
    Vdsp56001_agu agu;
    agu.r_in = test.r;
    agu.n_in = test.n;
    agu.m_in = test.m;
    agu.agu_rrr = rrr;
    agu.agu_mmm = test.mode;
    agu.eval();

    const bool pass = agu.eff_addr == test.effective &&
                      (!test.write || agu.r_out == test.updated) &&
                      agu.r_wr_en == test.write &&
                      agu.r_wr_idx == rrr &&
                      agu.eff_addr_valid == test.valid;
    std::printf(
        "%s AGU %s R%u r=%04x n=%04x m=%04x mode=%u "
        "ea=%04x/%04x update=%04x/%04x wr=%u/%u valid=%u/%u\n",
        pass ? "PASS" : "FAIL", test.name, rrr, test.r, test.n, test.m,
        test.mode, agu.eff_addr, test.effective, agu.r_out, test.updated,
        agu.r_wr_en, test.write, agu.eff_addr_valid, test.valid);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);

    // Modes: 0=(Rn)-Nn, 1=(Rn)+Nn, 2=(Rn)-, 3=(Rn)+,
    // 4=(Rn), 5=(Rn+Nn), 6=absolute, 7=-(Rn).
    const Case cases[] = {
        {"linear-plus-wrap", 0xfff0, 0x0014, 0xffff, 1,
         0xfff0, 0x0004, true},
        {"linear-minus-wrap", 0x0004, 0x0014, 0xffff, 0,
         0x0004, 0xfff0, true},
        {"linear-indexed", 0x1234, 0x0021, 0xffff, 5,
         0x1255, 0x1255, false},
        {"linear-predecrement", 0x0000, 0x7777, 0xffff, 7,
         0xffff, 0xffff, true},
        {"direct-no-update", 0x5678, 0x1111, 0x0014, 4,
         0x5678, 0x5678, false},
        {"absolute-invalid", 0x5678, 0x1111, 0x0014, 6,
         0x5678, 0x5678, false, false},

        // UM05 Figure 5-13: modulo 21, base 64, R=75, N=15 -> 69.
        {"manual-modulo21-postplus", 75, 15, 20, 1,
         75, 69, true},
        // UM05 5.3.2.2 explicit outside-range example: R=6, M=5, N=0 -> 0.
        {"manual-outside-range", 6, 0, 5, 1,
         6, 0, true},
        {"modulo21-upper-plus-one", 84, 0x7777, 20, 3,
         84, 64, true},
        {"modulo21-lower-minus-one", 64, 0x7777, 20, 2,
         64, 84, true},
        {"modulo21-predecrement", 64, 0x7777, 20, 7,
         84, 84, true},
        // Legal |N| <= modifier at the two 16-bit address-space boundaries.
        {"modulo21-global-upper-wrap", 0xfff0, 20, 20, 1,
         0xfff0, 0xffef, true},
        {"modulo21-global-lower-wrap", 0x0004, 20, 20, 0,
         0x0004, 0x0005, true},
        // Indexed mode uses identical Mn arithmetic but does not update Rn.
        {"modulo21-indexed", 75, 15, 20, 5,
         69, 69, false},
        // UM05 5.3.2.2 special case: N=P*2^k jumps linearly to the
        // same relative address in the adjacent aligned buffer.
        {"modulo21-block-jump-plus", 75, 32, 20, 1,
         75, 107, true},
        {"modulo21-block-jump-minus", 75, 32, 20, 0,
         75, 43, true},
        {"modulo21-block-jump-indexed", 75, 32, 20, 5,
         107, 107, false},

        // UM05 Table 5-5 reverse-carry sequence for a 1024-point FFT.
        {"reverse-first", 3072, 512, 0, 1,
         3072, 3584, true},
        {"reverse-second", 3584, 512, 0, 1,
         3584, 3328, true},
        {"reverse-indexed", 3584, 512, 0, 5,
         3328, 3328, false},
    };

    unsigned failures = 0;
    unsigned count = 0;
    for (const auto &test : cases) {
        for (uint8_t rrr = 0; rrr < 8; ++rrr) {
            ++count;
            if (!run_case(test, rrr))
                ++failures;
        }
    }
    std::printf("AGU BOUNDARY SUMMARY cases=%u failures=%u\n", count,
                failures);
    return failures == 0 ? 0 : 1;
}
