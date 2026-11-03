// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// Deterministic DSP56001 AGU differential oracle
//
// Exercise the synthesizable AGU against an independent integer reference
// across legal linear, modulo, modulo block-jump and reverse-carry operands.
// The seed and case generation are fixed so every failure is reproducible.
///////////////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <cstdio>

#include "Vdsp56001_agu.h"
#include "verilated.h"

namespace {

uint32_t rng_state = 0x56001a6u;

uint32_t next_random() {
    uint32_t x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state = x;
    return x;
}

uint16_t reverse16(uint16_t value) {
    value = static_cast<uint16_t>(((value & 0xaaaau) >> 1) |
                                  ((value & 0x5555u) << 1));
    value = static_cast<uint16_t>(((value & 0xccccu) >> 2) |
                                  ((value & 0x3333u) << 2));
    value = static_cast<uint16_t>(((value & 0xf0f0u) >> 4) |
                                  ((value & 0x0f0fu) << 4));
    return static_cast<uint16_t>((value >> 8) | (value << 8));
}

uint16_t modulo_mask(uint16_t modifier) {
    uint16_t mask = modifier;
    mask = static_cast<uint16_t>(mask | (mask >> 1));
    mask = static_cast<uint16_t>(mask | (mask >> 2));
    mask = static_cast<uint16_t>(mask | (mask >> 4));
    mask = static_cast<uint16_t>(mask | (mask >> 8));
    return mask;
}

struct Expected {
    uint16_t effective;
    uint16_t updated;
    bool write;
    bool valid;
};

Expected reference(uint16_t r, uint16_t n, uint16_t m, uint8_t mode) {
    const bool use_n = mode == 0 || mode == 1 || mode == 5;
    int32_t offset = 0;
    if (mode == 0)
        offset = -static_cast<int32_t>(n);
    else if (mode == 1 || mode == 5)
        offset = n;
    else if (mode == 2 || mode == 7)
        offset = -1;
    else if (mode == 3)
        offset = 1;

    uint16_t updated = r;
    if (m == 0xffffu) {
        updated = static_cast<uint16_t>(static_cast<int32_t>(r) + offset);
    } else if (m == 0x0000u) {
        const uint16_t reversed_r = reverse16(r);
        const uint16_t reversed_delta = use_n ? reverse16(n) : 0x8000u;
        const uint16_t reversed_result = offset < 0
            ? static_cast<uint16_t>(reversed_r - reversed_delta)
            : static_cast<uint16_t>(reversed_r + reversed_delta);
        updated = reverse16(reversed_result);
    } else {
        const uint16_t mask = modulo_mask(m);
        const uint16_t lower = static_cast<uint16_t>(r & ~mask);
        const bool block_jump = use_n && n != 0 && (n & mask) == 0;
        if (block_jump) {
            updated = static_cast<uint16_t>(static_cast<int32_t>(r) + offset);
        } else {
            int32_t relative = static_cast<int32_t>(r) - lower + offset;
            const int32_t size = static_cast<int32_t>(m) + 1;
            if (relative < 0)
                relative += size;
            else if (relative > m)
                relative -= size;
            updated = static_cast<uint16_t>(lower + relative);
        }
    }

    Expected expected{};
    expected.write = mode != 4 && mode != 5 && mode != 6;
    expected.valid = mode != 6;
    expected.updated = updated;
    expected.effective = mode == 5 ? updated : (mode == 7 ? updated : r);
    return expected;
}

bool check(Vdsp56001_agu &agu, uint16_t r, uint16_t n, uint16_t m,
           uint8_t mode, uint8_t rrr, unsigned ordinal) {
    const Expected expected = reference(r, n, m, mode);
    agu.r_in = r;
    agu.n_in = n;
    agu.m_in = m;
    agu.agu_mmm = mode;
    agu.agu_rrr = rrr;
    agu.eval();

    const bool pass = agu.eff_addr == expected.effective &&
                      (!expected.write || agu.r_out == expected.updated) &&
                      agu.r_wr_en == expected.write &&
                      agu.r_wr_idx == rrr &&
                      agu.eff_addr_valid == expected.valid;
    if (!pass) {
        std::printf(
            "FAIL AGU differential ordinal=%u r=%04x n=%04x m=%04x "
            "mode=%u R%u ea=%04x/%04x update=%04x/%04x wr=%u/%u "
            "valid=%u/%u\n",
            ordinal, r, n, m, mode, rrr, agu.eff_addr,
            expected.effective, agu.r_out, expected.updated, agu.r_wr_en,
            expected.write, agu.eff_addr_valid, expected.valid);
    }
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Vdsp56001_agu agu;
    unsigned cases = 0;
    unsigned failures = 0;

    auto run = [&](uint16_t r, uint16_t n, uint16_t m, uint8_t mode) {
        const uint8_t rrr = static_cast<uint8_t>(next_random() & 7u);
        ++cases;
        if (!check(agu, r, n, m, mode, rrr, cases) && ++failures >= 20)
            return false;
        return true;
    };

    for (unsigned i = 0; i < 8192 && failures < 20; ++i) {
        if (!run(static_cast<uint16_t>(next_random()),
                 static_cast<uint16_t>(next_random()), 0xffffu,
                 static_cast<uint8_t>(next_random() & 7u)))
            break;
    }

    for (unsigned i = 0; i < 8192 && failures < 20; ++i) {
        if (!run(static_cast<uint16_t>(next_random()),
                 static_cast<uint16_t>(next_random()), 0x0000u,
                 static_cast<uint8_t>(next_random() & 7u)))
            break;
    }

    const uint16_t modifiers[] = {
        1, 2, 3, 4, 5, 7, 8, 15, 16, 20, 31, 32, 63,
        64, 127, 255, 256, 511, 512, 1023, 2047, 4095,
        8191, 16383, 32767,
    };
    for (uint16_t m : modifiers) {
        const uint16_t mask = modulo_mask(m);
        for (unsigned i = 0; i < 256 && failures < 20; ++i) {
            const uint16_t base = static_cast<uint16_t>(next_random()) &
                                  static_cast<uint16_t>(~mask);
            const uint16_t r = static_cast<uint16_t>(
                base + next_random() % (static_cast<uint32_t>(m) + 1u));
            const uint16_t n = static_cast<uint16_t>(
                next_random() % (static_cast<uint32_t>(m) + 1u));
            for (uint8_t mode = 0; mode < 8 && failures < 20; ++mode) {
                if (!run(r, n, m, mode))
                    break;
            }
        }
    }

    for (uint16_t m : modifiers) {
        const uint16_t mask = modulo_mask(m);
        const uint32_t step = static_cast<uint32_t>(mask) + 1u;
        const uint32_t max_multiple = 0xffffu / step;
        if (max_multiple == 0)
            continue;
        for (unsigned i = 0; i < 128 && failures < 20; ++i) {
            const uint16_t base = static_cast<uint16_t>(next_random()) &
                                  static_cast<uint16_t>(~mask);
            const uint16_t r = static_cast<uint16_t>(
                base + next_random() % (static_cast<uint32_t>(m) + 1u));
            const uint16_t n = static_cast<uint16_t>(
                step * (1u + next_random() % max_multiple));
            const uint8_t modes[] = {0, 1, 5};
            for (uint8_t mode : modes) {
                if (!run(r, n, m, mode))
                    break;
            }
        }
    }

    std::printf("PASS AGU deterministic differential cases=%u failures=%u "
                "seed=056001a6\n", cases, failures);
    return failures == 0 ? 0 : 1;
}
