// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 model-specific DO/ENDDO decode fail-before oracle
//
// Motorola assigns DO and REP to neighboring encodings in the 0x06xxxx
// family.  This matrix admits every documented DO form, rejects the reserved
// EA/register sources and REP collisions, and fixes ENDDO at opcode 0x00008c.
///////////////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <cstdio>

#include "Vdsp56001_decode.h"
#include "verilated.h"

struct Vector {
    uint32_t opcode;
    bool is_do;
    bool is_enddo;
    const char *name;
};

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Vdsp56001_decode dut;
    dut.ext_word = 0x00e052;

    const Vector vectors[] = {
        {0x06cf00, true,  false, "DO B,expr"},
        {0x060380, true,  false, "DO #3,expr"},
        {0x064000, true,  false, "DO X:(R0)-N0,expr"},
        {0x060000, true,  false, "DO X:$00,expr"},
        {0x06cf20, false, false, "REP B collision"},
        {0x0603a0, false, false, "REP #3 collision"},
        {0x064020, false, false, "REP X:(R0)-N0 collision"},
        {0x060020, false, false, "REP X:$00 collision"},
        {0x067000, false, false, "reserved DO MMM=110"},
        {0x06c000, false, false, "reserved DO source 00"},
        {0x00008c, false, true,  "ENDDO"},
        {0x000005, false, false, "legacy false ENDDO"},
    };

    unsigned passed = 0;
    for (const auto &vector : vectors) {
        dut.inst = vector.opcode;
        dut.eval();
        const bool actual_do = dut.loop_is_do;
        const bool actual_enddo = dut.loop_is_enddo;
        const bool two_word_ok = !actual_do || dut.is_two_word;
        const bool disjoint = !(actual_do && dut.loop_is_rep);
        const bool ok = actual_do == vector.is_do &&
                        actual_enddo == vector.is_enddo &&
                        two_word_ok && disjoint;
        std::printf("%s %-31s opcode=%06x do=%u enddo=%u rep=%u two_word=%u\n",
                    ok ? "PASS" : "FAIL", vector.name, vector.opcode,
                    actual_do, actual_enddo,
                    static_cast<unsigned>(dut.loop_is_rep),
                    static_cast<unsigned>(dut.is_two_word));
        if (ok)
            ++passed;
    }

    const unsigned total = sizeof(vectors) / sizeof(vectors[0]);
    std::printf("%s DSP56001 DO/ENDDO decode matrix: %u/%u vectors\n",
                passed == total ? "PASS" : "FAIL", passed, total);
    return passed == total ? 0 : 1;
}

