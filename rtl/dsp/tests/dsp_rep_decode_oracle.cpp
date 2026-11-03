// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 model-specific REP decode oracle
//
// Motorola assigns REP and DO to neighboring encodings in the 0x06xxxx
// family.  This oracle prevents a broad family match, rejects reserved source
// fields, and proves that every admitted REP remains a one-word instruction.
///////////////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <cstdio>

#include "Vdsp56001_decode.h"
#include "verilated.h"

struct Vector {
    uint32_t opcode;
    bool rep;
    const char *name;
};

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Vdsp56001_decode dut;
    dut.ext_word = 0;

    const Vector vectors[] = {
        {0x06ce20, true,  "REP A"},
        {0x06cf20, true,  "REP B"},
        {0x0603a0, true,  "REP #3"},
        {0x064020, true,  "REP X:(R0)-N0"},
        {0x060020, true,  "REP X:$00"},
        {0x06cf00, false, "DO B collision"},
        {0x060380, false, "DO #3 collision"},
        {0x064000, false, "DO X:(R0)-N0 collision"},
        {0x060000, false, "DO X:$00 collision"},
        {0x067020, false, "reserved REP MMM=110"},
        {0x06c020, false, "reserved REP source 00"},
    };

    unsigned passed = 0;
    for (const auto &vector : vectors) {
        dut.inst = vector.opcode;
        dut.eval();
        const bool actual_rep = dut.loop_is_rep;
        const bool valid_one_word = !actual_rep || !dut.is_two_word;
        const bool ok = actual_rep == vector.rep && valid_one_word;
        std::printf("%s %-29s opcode=%06x rep=%u two_word=%u\n",
                    ok ? "PASS" : "FAIL", vector.name, vector.opcode,
                    actual_rep, static_cast<unsigned>(dut.is_two_word));
        if (ok)
            ++passed;
    }

    const unsigned total = sizeof(vectors) / sizeof(vectors[0]);
    std::printf("%s DSP56001 REP decode matrix: %u/%u vectors\n",
                passed == total ? "PASS" : "FAIL", passed, total);
    return passed == total ? 0 : 1;
}

