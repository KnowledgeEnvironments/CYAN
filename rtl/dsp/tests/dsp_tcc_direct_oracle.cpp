// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

// Direct production-core DSP56001 conditional-transfer oracle.

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "Vdsp56001_core___024root.h"
#include "verilated.h"

namespace {

constexpr uint32_t kSuccessor = 0x3f5500; // MOVE #<$55,N7.
constexpr uint64_t kInitialA = 0x00112233445566ULL;
constexpr uint64_t kInitialB = 0x00aabbccddee00ULL;
constexpr uint32_t kX0 = 0x800001;
constexpr uint32_t kX1 = 0x123456;
constexpr uint32_t kY0 = 0x7fffff;
constexpr uint32_t kY1 = 0xfedcba;

struct Condition {
    const char *name;
    uint8_t code;
    uint8_t true_sr;
    uint8_t false_sr;
};

constexpr std::array<Condition, 16> kConditions{{
    {"CC", 0x0, 0x00, 0x01}, {"GE", 0x1, 0x00, 0x08},
    {"NE", 0x2, 0x00, 0x04}, {"PL", 0x3, 0x00, 0x08},
    {"NN", 0x4, 0x10, 0x00}, {"EC", 0x5, 0x00, 0x20},
    {"LC", 0x6, 0x00, 0x40}, {"GT", 0x7, 0x00, 0x04},
    {"CS", 0x8, 0x01, 0x00}, {"LT", 0x9, 0x08, 0x00},
    {"EQ", 0xa, 0x04, 0x00}, {"MI", 0xb, 0x08, 0x00},
    {"NR", 0xc, 0x00, 0x10}, {"ES", 0xd, 0x20, 0x00},
    {"LS", 0xe, 0x40, 0x00}, {"LE", 0xf, 0x04, 0x00},
}};

uint64_t data_source(uint8_t jjj, bool destination_b) {
    if (jjj == 0)
        return destination_b ? kInitialA : kInitialB;
    const uint32_t word = jjj == 4 ? kX0 : jjj == 5 ? kY0 :
                          jjj == 6 ? kX1 : kY1;
    return ((word & 0x800000) ? 0xff000000000000ULL : 0) |
           (uint64_t(word) << 24);
}

uint32_t tcc_opcode(uint8_t condition, uint8_t jjj, bool destination_b) {
    return 0x020000u | (uint32_t(condition) << 12) |
           (uint32_t(jjj) << 4) | (uint32_t(destination_b) << 3);
}

uint32_t tcc_dual_opcode(uint8_t condition, uint8_t jjj,
                         bool destination_b, uint8_t r_source,
                         uint8_t r_destination) {
    return 0x030000u | (uint32_t(condition) << 12) |
           (uint32_t(r_source) << 8) | (uint32_t(jjj) << 4) |
           (uint32_t(destination_b) << 3) | r_destination;
}

uint32_t move_long_immediate_r(uint8_t index) {
    return (uint32_t(0x60u | index) << 16) | 0xf400u;
}

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 64> p{};
    std::array<uint16_t, 8> expected_r{};
    const uint32_t instruction;
    uint64_t cycles = 0;
    int64_t decode_cycle = -1;
    int64_t writeback_cycle = -1;
    int64_t successor_cycle = -1;
    unsigned xr = 0, xw = 0, yr = 0, yw = 0;

    Harness(uint32_t opcode, uint8_t sr, bool dual = false,
            uint8_t r_source = 0, uint8_t r_destination = 0)
        : instruction(opcode) {
        unsigned pc = 0;
        if (dual) {
            for (unsigned i = 0; i < expected_r.size(); ++i)
                expected_r[i] = 0;
            expected_r[r_source] = uint16_t(0x1100 + r_source);
            p[pc++] = move_long_immediate_r(r_source);
            p[pc++] = expected_r[r_source];
            if (r_destination != r_source) {
                expected_r[r_destination] = uint16_t(0x8800 + r_destination);
                p[pc++] = move_long_immediate_r(r_destination);
                p[pc++] = expected_r[r_destination];
            }
            p[pc++] = 0x000000; // Address-register visibility spacing.
        }
        p[pc++] = instruction;
        p[pc++] = kSuccessor;
        p[pc] = 0x0c0000u | pc;

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
        debug_write(0x04, kX0);
        debug_write(0x05, kX1);
        debug_write(0x06, kY0);
        debug_write(0x07, kY1);
        debug_write(0x0e, kInitialA);
        debug_write(0x0f, kInitialB);
        debug_write(0x39, sr);
        core.run = 1;
    }

    void debug_write(uint8_t address, uint64_t data) {
        core.dbg_reg_wr_en = 1;
        core.dbg_reg_wr_addr = address;
        core.dbg_reg_wr_data = data;
        tick();
        core.dbg_reg_wr_en = 0;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch = core.p_fetch_en;
        const uint16_t address = core.p_fetch_addr;
        if (core.state_out == 2 &&
            (core.p_fetch_data & 0x00ffffff) == instruction &&
            decode_cycle < 0)
            decode_cycle = cycles;
        if (core.state_out == 4 && core.cur_inst_out == instruction &&
            writeback_cycle < 0)
            writeback_cycle = cycles;
        if (core.state_out == 4 && core.cur_inst_out == kSuccessor &&
            successor_cycle < 0)
            successor_cycle = cycles;
        xr += core.x_rd_en;
        xw += core.x_wr_en;
        yr += core.y_rd_en;
        yw += core.y_wr_en;
        core.clk = 1;
        core.eval();
        if (fetch)
            core.p_fetch_data = p[address & 0x3f] & 0x00ffffff;
        ++cycles;
    }

    bool finish() {
        for (unsigned i = 0; i < 160 && successor_cycle < 0; ++i)
            tick();
        return successor_cycle >= 0;
    }
};

bool common_ok(const Harness &h, uint8_t sr) {
    const int64_t clocks = h.decode_cycle >= 0 && h.writeback_cycle >= 0 ?
        h.writeback_cycle - h.decode_cycle + 1 : -1;
    const int64_t gap = h.writeback_cycle >= 0 && h.successor_cycle >= 0 ?
        h.successor_cycle - h.writeback_cycle : -1;
    return clocks == 2 && gap == 2 && h.core.sr_out == sr &&
           h.xr == 0 && h.xw == 0 && h.yr == 0 && h.yw == 0 &&
           h.core.rootp->dsp56001_core__DOT__n_reg[7] == 0x0055;
}

bool condition_case(const Condition &condition, bool taken) {
    const uint8_t sr = taken ? condition.true_sr : condition.false_sr;
    Harness h(tcc_opcode(condition.code, 4, false), sr);
    h.finish();
    const uint64_t expected_a = taken ? data_source(4, false) : kInitialA;
    const bool pass = common_ok(h, sr) && h.core.acc_a_out == expected_a &&
                      h.core.acc_b_out == kInitialB;
    std::printf("%s TCC condition %-2s %-5s A=%014llx/%014llx "
                "SR=%02x clocks=%lld gap=%lld bus=%u/%u/%u/%u\n",
                pass ? "PASS" : "FAIL", condition.name,
                taken ? "true" : "false",
                static_cast<unsigned long long>(h.core.acc_a_out),
                static_cast<unsigned long long>(expected_a), sr,
                static_cast<long long>(h.writeback_cycle - h.decode_cycle + 1),
                static_cast<long long>(h.successor_cycle - h.writeback_cycle),
                h.xr, h.xw, h.yr, h.yw);
    return pass;
}

bool selector_case(uint8_t jjj, bool destination_b) {
    Harness h(tcc_opcode(0, jjj, destination_b), 0);
    h.finish();
    const uint64_t expected = data_source(jjj, destination_b);
    const uint64_t expected_a = destination_b ? kInitialA : expected;
    const uint64_t expected_b = destination_b ? expected : kInitialB;
    const bool pass = common_ok(h, 0) && h.core.acc_a_out == expected_a &&
                      h.core.acc_b_out == expected_b;
    std::printf("%s TCC selector JJJ=%u D=%c A=%014llx/%014llx "
                "B=%014llx/%014llx\n", pass ? "PASS" : "FAIL", jjj,
                destination_b ? 'B' : 'A',
                static_cast<unsigned long long>(h.core.acc_a_out),
                static_cast<unsigned long long>(expected_a),
                static_cast<unsigned long long>(h.core.acc_b_out),
                static_cast<unsigned long long>(expected_b));
    return pass;
}

bool dual_case(uint8_t source, uint8_t destination) {
    Harness h(tcc_dual_opcode(0, 4, false, source, destination), 0, true,
              source, destination);
    const uint16_t source_value = h.expected_r[source];
    h.expected_r[destination] = source_value;
    h.finish();
    bool registers_ok = true;
    for (unsigned i = 0; i < h.expected_r.size(); ++i)
        registers_ok &= h.core.rootp->dsp56001_core__DOT__r_reg[i] ==
                        h.expected_r[i];
    const bool pass = common_ok(h, 0) && registers_ok &&
                      h.core.acc_a_out == data_source(4, false) &&
                      h.core.acc_b_out == kInitialB;
    std::printf("%s TCC dual R%u->R%u value=%04x/%04x all-R=%u\n",
                pass ? "PASS" : "FAIL", source, destination,
                h.core.rootp->dsp56001_core__DOT__r_reg[destination],
                source_value, registers_ok);
    return pass;
}

bool dual_false_case() {
    Harness h(tcc_dual_opcode(8, 4, false, 2, 5), 0, true, 2, 5);
    h.finish();
    bool registers_ok = true;
    for (unsigned i = 0; i < h.expected_r.size(); ++i)
        registers_ok &= h.core.rootp->dsp56001_core__DOT__r_reg[i] ==
                        h.expected_r[i];
    const bool pass = common_ok(h, 0) && registers_ok &&
                      h.core.acc_a_out == kInitialA &&
                      h.core.acc_b_out == kInitialB;
    std::printf("%s TCC dual false destinations-unchanged=%u\n",
                pass ? "PASS" : "FAIL", registers_ok);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    unsigned failures = 0;
    unsigned cases = 0;
    for (const auto &condition : kConditions) {
        failures += !condition_case(condition, true);
        failures += !condition_case(condition, false);
        cases += 2;
    }
    for (uint8_t jjj : {uint8_t(0), uint8_t(4), uint8_t(5), uint8_t(6),
                        uint8_t(7)}) {
        failures += !selector_case(jjj, false);
        failures += !selector_case(jjj, true);
        cases += 2;
    }
    for (uint8_t source = 0; source < 8; ++source) {
        for (uint8_t destination = 0; destination < 8; ++destination) {
            failures += !dual_case(source, destination);
            ++cases;
        }
    }
    failures += !dual_false_case();
    ++cases;
    std::printf("TCC DIRECT SUMMARY cases=%u failures=%u\n", cases, failures);
    return failures == 0 ? 0 : 1;
}
