// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

// Production-core DSP56001 MOVEP form-family oracle.
//
// DSP56001UMA3 A-250 defines register, P-memory, X/Y-memory and immediate
// peripheral moves.  This exercises both transfer directions and both
// peripheral spaces without a private decoder or architectural force.
#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "Vdsp56001_core.h"
#include "Vdsp56001_core___024root.h"
#include "verilated.h"

namespace {
constexpr uint8_t kWriteback = 4;
constexpr uint16_t kXTarget = 0x0123;
constexpr uint16_t kYTarget = 0x0234;
constexpr uint16_t kPTarget = 0x0345;
constexpr uint32_t kSource = 0x654321;
constexpr uint32_t kPeripheral = 0x13579b;

struct AguMode {
    const char *name;
    uint8_t mmm;
    uint16_t address;
};

constexpr std::array<AguMode, 7> kAguModes{{
    {"postminus-n", 0, 0x0100},
    {"postplus-n",  1, 0x0100},
    {"postminus",   2, 0x0100},
    {"postplus",    3, 0x0100},
    {"direct",      4, 0x0100},
    {"indexed",     5, 0x0103},
    {"preminus",    7, 0x00ff},
}};

struct Event {
    char space;
    bool write;
    uint16_t address;
    uint32_t data;
};

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 65536> p{};
    std::array<uint32_t, 65536> x{};
    std::array<uint32_t, 65536> y{};
    std::vector<Event> events;

    Harness(uint32_t opcode, uint32_t extension, bool two_word) {
        p[0] = opcode;
        p[1] = two_word ? extension : 0x000000;
        p[two_word ? 2 : 1] = 0x0c0001 + (two_word ? 1 : 0); // JMP to self.

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
        core.irq_a = core.irq_b = 0;
        core.irq_ssi_tx = core.irq_ssi_rx = 0;
        core.irq_ssi_tx_exception = core.irq_ssi_rx_exception = 0;
        core.irq_sci_tx = core.irq_sci_rx = 0;
        core.irq_sci_timer = 0;
        core.irq_sci_rx_exception = 0;
        core.irq_sci_idle = 0;
        core.irq_priority = 0;
        core.dbg_reg_wr_en = 0;
        core.dbg_reg_wr_addr = 0;
        core.dbg_reg_wr_data = 0;
        for (unsigned i = 0; i < 4; ++i) tick(false);
        core.rst_n = 1;
    }

    void debug_write(uint8_t reg, uint64_t value) {
        core.dbg_reg_wr_addr = reg;
        core.dbg_reg_wr_data = value;
        core.dbg_reg_wr_en = 1;
        tick(false);
        core.dbg_reg_wr_en = 0;
    }

    void tick(bool record = true) {
        core.clk = 0;
        core.eval();
        const bool fetch = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        const bool pr = core.p_data_rd_en;
        const bool pw = core.p_data_wr_en;
        const bool xr = core.x_rd_en;
        const bool xw = core.x_wr_en;
        const bool yr = core.y_rd_en;
        const bool yw = core.y_wr_en;
        const uint16_t pa = core.p_data_addr;
        const uint16_t xa = core.x_addr;
        const uint16_t ya = core.y_addr;
        const uint32_t pwd = core.p_data_wdata & 0xffffff;
        const uint32_t xwd = core.x_wdata & 0xffffff;
        const uint32_t ywd = core.y_wdata & 0xffffff;

        core.clk = 1;
        core.eval();
        if (fetch) core.p_fetch_data = p[fetch_addr] & 0xffffff;
        if (pr) core.p_data_rdata = p[pa] & 0xffffff;
        if (xr) core.x_rdata = x[xa] & 0xffffff;
        if (yr) core.y_rdata = y[ya] & 0xffffff;
        if (pw) p[pa] = pwd;
        if (xw) x[xa] = xwd;
        if (yw) y[ya] = ywd;
        if (record) {
            if (pr) events.push_back({'P', false, pa, p[pa] & 0xffffff});
            if (pw) events.push_back({'P', true, pa, pwd});
            if (xr) events.push_back({'X', false, xa, x[xa] & 0xffffff});
            if (xw) events.push_back({'X', true, xa, xwd});
            if (yr) events.push_back({'Y', false, ya, y[ya] & 0xffffff});
            if (yw) events.push_back({'Y', true, ya, ywd});
        }
    }

    bool retire(uint32_t opcode) {
        core.run = 1;
        for (unsigned i = 0; i < 100; ++i) {
            if (core.state_out == kWriteback &&
                core.cur_inst_out == opcode) {
                tick();
                return true;
            }
            tick();
        }
        return false;
    }
};

uint16_t peripheral_address(uint8_t pp) { return 0xffc0 | pp; }

bool exact_events(const Harness &h, std::initializer_list<Event> expected) {
    if (h.events.size() != expected.size()) return false;
    unsigned i = 0;
    for (const Event &want : expected) {
        const Event &got = h.events[i++];
        if (got.space != want.space || got.write != want.write ||
            got.address != want.address || got.data != want.data)
            return false;
    }
    return true;
}

struct SelectorCase {
    const char *name;
    uint8_t code;
    uint32_t input;
    uint32_t expected;
    uint8_t expected_sp;
};

bool register_selector_roundtrip(const SelectorCase &test,
                                 bool y_peripheral) {
    // DSP56001UMA2 A-164--A-170: MOVEP register forms admit every assigned
    // six-bit architectural register code.  Read one peripheral word into
    // the register, allow the documented move pipeline spacing, then write
    // the register to a second peripheral address.
    constexpr uint8_t kInputPp = 0x2c;
    constexpr uint8_t kOutputPp = 0x2d;
    const uint32_t read = (y_peripheral ? 0x094000u : 0x084000u) |
                          (uint32_t(test.code) << 8) | kInputPp;
    const uint32_t write = (y_peripheral ? 0x09c000u : 0x08c000u) |
                           (uint32_t(test.code) << 8) | kOutputPp;
    Harness h(read, 0, false);
    h.p[0] = read;
    h.p[1] = 0x000000; // Required move pipeline separation.
    h.p[2] = write;
    h.p[3] = 0x0c0003; // JMP $0003.
    const uint16_t input_address = peripheral_address(kInputPp);
    const uint16_t output_address = peripheral_address(kOutputPp);
    auto &memory = y_peripheral ? h.y : h.x;
    memory[input_address] = test.input;
    memory[output_address] = 0x5a5a5a;
    const bool loaded = h.retire(read);
    const bool stored = h.retire(write);
    const char space = y_peripheral ? 'Y' : 'X';
    const bool pass = loaded && stored &&
        memory[output_address] == test.expected &&
        h.core.rootp->dsp56001_core__DOT__sp == test.expected_sp &&
        exact_events(h, {{space, false, input_address, test.input},
                         {space, true, output_address, test.expected}});
    std::printf("%s MOVEP selector %-3s %c code=%02x data=%06x/%06x "
                "events=%zu SP=%02x/%02x retired=%u/%u\n",
                pass ? "PASS" : "FAIL", test.name, space, test.code,
                memory[output_address], test.expected, h.events.size(),
                h.core.rootp->dsp56001_core__DOT__sp, test.expected_sp,
                loaded, stored);
    return pass;
}

bool reg_to_peripheral(bool y_peripheral) {
    constexpr uint8_t pp = 0x21;
    const uint32_t op = (y_peripheral ? 0x09c400 : 0x08c400) | pp;
    Harness h(op, 0, false);
    h.debug_write(0x04, kSource); // X0
    const bool retired = h.retire(op);
    const char space = y_peripheral ? 'Y' : 'X';
    const bool pass = retired && h.core.pc_out == 1 &&
        exact_events(h, {{space, true, peripheral_address(pp), kSource}});
    std::printf("%s MOVEP X0,%c:pp opcode=%06x events=%zu pc=%04x\n",
                pass ? "PASS" : "FAIL", space, op, h.events.size(),
                h.core.pc_out);
    return pass;
}

bool peripheral_to_reg(bool y_peripheral) {
    constexpr uint8_t pp = 0x22;
    const uint32_t op = (y_peripheral ? 0x094700 : 0x084700) | pp;
    constexpr uint32_t readback = 0x200000 | (0x07 << 13) | (0x0e << 8);
    Harness h(op, 0, false);
    h.p[1] = readback;                 // MOVE Y1,A
    h.p[2] = 0x0c0002;                // JMP $002
    const uint16_t pa = peripheral_address(pp);
    (y_peripheral ? h.y[pa] : h.x[pa]) = kPeripheral;
    const bool retired = h.retire(op) && h.retire(readback);
    const char space = y_peripheral ? 'Y' : 'X';
    const uint32_t value = (h.core.acc_a_out >> 24) & 0xffffff;
    const bool pass = retired && h.core.pc_out == 2 && value == kPeripheral &&
        exact_events(h, {{space, false, pa, kPeripheral}});
    std::printf("%s MOVEP %c:pp,Y1 opcode=%06x value=%06x events=%zu pc=%04x\n",
                pass ? "PASS" : "FAIL", space, op, value,
                h.events.size(), h.core.pc_out);
    return pass;
}

bool p_to_peripheral(bool y_peripheral) {
    constexpr uint8_t pp = 0x23;
    const uint32_t op = (y_peripheral ? 0x09f040 : 0x08f040) | pp;
    Harness h(op, kPTarget, true);
    h.p[kPTarget] = kSource;
    const bool retired = h.retire(op);
    const char space = y_peripheral ? 'Y' : 'X';
    const bool pass = retired && h.core.pc_out == 2 &&
        exact_events(h, {{'P', false, kPTarget, kSource},
                         {space, true, peripheral_address(pp), kSource}});
    std::printf("%s MOVEP P:abs,%c:pp opcode=%06x events=%zu pc=%04x\n",
                pass ? "PASS" : "FAIL", space, op, h.events.size(),
                h.core.pc_out);
    return pass;
}

bool peripheral_to_p(bool y_peripheral) {
    constexpr uint8_t pp = 0x24;
    const uint32_t op = (y_peripheral ? 0x097040 : 0x087040) | pp;
    Harness h(op, kPTarget, true);
    const uint16_t pa = peripheral_address(pp);
    (y_peripheral ? h.y[pa] : h.x[pa]) = kPeripheral;
    const bool retired = h.retire(op);
    const char space = y_peripheral ? 'Y' : 'X';
    const bool pass = retired && h.core.pc_out == 2 &&
        h.p[kPTarget] == kPeripheral &&
        exact_events(h, {{space, false, pa, kPeripheral},
                         {'P', true, kPTarget, kPeripheral}});
    std::printf("%s MOVEP %c:pp,P:abs opcode=%06x value=%06x events=%zu pc=%04x\n",
                pass ? "PASS" : "FAIL", space, op, h.p[kPTarget],
                h.events.size(), h.core.pc_out);
    return pass;
}

bool memory_to_peripheral(bool y_memory, bool y_peripheral) {
    constexpr uint8_t pp = 0x25;
    const uint16_t target = y_memory ? kYTarget : kXTarget;
    const uint32_t op = (y_peripheral ? 0x09f080 : 0x08f080) |
                        (y_memory ? 0x40 : 0) | pp;
    Harness h(op, target, true);
    (y_memory ? h.y[target] : h.x[target]) = kSource;
    const bool retired = h.retire(op);
    const char ms = y_memory ? 'Y' : 'X';
    const char ps = y_peripheral ? 'Y' : 'X';
    const bool pass = retired && h.core.pc_out == 2 &&
        exact_events(h, {{ms, false, target, kSource},
                         {ps, true, peripheral_address(pp), kSource}});
    std::printf("%s MOVEP %c:abs,%c:pp opcode=%06x events=%zu pc=%04x\n",
                pass ? "PASS" : "FAIL", ms, ps, op, h.events.size(),
                h.core.pc_out);
    return pass;
}

bool peripheral_to_memory(bool y_peripheral, bool y_memory) {
    constexpr uint8_t pp = 0x26;
    const uint16_t target = y_memory ? kYTarget : kXTarget;
    const uint32_t op = (y_peripheral ? 0x097080 : 0x087080) |
                        (y_memory ? 0x40 : 0) | pp;
    Harness h(op, target, true);
    const uint16_t pa = peripheral_address(pp);
    (y_peripheral ? h.y[pa] : h.x[pa]) = kPeripheral;
    const bool retired = h.retire(op);
    const char ps = y_peripheral ? 'Y' : 'X';
    const char ms = y_memory ? 'Y' : 'X';
    const uint32_t value = y_memory ? h.y[target] : h.x[target];
    const bool pass = retired && h.core.pc_out == 2 && value == kPeripheral &&
        exact_events(h, {{ps, false, pa, kPeripheral},
                         {ms, true, target, kPeripheral}});
    std::printf("%s MOVEP %c:pp,%c:abs opcode=%06x value=%06x events=%zu pc=%04x\n",
                pass ? "PASS" : "FAIL", ps, ms, op, value,
                h.events.size(), h.core.pc_out);
    return pass;
}

bool immediate_to_peripheral(bool y_peripheral) {
    constexpr uint8_t pp = 0x27;
    const uint32_t op = (y_peripheral ? 0x09f480 : 0x08f480) | pp;
    Harness h(op, kSource, true);
    const char space = y_peripheral ? 'Y' : 'X';
    const bool retired = h.retire(op);
    const bool pass = retired && h.core.pc_out == 2 &&
        exact_events(h, {{space, true, peripheral_address(pp), kSource}});
    std::printf("%s MOVEP #imm,%c:pp opcode=%06x events=%zu pc=%04x\n",
                pass ? "PASS" : "FAIL", space, op, h.events.size(),
                h.core.pc_out);
    return pass;
}

bool setup_agu(Harness &h, uint32_t opcode) {
    // Configure the AGU through architectural instructions.  The debug port
    // deliberately exposes data/accumulator/SR state only; pretending it can
    // write R/N/M would leave R0 at reset and invalidate the oracle.
    constexpr uint32_t kSetR0 = 0x60f400; // MOVE #>imm,R0 (two words)
    constexpr uint32_t kSetN0 = 0x380300; // MOVE #$03,N0
    h.p[0] = kSetR0;
    h.p[1] = 0x000100;
    h.p[2] = kSetN0;
    h.p[3] = 0x000000; // Settle the explicit N0 write before AGU use.
    h.p[4] = opcode;
    h.p[5] = 0x0c0005; // JMP $005
    const bool setup = h.retire(kSetR0) && h.retire(kSetN0) &&
                       h.retire(0x000000);
    h.events.clear();
    return setup;
}

uint32_t movep_ea_opcode(const AguMode &mode, bool y_peripheral,
                         bool p_memory, bool y_memory,
                         bool memory_to_peripheral, uint8_t pp) {
    return (y_peripheral ? 0x090000 : 0x080000) |
           (memory_to_peripheral ? 0x008000 : 0) | 0x004000 |
           (uint32_t(mode.mmm) << 11) |
           (p_memory ? 0x40 : (0x80 | (y_memory ? 0x40 : 0))) | pp;
}

bool memory_ea_to_peripheral(const AguMode &mode, bool p_memory,
                             bool y_memory, bool y_peripheral) {
    constexpr uint8_t pp = 0x2a;
    const uint32_t op = movep_ea_opcode(mode, y_peripheral, p_memory,
                                        y_memory, true, pp);
    Harness h(op, 0, false);
    const bool setup = setup_agu(h, op);
    if (p_memory)
        h.p[mode.address] = kSource;
    else if (y_memory)
        h.y[mode.address] = kSource;
    else
        h.x[mode.address] = kSource;
    const bool retired = h.retire(op);
    const char memory_space = p_memory ? 'P' : y_memory ? 'Y' : 'X';
    const char peripheral_space = y_peripheral ? 'Y' : 'X';
    const bool pass = setup && retired && h.core.pc_out == 5 &&
        exact_events(h, {{memory_space, false, mode.address, kSource},
                         {peripheral_space, true, peripheral_address(pp),
                          kSource}});
    std::printf("%s MOVEP %c:(R0)%s,%c:pp opcode=%06x events=%zu pc=%04x\n",
                pass ? "PASS" : "FAIL", memory_space, mode.name,
                peripheral_space, op, h.events.size(), h.core.pc_out);
    return pass;
}

bool peripheral_to_memory_ea(const AguMode &mode, bool y_peripheral,
                             bool p_memory, bool y_memory) {
    constexpr uint8_t pp = 0x2b;
    const uint32_t op = movep_ea_opcode(mode, y_peripheral, p_memory,
                                        y_memory, false, pp);
    Harness h(op, 0, false);
    const bool setup = setup_agu(h, op);
    const uint16_t pa = peripheral_address(pp);
    (y_peripheral ? h.y[pa] : h.x[pa]) = kPeripheral;
    const bool retired = h.retire(op);
    const char peripheral_space = y_peripheral ? 'Y' : 'X';
    const char memory_space = p_memory ? 'P' : y_memory ? 'Y' : 'X';
    const uint32_t value = p_memory ? h.p[mode.address] :
                           y_memory ? h.y[mode.address] : h.x[mode.address];
    const bool pass = setup && retired && h.core.pc_out == 5 &&
        value == kPeripheral &&
        exact_events(h, {{peripheral_space, false, pa, kPeripheral},
                         {memory_space, true, mode.address, kPeripheral}});
    std::printf("%s MOVEP %c:pp,%c:(R0)%s opcode=%06x value=%06x "
                "events=%zu pc=%04x\n",
                pass ? "PASS" : "FAIL", peripheral_space, memory_space,
                mode.name, op, value, h.events.size(), h.core.pc_out);
    return pass;
}
} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    unsigned failures = 0;
    for (bool yp : {false, true}) failures += !reg_to_peripheral(yp);
    for (bool yp : {false, true}) failures += !peripheral_to_reg(yp);
    for (bool yp : {false, true}) failures += !p_to_peripheral(yp);
    for (bool yp : {false, true}) failures += !peripheral_to_p(yp);
    for (bool ym : {false, true})
        for (bool yp : {false, true})
            failures += !memory_to_peripheral(ym, yp);
    for (bool yp : {false, true})
        for (bool ym : {false, true})
            failures += !peripheral_to_memory(yp, ym);
    for (bool yp : {false, true}) failures += !immediate_to_peripheral(yp);
    unsigned ea_cases = 0;
    for (const AguMode &mode : kAguModes) {
        for (bool yp : {false, true}) {
            failures += !memory_ea_to_peripheral(mode, true, false, yp);
            ++ea_cases;
            failures += !peripheral_to_memory_ea(mode, yp, true, false);
            ++ea_cases;
            for (bool ym : {false, true}) {
                failures += !memory_ea_to_peripheral(mode, false, ym, yp);
                ++ea_cases;
                failures += !peripheral_to_memory_ea(mode, yp, false, ym);
                ++ea_cases;
            }
        }
    }
    std::vector<SelectorCase> selectors;
    const char *data_names[] = {
        "X0", "X1", "Y0", "Y1", "A0", "B0", "A2", "B2",
        "A1", "B1", "A", "B"
    };
    for (unsigned i = 0; i < 12; ++i) {
        const uint8_t code = 0x04 + i;
        const uint32_t input = (code == 0x0a || code == 0x0b) ?
                               0xabcdef : 0x654321;
        const uint32_t expected = (code == 0x0a || code == 0x0b) ?
                                  0x0000ef : input;
        selectors.push_back({data_names[i], code, input, expected, 0});
    }
    for (unsigned bank = 0; bank < 3; ++bank)
        for (unsigned i = 0; i < 8; ++i) {
            static const char *prefix[] = {"R", "N", "M"};
            static char names[3][8][4];
            std::snprintf(names[bank][i], sizeof(names[bank][i]), "%s%u",
                          prefix[bank], i);
            selectors.push_back({names[bank][i], uint8_t(0x10 + bank * 8 + i),
                                 0xabcdef, 0x00cdef, 0});
        }
    const SelectorCase controls[] = {
        {"SR",  0x39, 0x000035, 0x000035, 0},
        {"OMR", 0x3a, 0x12ab55, 0x000055, 0},
        {"SP",  0x3b, 0x000003, 0x000003, 3},
        {"SSH", 0x3c, 0xabcdef, 0x00cdef, 0},
        {"SSL", 0x3d, 0xabcdef, 0x00cdef, 0},
        {"LA",  0x3e, 0xabcdef, 0x00cdef, 0},
        {"LC",  0x3f, 0xabcdef, 0x00cdef, 0},
    };
    for (const auto &test : controls) selectors.push_back(test);
    unsigned selector_cases = 0;
    for (const auto &test : selectors)
        for (bool yp : {false, true}) {
            failures += !register_selector_roundtrip(test, yp);
            ++selector_cases;
        }
    std::printf("MOVEP SUMMARY cases=%u failures=%u (18 fixed/absolute, "
                "%u ordinary-EA, %u register-selector)\n",
                18 + ea_cases + selector_cases, failures, ea_cases,
                selector_cases);
    return failures ? 1 : 0;
}
