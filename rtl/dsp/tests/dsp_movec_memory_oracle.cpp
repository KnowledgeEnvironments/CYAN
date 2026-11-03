// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

// Production-core DSP56001 MOVEC memory-form oracle.
// Covers short-absolute, register-indirect and long-absolute X/Y forms in
// both directions, with an architectural M0 readback after every transfer.
#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {
constexpr uint32_t kSetupM0 = 0x05f420;   // MOVEC #>imm,M0
constexpr uint32_t kReadM0ToA = 0x04e08e; // MOVEC M0,A
constexpr uint32_t kSetupM1 = 0x05f421;   // MOVEC #>imm,M1
constexpr uint32_t kReadM1ToA = 0x04e18e; // MOVEC M1,A
constexpr uint32_t kSetupR0 = 0x60f400;   // MOVE #>imm,R0
constexpr uint32_t kSetupN0 = 0x380300;   // MOVE #$03,N0
constexpr uint16_t kInitialM0 = 0x1357;
constexpr uint32_t kMemoryValue = 0xab4567;

struct Event { char space; bool write; uint16_t address; uint32_t data; };

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 65536> p{};
    std::array<uint32_t, 65536> x{};
    std::array<uint32_t, 65536> y{};
    std::vector<Event> events;

    Harness(uint32_t instruction, bool extension, uint16_t address) {
        p[0] = kSetupM0;
        p[1] = kInitialM0;
        p[2] = instruction;
        unsigned next = 3;
        if (extension) p[next++] = address;
        p[next++] = kReadM0ToA;
        p[next] = 0x0c0000 | next;

        core.clk = 0;
        core.rst_n = 0;
        core.clock_enable = 1;
        core.halt = 0;
        core.run = 0;
        core.p_fetch_data = 0;
        core.p_fetch_ready = 1;
        core.x_rdata = core.y_rdata = core.p_data_rdata = 0;
        core.x_ready = core.y_ready = 1;
        core.irq_a = core.irq_b = 0;
        core.irq_ssi_tx = core.irq_ssi_rx = 0;
        core.irq_ssi_tx_exception = core.irq_ssi_rx_exception = 0;
        core.irq_sci_tx = core.irq_sci_rx = 0;
        core.irq_sci_timer = core.irq_sci_rx_exception = core.irq_sci_idle = 0;
        core.irq_priority = 0;
        core.dbg_reg_wr_en = 0;
        core.dbg_reg_wr_addr = 0;
        core.dbg_reg_wr_data = 0;
        for (unsigned i = 0; i < 4; ++i) tick(false);
        core.rst_n = 1;
    }

    void tick(bool record = true) {
        core.clk = 0;
        core.eval();
        const bool fetch = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        const bool xr = core.x_rd_en, xw = core.x_wr_en;
        const bool yr = core.y_rd_en, yw = core.y_wr_en;
        const uint16_t xa = core.x_addr, ya = core.y_addr;
        const uint32_t xwd = core.x_wdata & 0xffffff;
        const uint32_t ywd = core.y_wdata & 0xffffff;
        core.clk = 1;
        core.eval();
        if (fetch) core.p_fetch_data = p[fetch_addr] & 0xffffff;
        if (xr) core.x_rdata = x[xa] & 0xffffff;
        if (yr) core.y_rdata = y[ya] & 0xffffff;
        if (xw) x[xa] = xwd;
        if (yw) y[ya] = ywd;
        if (record) {
            if (xr) events.push_back({'X', false, xa, x[xa] & 0xffffff});
            if (xw) events.push_back({'X', true, xa, xwd});
            if (yr) events.push_back({'Y', false, ya, y[ya] & 0xffffff});
            if (yw) events.push_back({'Y', true, ya, ywd});
        }
    }

    bool retire(uint32_t opcode) {
        core.run = 1;
        for (unsigned i = 0; i < 100; ++i) {
            if (core.state_out == 4 && core.cur_inst_out == opcode) {
                tick();
                return true;
            }
            tick();
        }
        return false;
    }
};

struct Case {
    const char *name;
    uint32_t opcode;
    bool extension;
    bool y_space;
    bool load;
    uint16_t address;
};

bool run(const Case &test) {
    Harness h(test.opcode, test.extension, test.address);
    (test.y_space ? h.y[test.address] : h.x[test.address]) = kMemoryValue;
    const bool setup = h.retire(kSetupM0);
    const bool retired = h.retire(test.opcode);
    const uint16_t after_pc = h.core.pc_out;
    const std::vector<Event> transfer_events = h.events;
    h.events.clear();
    const bool readback = h.retire(kReadM0ToA);
    const uint16_t m0 = (h.core.acc_a_out >> 24) & 0xffff;
    const uint16_t expected_m0 = test.load ? (kMemoryValue & 0xffff) : kInitialM0;
    const uint32_t expected_data = test.load ? kMemoryValue : kInitialM0;
    const char space = test.y_space ? 'Y' : 'X';
    const bool event_ok = transfer_events.size() == 1 &&
        transfer_events[0].space == space &&
        transfer_events[0].write == !test.load &&
        transfer_events[0].address == test.address &&
        transfer_events[0].data == expected_data;
    const uint32_t memory = test.y_space ? h.y[test.address] : h.x[test.address];
    const bool pass = setup && retired && readback &&
        after_pc == 2 + (test.extension ? 2 : 1) &&
        m0 == expected_m0 && event_ok &&
        memory == (test.load ? kMemoryValue : uint32_t(kInitialM0));
    std::printf("%s MOVEC %s opcode=%06x %c:%04x %s M0=%04x/%04x "
                "mem=%06x events=%zu pc=%04x\n",
                pass ? "PASS" : "FAIL", test.name, test.opcode, space,
                test.address, test.load ? "load" : "store", m0,
                expected_m0, memory, transfer_events.size(), after_pc);
    if (!pass) {
        for (unsigned i = 0; i < transfer_events.size(); ++i) {
            const Event &event = transfer_events[i];
            std::printf("  event[%u]=%c:%04x %s %06x\n", i,
                        event.space, event.address,
                        event.write ? "write" : "read", event.data);
        }
    }
    return pass;
}


struct AguMode {
    const char *name;
    uint8_t mmm;
    uint16_t address;
    uint16_t final_r0;
};

bool run_agu(const AguMode &mode, bool y_space, bool load) {
    constexpr uint16_t kInitialR0 = 0x0100;
    constexpr uint16_t kInitialN0 = 0x0003;
    constexpr uint32_t kInitialM1 = 0x001357;
    const uint8_t control = 0x01; // M1; M0 remains the R0 linear modifier.
    const uint8_t space = y_space ? 0x40 : 0x00;
    const uint32_t opcode = (load ? 0x05c000 : 0x054000) |
                            (uint32_t(mode.mmm) << 11) |
                            space | 0x20 | control;
    const uint32_t followup = 0x056021; // MOVEC M1,X:(R0)

    Harness h(opcode, false, 0);
    h.p[0] = kSetupM0;
    h.p[1] = 0xffff;       // M0 linear addressing.
    h.p[2] = kSetupM1;
    h.p[3] = kInitialM1;
    h.p[4] = kSetupR0;
    h.p[5] = kInitialR0;
    h.p[6] = kSetupN0;
    h.p[7] = 0x000000; // Settle the explicit N0 write before AGU use.
    h.p[8] = opcode;
    h.p[9] = followup;
    h.p[10] = kReadM1ToA;
    h.p[11] = 0x0c000b;
    (y_space ? h.y[mode.address] : h.x[mode.address]) = kMemoryValue;

    const bool setup = h.retire(kSetupM0) && h.retire(kSetupM1) &&
                       h.retire(kSetupR0) && h.retire(kSetupN0) &&
                       h.retire(0x000000);
    h.events.clear();
    const bool retired = h.retire(opcode);
    const std::vector<Event> transfer_events = h.events;
    h.events.clear();
    const bool followed = h.retire(followup);
    const std::vector<Event> followup_events = h.events;
    h.events.clear();
    const bool readback = h.retire(kReadM1ToA);

    const uint16_t expected_m1 = load ? (kMemoryValue & 0xffff) :
                                        uint16_t(kInitialM1);
    const uint32_t expected_data = load ? kMemoryValue : kInitialM1;
    const uint16_t m1 = (h.core.acc_a_out >> 24) & 0xffff;
    const char memory_space = y_space ? 'Y' : 'X';
    const bool transfer_ok = transfer_events.size() == 1 &&
        transfer_events[0].space == memory_space &&
        transfer_events[0].write == !load &&
        transfer_events[0].address == mode.address &&
        transfer_events[0].data == expected_data;
    const bool followup_ok = followup_events.size() == 1 &&
        followup_events[0].space == 'X' && followup_events[0].write &&
        followup_events[0].address == mode.final_r0 &&
        followup_events[0].data == expected_m1;
    const bool pass = setup && retired && followed && readback &&
        transfer_ok && followup_ok && m1 == expected_m1;
    std::printf("%s MOVEC agu-%s %c %s opcode=%06x access=%04x "
                "final-R0=%04x M1=%04x/%04x events=%zu/%zu\n",
                pass ? "PASS" : "FAIL", mode.name, memory_space,
                load ? "load" : "store", opcode, mode.address,
                mode.final_r0, m1, expected_m1, transfer_events.size(),
                followup_events.size());
    if (!pass) {
        for (unsigned i = 0; i < transfer_events.size(); ++i) {
            const Event &event = transfer_events[i];
            std::printf("  transfer[%u]=%c:%04x %s %06x\n", i,
                        event.space, event.address,
                        event.write ? "write" : "read", event.data);
        }
        for (unsigned i = 0; i < followup_events.size(); ++i) {
            const Event &event = followup_events[i];
            std::printf("  followup[%u]=%c:%04x %s %06x\n", i,
                        event.space, event.address,
                        event.write ? "write" : "read", event.data);
        }
    }
    return pass;
}
} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const Case cases[] = {
        {"short-X-load",  0x059220, false, false, true,  0x0012},
        {"short-X-store", 0x051220, false, false, false, 0x0012},
        {"short-Y-load",  0x059260, false, true,  true,  0x0012},
        {"short-Y-store", 0x051260, false, true,  false, 0x0012},
        {"direct-X-load",  0x05e020, false, false, true,  0x0000},
        {"direct-X-store", 0x056020, false, false, false, 0x0000},
        {"direct-Y-load",  0x05e060, false, true,  true,  0x0000},
        {"direct-Y-store", 0x056060, false, true,  false, 0x0000},
        {"long-X-load",   0x05f020, true,  false, true,  0x1234},
        {"long-X-store",  0x057020, true,  false, false, 0x1234},
        {"long-Y-load",   0x05f060, true,  true,  true,  0x1234},
        {"long-Y-store",  0x057060, true,  true,  false, 0x1234},
    };
    unsigned failures = 0;
    for (const Case &test : cases) failures += !run(test);
    const AguMode modes[] = {
        {"postminus-n", 0, 0x0100, 0x00fd},
        {"postplus-n",  1, 0x0100, 0x0103},
        {"postminus",   2, 0x0100, 0x00ff},
        {"postplus",    3, 0x0100, 0x0101},
        {"direct",      4, 0x0100, 0x0100},
        {"indexed",     5, 0x0103, 0x0100},
        {"preminus",    7, 0x00ff, 0x00ff},
    };
    for (const AguMode &mode : modes) {
        for (bool y_space : {false, true}) {
            failures += !run_agu(mode, y_space, false);
            failures += !run_agu(mode, y_space, true);
        }
    }
    const size_t cases_run = sizeof(cases) / sizeof(cases[0]) +
                             sizeof(modes) / sizeof(modes[0]) * 4;
    std::printf("MOVEC MEMORY SUMMARY cases=%zu failures=%u\n",
                cases_run, failures);
    return failures ? 1 : 0;
}
