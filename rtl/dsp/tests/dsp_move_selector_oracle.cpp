// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 ordinary parallel-MOVE register-selector oracle
//
// DSP56001UMA3 Tables A-20 and A-31 define 28 five-bit register codes for
// immediate, register-to-register, and ordinary X/Y data moves. Exercise every
// assigned code as both source and destination in both memory-space encodings
// through the production core. The memory writeback makes each result public
// without peeking into the internal register file.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint8_t kStateWritebackFetch = 4;
constexpr uint32_t kMemoryData = 0x456789;
constexpr uint32_t kStoreData = 0x234567;
constexpr uint32_t kImmediateData = 0x345678;

uint32_t memory_move(unsigned reg, bool y_space, bool load,
                     unsigned mmm, unsigned rrr) {
    return ((0x40u | ((reg >> 3) << 4) | (y_space ? 8u : 0u) |
             (reg & 7u)) << 16) |
           (load ? 0x8000u : 0u) | 0x4000u | (mmm << 11) | (rrr << 8);
}

uint32_t register_move(unsigned source, unsigned destination) {
    return 0x200000u | (source << 13) | (destination << 8);
}

uint32_t register_word(unsigned code, uint32_t word) {
    if (code == 0x0a || code == 0x0b)
        return word & 0x0000ffu; // A2/B2 are eight bits.
    if (code >= 0x10)
        return word & 0x00ffffu; // Rn/Nn/Mn are sixteen bits.
    return word & 0x00ffffffu;
}

class Harness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 32> p{};
    std::array<uint32_t, 256> x{};
    std::array<uint32_t, 256> y{};
    unsigned x_reads = 0;
    unsigned x_writes = 0;
    unsigned y_reads = 0;
    unsigned y_writes = 0;
    unsigned p_reads = 0;
    unsigned p_writes = 0;
    uint16_t last_addr = 0;
    uint16_t last_read_addr = 0;
    uint32_t last_data = 0;

    Harness() {
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
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        const bool x_read = core.x_rd_en;
        const bool y_read = core.y_rd_en;
        const bool x_write = core.x_wr_en;
        const bool y_write = core.y_wr_en;
        const uint16_t x_addr = core.x_addr;
        const uint16_t y_addr = core.y_addr;
        const uint32_t x_wdata = core.x_wdata;
        const uint32_t y_wdata = core.y_wdata;
        if (x_read) {
            ++x_reads;
            last_read_addr = x_addr;
        }
        if (x_write) {
            ++x_writes;
            last_addr = core.x_addr;
            last_data = core.x_wdata & 0x00ffffffu;
        }
        if (y_read) {
            ++y_reads;
            last_read_addr = y_addr;
        }
        if (y_write) {
            ++y_writes;
            last_addr = core.y_addr;
            last_data = core.y_wdata & 0x00ffffffu;
        }
        if (core.p_data_rd_en)
            ++p_reads;
        if (core.p_data_wr_en)
            ++p_writes;
        core.clk = 1;
        core.eval();
        if (fetch)
            core.p_fetch_data = p[fetch_addr & 0x1f] & 0x00ffffffu;
        if (x_read)
            core.x_rdata = x[x_addr & 0xff] & 0x00ffffffu;
        if (y_read)
            core.y_rdata = y[y_addr & 0xff] & 0x00ffffffu;
        if (x_write)
            x[x_addr & 0xff] = x_wdata & 0x00ffffffu;
        if (y_write)
            y[y_addr & 0xff] = y_wdata & 0x00ffffffu;
    }

    bool retire(uint32_t opcode) {
        for (unsigned i = 0; i < 100; ++i) {
            if (core.state_out == kStateWritebackFetch &&
                core.cur_inst_out == opcode) {
                tick();
                return true;
            }
            tick();
        }
        return false;
    }

    void debug_write(uint8_t address, uint64_t data) {
        core.dbg_reg_wr_addr = address;
        core.dbg_reg_wr_data = data;
        core.dbg_reg_wr_en = 1;
        tick();
        core.dbg_reg_wr_en = 0;
    }
};

uint32_t short_memory_move(unsigned reg, bool y_space, bool load,
                           unsigned address) {
    return ((0x40u | ((reg >> 3) << 4) | (y_space ? 8u : 0u) |
             (reg & 7u)) << 16) |
           (load ? 0x8000u : 0u) | ((address & 0x3fu) << 8);
}

struct AddressCase {
    const char *name;
    uint32_t opcode;
    bool extension;
    uint32_t extension_word;
    bool load;
    bool immediate;
    uint16_t address;
    uint16_t final_r0;
};

bool check_address_case(bool y_space, const AddressCase &test) {
    Harness h;

    h.p[0] = memory_move(0x10, false, true, 6, 4); // MOVE #$20,R0
    h.p[1] = 0x000020;
    h.p[2] = memory_move(0x18, false, true, 6, 4); // MOVE #$03,N0
    h.p[3] = 0x000003;
    h.p[4] = 0x05f420; // MOVEC #>$ffff,M0
    h.p[5] = 0x00ffff;
    h.p[6] = memory_move(0x05, false, true, 6, 4); // seed X1
    h.p[7] = kStoreData;
    h.p[8] = test.opcode;
    unsigned next = 9;
    if (test.extension)
        h.p[next++] = test.extension_word;
    h.p[next++] = register_move(0x05, 0x08); // X1 -> A0, public data
    h.p[next++] = register_move(0x10, 0x0c); // R0 -> A1, public address
    const unsigned terminal = next;
    h.p[next++] = 0x0af080;
    h.p[next] = terminal;

    if (!test.immediate) {
        auto &memory = y_space ? h.y : h.x;
        memory[test.address] = kMemoryData;
    }
    h.core.run = 1;
    const bool setup = h.retire(0x05f420);
    h.x_reads = h.x_writes = h.y_reads = h.y_writes = 0;
    h.p_reads = h.p_writes = 0;
    h.last_addr = h.last_read_addr = 0;
    h.last_data = 0;
    const bool moved = h.retire(test.opcode);
    const bool data_public = h.retire(register_move(0x05, 0x08));
    const bool address_public = h.retire(register_move(0x10, 0x0c));

    const uint32_t expected_data = test.load ?
        (test.immediate ? kImmediateData : kMemoryData) : kStoreData;
    const uint32_t actual_data = h.core.acc_a_out & 0x00ffffffu;
    const uint16_t actual_r0 = (h.core.acc_a_out >> 24) & 0x00ffffu;
    const unsigned expected_reads = test.load && !test.immediate ? 1u : 0u;
    const unsigned expected_writes = test.load ? 0u : 1u;
    const unsigned actual_reads = y_space ? h.y_reads : h.x_reads;
    const unsigned actual_writes = y_space ? h.y_writes : h.x_writes;
    const unsigned wrong_reads = y_space ? h.x_reads : h.y_reads;
    const unsigned wrong_writes = y_space ? h.x_writes : h.y_writes;
    const uint16_t actual_address = expected_reads ? h.last_read_addr :
                                    expected_writes ? h.last_addr : 0;
    const bool pass = setup && moved && data_public && address_public &&
                      actual_data == expected_data &&
                      actual_r0 == test.final_r0 &&
                      actual_reads == expected_reads &&
                      actual_writes == expected_writes &&
                      wrong_reads == 0 && wrong_writes == 0 &&
                      h.p_reads == 0 && h.p_writes == 0 &&
                      (test.immediate || actual_address == test.address) &&
                      (test.load || h.last_data == kStoreData);
    std::printf("%s MOVE form %-14s %c %s data=%06x/%06x "
                "addr=%04x/%04x R0=%04x/%04x traffic=%uR/%uW "
                "wrong=%uR/%uW retire=%u/%u/%u/%u\n",
                pass ? "PASS" : "FAIL", test.name,
                y_space ? 'Y' : 'X', test.load ? "load" : "store",
                actual_data, expected_data, actual_address,
                test.immediate ? 0 : test.address, actual_r0, test.final_r0,
                actual_reads, actual_writes, wrong_reads, wrong_writes,
                setup, moved, data_public, address_public);
    return pass;
}

bool check_case(bool y_space, unsigned source, unsigned destination,
                uint32_t seed, const char *direction) {
    Harness h;
    const uint32_t seed_op = memory_move(source, y_space, true, 6, 4);
    const uint32_t move_op = register_move(source, destination);
    const uint32_t store_op = memory_move(destination, y_space, false, 6, 0);
    h.p[0] = seed_op;
    h.p[1] = seed & 0x00ffffffu;
    h.p[2] = move_op;
    h.p[3] = store_op;
    h.p[4] = 0x0000a0;
    h.p[5] = 0x0af080; // terminal JMP >$0005
    h.p[6] = 0x000005;
    h.core.run = 1;

    const bool seeded = h.retire(seed_op);
    const bool moved = h.retire(move_op);
    const bool stored = h.retire(store_op);
    const uint32_t expected = register_word(
        destination, register_word(source, seed));
    const unsigned expected_x_writes = y_space ? 0u : 1u;
    const unsigned expected_y_writes = y_space ? 1u : 0u;
    const bool pass = seeded && moved && stored &&
                      h.last_addr == 0x00a0 && h.last_data == expected &&
                      h.x_reads == 0 && h.y_reads == 0 &&
                      h.x_writes == expected_x_writes &&
                      h.y_writes == expected_y_writes &&
                      h.p_reads == 0 && h.p_writes == 0;
    std::printf("%s MOVE selector %s-%c src=%02x dst=%02x "
                "data=%06x/%06x traffic=X%u/%u,Y%u/%u,P%u/%u "
                "retire=%u/%u/%u\n",
                pass ? "PASS" : "FAIL", direction,
                y_space ? 'Y' : 'X', source, destination,
                h.last_data, expected, h.x_reads, h.x_writes,
                h.y_reads, h.y_writes, h.p_reads, h.p_writes,
                seeded, moved, stored);
    return pass;
}

bool check_address_update(unsigned reg, unsigned mode) {
    Harness h;
    const uint32_t seed_r = memory_move(0x10 + reg, false, true, 6, 4);
    const uint32_t seed_n = memory_move(0x18 + reg, false, true, 6, 4);
    const uint32_t seed_m = 0x05f420u + reg; // MOVEC #>$ffff,Mn.
    const uint32_t update = 0x204000u | (mode << 11) | (reg << 8);
    const uint32_t publish = register_move(0x10 + reg, 0x0c); // Rn -> A1.
    const std::array<uint16_t, 4> expected{{0x001d, 0x0023, 0x001f, 0x0021}};
    const std::array<const char *, 4> names{{"post-N", "post+N", "post-1",
                                             "post+1"}};

    h.p[0] = seed_r;
    h.p[1] = 0x000020;
    h.p[2] = seed_n;
    h.p[3] = 0x000003;
    h.p[4] = seed_m;
    h.p[5] = 0x00ffff;
    h.p[6] = update;
    h.p[7] = publish;
    h.p[8] = 0x0af080;
    h.p[9] = 0x000008;
    h.core.run = 1;

    const bool setup_r = h.retire(seed_r);
    const bool setup_n = h.retire(seed_n);
    const bool setup_m = h.retire(seed_m);
    const bool updated = h.retire(update);
    const bool published = h.retire(publish);
    const uint16_t actual = (h.core.acc_a_out >> 24) & 0xffffu;
    const bool pass = setup_r && setup_n && setup_m && updated && published &&
                      actual == expected[mode] &&
                      h.x_reads == 0 && h.x_writes == 0 &&
                      h.y_reads == 0 && h.y_writes == 0 &&
                      h.p_reads == 0 && h.p_writes == 0;
    std::printf("%s MOVE address-update %-6s R%u=%04x/%04x "
                "traffic=X%u/%u,Y%u/%u,P%u/%u retire=%u/%u/%u/%u/%u\n",
                pass ? "PASS" : "FAIL", names[mode], reg, actual,
                expected[mode], h.x_reads, h.x_writes, h.y_reads,
                h.y_writes, h.p_reads, h.p_writes, setup_r, setup_n,
                setup_m, updated, published);
    return pass;
}

bool check_long_selector(unsigned selector, bool load) {
    Harness h;
    constexpr uint16_t kAddress = 0x2a;
    constexpr uint32_t kLoadX = 0x123456;
    constexpr uint32_t kLoadY = 0x654321;
    constexpr uint64_t kInitialA = 0x005a111111222222ULL;
    constexpr uint64_t kInitialB = 0x0080333333444444ULL;
    constexpr uint32_t kInitialX1 = 0x555555;
    constexpr uint32_t kInitialX0 = 0x666666;
    constexpr uint32_t kInitialY1 = 0x777777;
    constexpr uint32_t kInitialY0 = 0x111111;
    constexpr std::array<uint32_t, 8> kStoreX{{
        0x111111, 0x333333, 0x555555, 0x777777,
        0x7fffff, 0x800000, 0x7fffff, 0x800000,
    }};
    constexpr std::array<uint32_t, 8> kStoreY{{
        0x222222, 0x444444, 0x666666, 0x111111,
        0xffffff, 0x000000, 0x800000, 0x7fffff,
    }};

    const uint32_t encoded_selector = ((selector & 4u) << 17) |
                                      ((selector & 3u) << 16);
    const uint32_t opcode = 0x400000u | encoded_selector |
                            (load ? 0x008000u : 0) | (kAddress << 8);
    h.p[0] = opcode;
    unsigned terminal = 1;
    if (load && (selector == 2 || selector == 3)) {
        const unsigned high = selector == 2 ? 0x05 : 0x07;
        const unsigned low = selector == 2 ? 0x04 : 0x06;
        h.p[terminal++] = short_memory_move(high, false, false, 0x2b);
        h.p[terminal++] = short_memory_move(low, false, false, 0x2c);
    }
    h.p[terminal] = 0x0af080;
    h.p[terminal + 1] = terminal;
    h.x[kAddress] = kLoadX;
    h.y[kAddress] = kLoadY;
    h.debug_write(0x0e, kInitialA);
    h.debug_write(0x0f, kInitialB);
    h.debug_write(0x05, kInitialX1);
    h.debug_write(0x04, kInitialX0);
    h.debug_write(0x07, kInitialY1);
    h.debug_write(0x06, kInitialY0);
    h.core.run = 1;
    const bool retired = h.retire(opcode);
    bool observed = true;
    if (load && (selector == 2 || selector == 3)) {
        observed = h.retire(h.p[1]) && h.retire(h.p[2]);
    }

    bool data_ok = false;
    if (!load) {
        data_ok = h.x[kAddress] == kStoreX[selector] &&
                  h.y[kAddress] == kStoreY[selector];
    } else {
        switch (selector) {
            case 0:
                data_ok = h.core.acc_a_out == 0x005a123456654321ULL &&
                          h.core.acc_b_out == kInitialB;
                break;
            case 1:
                data_ok = h.core.acc_b_out == 0x0080123456654321ULL &&
                          h.core.acc_a_out == kInitialA;
                break;
            case 2:
                data_ok = h.x[0x2b] == kLoadX && h.x[0x2c] == kLoadY;
                break;
            case 3:
                data_ok = h.x[0x2b] == kLoadX && h.x[0x2c] == kLoadY;
                break;
            case 4:
                data_ok = h.core.acc_a_out == 0x00123456654321ULL &&
                          h.core.acc_b_out == kInitialB;
                break;
            case 5:
                data_ok = h.core.acc_b_out == 0x00123456654321ULL &&
                          h.core.acc_a_out == kInitialA;
                break;
            case 6:
                data_ok = h.core.acc_a_out == 0x00123456000000ULL &&
                          h.core.acc_b_out == 0x00654321000000ULL;
                break;
            default:
                data_ok = h.core.acc_b_out == 0x00123456000000ULL &&
                          h.core.acc_a_out == 0x00654321000000ULL;
                break;
        }
    }
    const unsigned observer_writes = load && (selector == 2 || selector == 3) ?
                                      2u : 0u;
    const bool traffic_ok = h.x_reads == (load ? 1u : 0u) &&
                            h.y_reads == (load ? 1u : 0u) &&
                            h.x_writes == (load ? observer_writes : 1u) &&
                            h.y_writes == (load ? 0u : 1u) &&
                            h.p_reads == 0 && h.p_writes == 0;
    const bool pass = retired && observed && data_ok && traffic_ok;
    std::printf("%s MOVE long selector=%u %s X=%06x Y=%06x "
                "A=%014llx B=%014llx traffic=X%u/%u,Y%u/%u,P%u/%u\n",
                pass ? "PASS" : "FAIL", selector, load ? "load " : "store",
                h.x[kAddress], h.y[kAddress],
                static_cast<unsigned long long>(h.core.acc_a_out),
                static_cast<unsigned long long>(h.core.acc_b_out),
                h.x_reads, h.x_writes, h.y_reads, h.y_writes,
                h.p_reads, h.p_writes);
    return pass;
}

bool check_long_address_form(const AddressCase &test) {
    Harness h;
    constexpr uint32_t kXWord = 0x123456;
    constexpr uint32_t kYWord = 0x654321;
    const uint32_t publish_r0 = register_move(0x10, 0x0c); // R0 -> A1.

    h.p[0] = memory_move(0x10, false, true, 6, 4); // MOVE #$20,R0.
    h.p[1] = 0x000020;
    h.p[2] = memory_move(0x18, false, true, 6, 4); // MOVE #$03,N0.
    h.p[3] = 0x000003;
    h.p[4] = 0x05f420; // MOVEC #>$ffff,M0.
    h.p[5] = 0x00ffff;
    h.p[6] = test.opcode;
    unsigned next = 7;
    if (test.extension)
        h.p[next++] = test.extension_word;
    if (test.load) {
        h.p[next++] = short_memory_move(0x05, false, false, 0x2b);
        h.p[next++] = short_memory_move(0x04, false, false, 0x2c);
    }
    h.p[next++] = publish_r0;
    const unsigned terminal = next;
    h.p[next++] = 0x0af080;
    h.p[next] = terminal;
    h.x[test.address] = kXWord;
    h.y[test.address] = kYWord;
    h.debug_write(0x05, kXWord);
    h.debug_write(0x04, kYWord);
    h.core.run = 1;

    const bool setup_r = h.retire(h.p[0]);
    const bool setup_n = h.retire(h.p[2]);
    const bool setup_m = h.retire(h.p[4]);
    const bool moved = h.retire(test.opcode);
    bool observed = true;
    if (test.load) {
        const unsigned first_observer = test.extension ? 8 : 7;
        observed = h.retire(h.p[first_observer]) &&
                   h.retire(h.p[first_observer + 1]);
    }
    const bool published = h.retire(publish_r0);
    const uint16_t actual_r0 = (h.core.acc_a_out >> 24) & 0xffffu;
    const unsigned observer_writes = test.load ? 2u : 0u;
    const bool data_ok = test.load ?
        (h.x[0x2b] == kXWord && h.x[0x2c] == kYWord) :
        (h.x[test.address] == kXWord && h.y[test.address] == kYWord);
    const bool pass = setup_r && setup_n && setup_m && moved && observed &&
                      published && data_ok && actual_r0 == test.final_r0 &&
                      h.x_reads == (test.load ? 1u : 0u) &&
                      h.y_reads == (test.load ? 1u : 0u) &&
                      h.x_writes == (test.load ? observer_writes : 1u) &&
                      h.y_writes == (test.load ? 0u : 1u) &&
                      h.p_reads == 0 && h.p_writes == 0;
    std::printf("%s MOVE long form %-10s %s addr=%04x R0=%04x/%04x "
                "data=%06x:%06x traffic=X%u/%u,Y%u/%u,P%u/%u\n",
                pass ? "PASS" : "FAIL", test.name,
                test.load ? "load " : "store", test.address, actual_r0,
                test.final_r0, h.x[test.load ? 0x2b : test.address],
                h.x[test.load ? 0x2c : test.address], h.x_reads,
                h.x_writes, h.y_reads, h.y_writes, h.p_reads, h.p_writes);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    unsigned failures = 0;
    unsigned cases = 0;

    for (const bool y_space : {false, true}) {
        for (unsigned code = 0x04; code <= 0x1f; ++code) {
            const unsigned neutral_destination = code == 0x04 ? 0x05 : 0x04;
            const uint32_t source_seed = 0x120000u | (code * 0x0101u);
            failures += !check_case(y_space, code, neutral_destination,
                                    source_seed, "source");
            ++cases;

            const unsigned neutral_source = code == 0x04 ? 0x05 : 0x04;
            const uint32_t destination_seed = 0x005500u | code;
            failures += !check_case(y_space, neutral_source, code,
                                    destination_seed, "destination");
            ++cases;
        }
    }

    for (const bool y_space : {false, true}) {
        const unsigned reg = 0x05;
        const std::array<AddressCase, 19> forms{{
            {"short", short_memory_move(reg, y_space, false, 0x2a),
             false, 0, false, false, 0x2a, 0x20},
            {"short", short_memory_move(reg, y_space, true, 0x2a),
             false, 0, true, false, 0x2a, 0x20},
            {"post-N", memory_move(reg, y_space, false, 0, 0),
             false, 0, false, false, 0x20, 0x1d},
            {"post-N", memory_move(reg, y_space, true, 0, 0),
             false, 0, true, false, 0x20, 0x1d},
            {"post+N", memory_move(reg, y_space, false, 1, 0),
             false, 0, false, false, 0x20, 0x23},
            {"post+N", memory_move(reg, y_space, true, 1, 0),
             false, 0, true, false, 0x20, 0x23},
            {"post-1", memory_move(reg, y_space, false, 2, 0),
             false, 0, false, false, 0x20, 0x1f},
            {"post-1", memory_move(reg, y_space, true, 2, 0),
             false, 0, true, false, 0x20, 0x1f},
            {"post+1", memory_move(reg, y_space, false, 3, 0),
             false, 0, false, false, 0x20, 0x21},
            {"post+1", memory_move(reg, y_space, true, 3, 0),
             false, 0, true, false, 0x20, 0x21},
            {"no-update", memory_move(reg, y_space, false, 4, 0),
             false, 0, false, false, 0x20, 0x20},
            {"no-update", memory_move(reg, y_space, true, 4, 0),
             false, 0, true, false, 0x20, 0x20},
            {"indexed+N", memory_move(reg, y_space, false, 5, 0),
             false, 0, false, false, 0x23, 0x20},
            {"indexed+N", memory_move(reg, y_space, true, 5, 0),
             false, 0, true, false, 0x23, 0x20},
            {"pre-1", memory_move(reg, y_space, false, 7, 0),
             false, 0, false, false, 0x1f, 0x1f},
            {"pre-1", memory_move(reg, y_space, true, 7, 0),
             false, 0, true, false, 0x1f, 0x1f},
            {"absolute", memory_move(reg, y_space, false, 6, 0),
             true, 0x00002a, false, false, 0x2a, 0x20},
            {"absolute", memory_move(reg, y_space, true, 6, 0),
             true, 0x00002a, true, false, 0x2a, 0x20},
            {"immediate", memory_move(reg, y_space, true, 6, 4),
             true, kImmediateData, true, true, 0, 0x20},
        }};
        for (const auto &test : forms) {
            failures += !check_address_case(y_space, test);
            ++cases;
        }
    }

    // DSP56001 UMA pp. A-125--A-126 define the tenth parallel-MOVE category:
    // an AGU-only update with all four legal modes across R0--R7.  This must
    // not create any X/Y/P memory transfer while committing the selected Rn.
    for (unsigned reg = 0; reg < 8; ++reg) {
        for (unsigned mode = 0; mode < 4; ++mode) {
            failures += !check_address_update(reg, mode);
            ++cases;
        }
    }

    // DSP56001 UMA pp. A-143--A-146 assign all eight L: register-pair
    // selectors in both directions.  Grade the pair mapping, accumulator
    // extension/preservation rules, and exact dual-bus traffic.
    for (unsigned selector = 0; selector < 8; ++selector) {
        failures += !check_long_selector(selector, false);
        failures += !check_long_selector(selector, true);
        cases += 2;
    }

    // The L: encoding reuses the ordinary AGU modes but drives X and Y
    // together.  Exercise every legal form in both directions so the shared
    // address and Rn-update contract is not inferred from single-bus MOVE.
    constexpr uint32_t kLongBase = 0x424000; // L selector X, EA form.
    const std::array<AddressCase, 18> long_forms{{
        {"short", 0x420000 | (0x2a << 8), false, 0, false, false, 0x2a, 0x20},
        {"short", 0x428000 | (0x2a << 8), false, 0, true,  false, 0x2a, 0x20},
        {"post-N", kLongBase | (0 << 11), false, 0, false, false, 0x20, 0x1d},
        {"post-N", kLongBase | 0x8000 | (0 << 11), false, 0, true, false, 0x20, 0x1d},
        {"post+N", kLongBase | (1 << 11), false, 0, false, false, 0x20, 0x23},
        {"post+N", kLongBase | 0x8000 | (1 << 11), false, 0, true, false, 0x20, 0x23},
        {"post-1", kLongBase | (2 << 11), false, 0, false, false, 0x20, 0x1f},
        {"post-1", kLongBase | 0x8000 | (2 << 11), false, 0, true, false, 0x20, 0x1f},
        {"post+1", kLongBase | (3 << 11), false, 0, false, false, 0x20, 0x21},
        {"post+1", kLongBase | 0x8000 | (3 << 11), false, 0, true, false, 0x20, 0x21},
        {"direct", kLongBase | (4 << 11), false, 0, false, false, 0x20, 0x20},
        {"direct", kLongBase | 0x8000 | (4 << 11), false, 0, true, false, 0x20, 0x20},
        {"indexed", kLongBase | (5 << 11), false, 0, false, false, 0x23, 0x20},
        {"indexed", kLongBase | 0x8000 | (5 << 11), false, 0, true, false, 0x23, 0x20},
        {"pre-1", kLongBase | (7 << 11), false, 0, false, false, 0x1f, 0x1f},
        {"pre-1", kLongBase | 0x8000 | (7 << 11), false, 0, true, false, 0x1f, 0x1f},
        {"absolute", kLongBase | (6 << 11), true, 0x2a, false, false, 0x2a, 0x20},
        {"absolute", kLongBase | 0x8000 | (6 << 11), true, 0x2a, true, false, 0x2a, 0x20},
    }};
    for (const auto &test : long_forms) {
        failures += !check_long_address_form(test);
        ++cases;
    }

    std::printf("MOVE SELECTOR SUMMARY cases=%u failures=%u "
                "assigned-registers=28 spaces=2 directions=2 "
                "address-forms=38 address-updates=32 long-selectors=16 "
                "long-forms=18\n",
                cases, failures);
    return failures ? 1 : 0;
}
