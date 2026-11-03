// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

#include "Vdsp56001_dma.h"
#include "verilated.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <unordered_map>

namespace {

[[noreturn]] void fail(const char *message) {
    std::fprintf(stderr, "FAIL dsp56001_dma: %s\n", message);
    std::exit(1);
}

struct Sim {
    Vdsp56001_dma top;
    std::array<uint32_t, 32768> sram{};
    std::unordered_map<uint32_t, uint64_t> memory;
    unsigned cycle = 0;
    unsigned sram_reads = 0;
    unsigned sram_writes = 0;
    unsigned mem_reads = 0;
    unsigned mem_writes = 0;

    bool sram_pending = false;
    unsigned sram_delay = 0;
    uint16_t sram_pending_addr = 0;
    uint32_t sram_pending_wdata = 0;
    bool sram_pending_read = false;
    bool sram_pending_write = false;
    bool mem_pending = false;
    unsigned mem_delay = 0;
    uint32_t mem_pending_addr = 0;
    uint64_t mem_pending_wdata = 0;
    uint8_t mem_pending_wmask = 0;
    bool mem_pending_write = false;

    void defaults() {
        top.clk = 0;
        top.rst_n = 1;
        top.dma_go = 0;
        top.dma_dir = 0;
        top.dma_mode = 0;
        top.dma_count = 0;
        top.dma_gio_addr = 0;
        top.dma_pbus_addr = 0;
        top.dma_sign_mode = 0;
        top.sram_rdata = 0;
        top.sram_ready = 0;
        top.mem_rdata = 0;
        top.mem_ready = 0;
    }

    void reset() {
        defaults();
        top.rst_n = 0;
        tick();
        tick();
        top.rst_n = 1;
        tick();
    }

    void service_requests() {
        top.sram_ready = 0;
        top.mem_ready = 0;

        if (!top.rst_n) {
            sram_pending = false;
            mem_pending = false;
            return;
        }

        if (!sram_pending && (top.sram_rd_en || top.sram_wr_en)) {
            sram_pending = true;
            sram_delay = 1 + (cycle % 4);
            sram_pending_addr = top.sram_addr;
            sram_pending_wdata = top.sram_wdata;
            sram_pending_read = top.sram_rd_en;
            sram_pending_write = top.sram_wr_en;
        }
        if (sram_pending) {
            if (top.sram_addr != sram_pending_addr ||
                uint32_t(top.sram_wdata) != sram_pending_wdata ||
                bool(top.sram_rd_en) != sram_pending_read ||
                bool(top.sram_wr_en) != sram_pending_write)
                fail("SRAM request changed while completion was stalled");
            if (sram_delay != 0) {
                --sram_delay;
            } else {
                top.sram_rdata = sram[top.sram_addr] & 0x00ffffffu;
                top.sram_ready = 1;
            }
        }

        if (!mem_pending && top.mem_valid) {
            mem_pending = true;
            mem_delay = 2 + (cycle % 5);
            mem_pending_addr = top.mem_addr;
            mem_pending_wdata = top.mem_wdata;
            mem_pending_wmask = top.mem_wmask;
            mem_pending_write = top.mem_write;
        }
        if (mem_pending) {
            if (!top.mem_valid || top.mem_addr != mem_pending_addr ||
                uint64_t(top.mem_wdata) != mem_pending_wdata ||
                uint8_t(top.mem_wmask) != mem_pending_wmask ||
                bool(top.mem_write) != mem_pending_write)
                fail("memory request changed while completion was stalled");
            if (mem_delay != 0) {
                --mem_delay;
            } else {
                top.mem_rdata = memory[top.mem_addr];
                top.mem_ready = 1;
            }
        }
    }

    void accept_requests() {
        if (top.sram_ready && (top.sram_rd_en || top.sram_wr_en)) {
            if (top.sram_rd_en) {
                ++sram_reads;
            } else {
                sram[top.sram_addr] = top.sram_wdata & 0x00ffffffu;
                ++sram_writes;
            }
            sram_pending = false;
        }

        if (top.mem_ready && top.mem_valid) {
            if (top.mem_write) {
                uint64_t &word = memory[top.mem_addr];
                for (unsigned byte = 0; byte < 8; ++byte) {
                    const uint8_t bit = uint8_t(0x80u >> byte);
                    if (top.mem_wmask & bit) {
                        const unsigned shift = 56 - 8 * byte;
                        word &= ~(uint64_t(0xff) << shift);
                        word |= ((uint64_t(top.mem_wdata) >> shift) & 0xffu)
                                << shift;
                    }
                }
                ++mem_writes;
            } else {
                ++mem_reads;
            }
            mem_pending = false;
        }
    }

    void tick() {
        top.clk = 0;
        top.eval();
        service_requests();
        top.eval();
        accept_requests();
        top.clk = 1;
        top.eval();
        top.clk = 0;
        top.eval();
        ++cycle;
    }

    void start(bool dir, uint16_t count, uint32_t gio, uint16_t pbus,
               uint8_t sign_mode = 4, bool folded = false) {
        top.dma_dir = dir;
        top.dma_count = count;
        top.dma_gio_addr = gio;
        top.dma_pbus_addr = pbus;
        top.dma_sign_mode = sign_mode;
        top.dma_mode = folded;
        top.dma_go = 1;
        tick();
        if (count != 0 && !top.dma_active) {
            std::fprintf(stderr,
                "start failed: go=%u input-count=%u current=%u dir=%u\n",
                top.dma_go, top.dma_count, top.dma_count_current,
                top.dma_dir);
            fail(folded ? "nonzero folded transfer did not leave idle" :
                          "nonzero normal transfer did not leave idle");
        }
    }

    void run_to_done(unsigned limit = 100000) {
        unsigned done_count = top.dma_done ? 1 : 0;
        if (!done_count) {
            for (unsigned i = 0; i < limit; ++i) {
                tick();
                if (top.dma_done) {
                    ++done_count;
                    break;
                }
            }
        }
        if (done_count != 1) {
            std::fprintf(stderr,
                "DMA timeout cycle=%u active=%u remaining=%u "
                "sram-r/w=%u/%u mem-r/w=%u/%u\n",
                cycle, top.dma_active, top.dma_count_current,
                sram_reads, sram_writes, mem_reads, mem_writes);
            fail("transfer did not emit one completion pulse");
        }
        for (unsigned i = 0; i < 8; ++i) {
            tick();
            if (top.dma_done)
                fail("held GO emitted a duplicate completion pulse");
        }
        top.dma_go = 0;
        tick();
        if (top.dma_active)
            fail("engine remained active after GO cleared");
    }
};

uint32_t mem_word(const Sim &sim, uint32_t addr) {
    auto it = sim.memory.find(addr & ~uint32_t(7));
    const uint64_t raw = it == sim.memory.end() ? 0 : it->second;
    return (addr & 4) ? uint32_t(raw) : uint32_t(raw >> 32);
}

void set_mem_word(Sim &sim, uint32_t addr, uint32_t value) {
    uint64_t &raw = sim.memory[addr & ~uint32_t(7)];
    if (addr & 4)
        raw = (raw & 0xffffffff00000000ULL) | value;
    else
        raw = (raw & 0x00000000ffffffffULL) | (uint64_t(value) << 32);
}

void test_stock_48_word_round_trip() {
    Sim sim;
    sim.reset();

    std::array<uint32_t, 48> pattern{};
    for (unsigned i = 0; i < 24; ++i) {
        pattern[i] = 1u << i;
        pattern[24 + i] = (~(1u << i)) & 0x00ffffffu;
        sim.sram[0x4000 + i] = pattern[i];
        sim.sram[0x4000 + 24 + i] = pattern[24 + i];
    }

    sim.start(true, 48, 0x08400000, 0xc000, 4);
    sim.run_to_done();
    if (sim.sram_reads != 48 || sim.mem_writes != 48 ||
        sim.sram_writes != 0 || sim.mem_reads != 0)
        fail("stock DSP-to-GIO transaction counts changed");
    for (unsigned i = 0; i < pattern.size(); ++i) {
        const uint32_t expected = (pattern[i] & 0x00800000u) ?
            (pattern[i] | 0xff000000u) : pattern[i];
        if (mem_word(sim, 0x08400000 + 4 * i) != expected)
            fail("stock DSP-to-GIO payload/sign extension changed");
    }

    const unsigned pre_mem_reads = sim.mem_reads;
    const unsigned pre_sram_writes = sim.sram_writes;
    sim.start(false, 48, 0x08400000, 0xc030, 4);
    sim.run_to_done();
    if (sim.mem_reads - pre_mem_reads != 48 ||
        sim.sram_writes - pre_sram_writes != 48)
        fail("stock GIO-to-DSP transaction counts changed");
    for (unsigned i = 0; i < pattern.size(); ++i) {
        if ((sim.sram[0x4030 + i] & 0x00ffffffu) != pattern[i])
            fail("stock 48-word DMA round trip corrupted data");
    }
}

void test_lanes_and_sign_modes() {
    struct Case { uint32_t source; uint8_t mode; uint32_t expected; };
    const Case cases[] = {
        {0x00800080, 7, 0xffffff80},
        {0x00808000, 6, 0xffff8000},
        {0x00800000, 4, 0xff800000},
        {0x00fabcde, 0, 0x00fabcde},
    };

    for (unsigned i = 0; i < 4; ++i) {
        Sim sim;
        sim.reset();
        sim.sram[0x4000] = cases[i].source;
        const uint32_t addr = 0x08401000 + ((i & 1) ? 4 : 0);
        sim.start(true, 1, addr, 0xc000, cases[i].mode);
        sim.run_to_done();
        if (mem_word(sim, addr) != cases[i].expected)
            fail("normal-format lane/sign conversion changed");
    }
}

void test_folded_pair_packing_and_odd_tail() {
    // Folded transfers use one host 32-bit word for each pair of DSP
    // 24-bit words, while the DMA word count remains measured in SRAM words.
    // Audio samples are left-justified: the first DSP word occupies the
    // big-endian high halfword and the second occupies the low halfword.
    // An odd final DSP word must update only the high half of the last MIPS
    // word, preserving the adjacent low half.
    Sim to_host;
    to_host.reset();
    const std::array<uint32_t, 5> dsp_words = {
        0x123456, 0x89abcd, 0x7fff80, 0x80007f, 0x55aa33
    };
    for (unsigned i = 0; i < dsp_words.size(); ++i)
        to_host.sram[0x4200 + i] = dsp_words[i];
    set_mem_word(to_host, 0x08430000, 0xdeadbeef);
    set_mem_word(to_host, 0x08430004, 0xcafef00d);
    set_mem_word(to_host, 0x08430008, 0x13572468);
    to_host.start(true, dsp_words.size(), 0x08430000, 0xc200, 7, true);
    to_host.run_to_done();
    if (to_host.top.dma_folded_unsupported)
        fail("supported folded DSP-to-GIO transfer reported unsupported");
    if (to_host.sram_reads != 5 || to_host.mem_writes != 3 ||
        to_host.sram_writes != 0 || to_host.mem_reads != 0)
        fail("folded DSP-to-GIO transaction counts were not 5 SRAM / 3 GIO");
    if (mem_word(to_host, 0x08430000) != 0x123489abu ||
        mem_word(to_host, 0x08430004) != 0x7fff8000u ||
        mem_word(to_host, 0x08430008) != 0x55aa2468u)
        fail("folded DSP-to-GIO pair packing or odd-tail preservation failed");

    Sim from_host;
    from_host.reset();
    set_mem_word(from_host, 0x08431004, 0x7fff8000u);
    set_mem_word(from_host, 0x08431008, 0x1234fedcu);
    set_mem_word(from_host, 0x0843100c, 0x55aaaa55u);
    from_host.start(false, 5, 0x08431004, 0xc240, 0, true);
    from_host.run_to_done();
    if (from_host.top.dma_folded_unsupported)
        fail("supported folded GIO-to-DSP transfer reported unsupported");
    if (from_host.mem_reads != 3 || from_host.sram_writes != 5 ||
        from_host.mem_writes != 0 || from_host.sram_reads != 0)
        fail("folded GIO-to-DSP transaction counts were not 3 GIO / 5 SRAM");
    const std::array<uint32_t, 5> expected = {
        0x7fff00, 0x800000, 0x123400, 0xfedc00, 0x55aa00
    };
    for (unsigned i = 0; i < expected.size(); ++i) {
        if (from_host.sram[0x4240 + i] != expected[i])
            fail("folded GIO-to-DSP pair unpacking or sample order failed");
    }
}

void test_zero_count() {
    Sim sim;
    sim.reset();
    sim.start(true, 0, 0x08402000, 0xc000);
    sim.run_to_done();
    if (sim.sram_reads || sim.sram_writes || sim.mem_reads || sim.mem_writes)
        fail("zero-count DMA touched a downstream port");

    sim.start(false, 0, 0x08402000, 0xc000, 7, true);
    sim.run_to_done();
    if (sim.top.dma_folded_unsupported || sim.sram_reads || sim.sram_writes ||
        sim.mem_reads || sim.mem_writes)
        fail("zero-count folded DMA did not complete without port traffic");
}

void test_count_sweep_and_latched_busy_controls() {
    for (unsigned count = 1; count <= 17; ++count) {
        Sim sim;
        sim.reset();
        const uint16_t pbus = uint16_t(0xc080 + count * 0x20);
        const uint32_t gio = 0x08410000 + count * 0x100 +
                             ((count & 1) ? 4 : 0);
        for (unsigned i = 0; i < count; ++i)
            sim.sram[(pbus + i) & 0x7fff] =
                (0x00100000u + count * 0x100u + i) & 0x00ffffffu;

        sim.start(true, uint16_t(count), gio, pbus, 0);

        // The programmed descriptor is sampled when GO leaves idle.  Later
        // host-register writes while the transfer is active may not redirect
        // or reinterpret the already-issued operation.
        sim.top.dma_dir = 0;
        sim.top.dma_mode = 1;
        sim.top.dma_count = 1;
        sim.top.dma_gio_addr = 0x0bad0004;
        sim.top.dma_pbus_addr = 0xdead;
        sim.top.dma_sign_mode = 7;
        sim.run_to_done();

        if (sim.sram_reads != count || sim.mem_writes != count ||
            sim.sram_writes != 0 || sim.mem_reads != 0)
            fail("count sweep changed exact transaction counts");
        for (unsigned i = 0; i < count; ++i) {
            const uint32_t expected =
                (0x00100000u + count * 0x100u + i) & 0x00ffffffu;
            if (mem_word(sim, gio + 4 * i) != expected)
                fail("busy control mutation changed latched DMA payload");
        }
    }
}

uint32_t large_pattern(unsigned ordinal) {
    return (0x005a0000u ^ (ordinal * 0x00012345u) ^
            (ordinal << 17)) & 0x00ffffffu;
}

void test_large_count_lane_wrap_matrix() {
    // Cross the small 1..17 sweep, the stock 48-word boundary, powers of two,
    // both halves of every 64-bit memory beat, and the physical 32K SRAM
    // address wrap.  DMAWDCNT is a DSP-word count in both normal and folded
    // modes; folded mode therefore issues ceil(count/2) memory transactions.
    constexpr std::array<unsigned, 11> counts = {
        18, 31, 32, 33, 47, 48, 49, 255, 256, 257, 1023
    };
    unsigned cases = 0;

    for (bool folded : {false, true}) {
        for (bool to_host : {false, true}) {
            for (unsigned lane : {0u, 4u}) {
                for (unsigned count : counts) {
                    Sim sim;
                    sim.reset();
                    const uint16_t pbus = uint16_t(0xffe0u + (count & 7u));
                    const uint32_t gio = 0x08480000u +
                        (folded ? 0x00100000u : 0u) +
                        (to_host ? 0x00080000u : 0u) + lane;
                    const unsigned mem_transactions = folded ?
                        (count + 1) / 2 : count;

                    if (to_host) {
                        for (unsigned i = 0; i < count; ++i)
                            sim.sram[(pbus + i) & 0x7fffu] = large_pattern(i);
                        for (unsigned i = 0; i < mem_transactions; ++i)
                            set_mem_word(sim, gio + 4 * i,
                                         0xa5000000u ^ (i * 0x01010101u));
                    } else {
                        for (unsigned i = 0; i < mem_transactions; ++i) {
                            const uint32_t value = folded ?
                                (((large_pattern(2 * i) >> 8) << 16) |
                                 (large_pattern(2 * i + 1) >> 8)) :
                                (0xc3000000u | large_pattern(i));
                            set_mem_word(sim, gio + 4 * i, value);
                        }
                        sim.sram[(pbus - 1) & 0x7fffu] = 0x00112233u;
                        sim.sram[(pbus + count) & 0x7fffu] = 0x00445566u;
                    }

                    sim.start(to_host, uint16_t(count), gio, pbus, 0, folded);
                    sim.run_to_done();
                    if (sim.top.dma_count_current != 0)
                        fail("large-count matrix did not retire DMAWDCNT");

                    if (to_host) {
                        if (sim.sram_reads != count ||
                            sim.mem_writes != mem_transactions ||
                            sim.sram_writes != 0 || sim.mem_reads != 0)
                            fail("large-count DSP-to-GIO transaction counts changed");
                        for (unsigned i = 0; i < mem_transactions; ++i) {
                            uint32_t expected;
                            if (folded) {
                                expected = (large_pattern(2 * i) >> 8) << 16;
                                if (2 * i + 1 < count)
                                    expected |= large_pattern(2 * i + 1) >> 8;
                                else
                                    expected |= (0xa5000000u ^
                                        (i * 0x01010101u)) & 0xffffu;
                            } else {
                                expected = large_pattern(i);
                            }
                            if (mem_word(sim, gio + 4 * i) != expected)
                                fail("large-count DSP-to-GIO payload/order changed");
                        }
                    } else {
                        if (sim.mem_reads != mem_transactions ||
                            sim.sram_writes != count ||
                            sim.mem_writes != 0 || sim.sram_reads != 0)
                            fail("large-count GIO-to-DSP transaction counts changed");
                        for (unsigned i = 0; i < count; ++i) {
                            const uint32_t expected = folded ?
                                ((large_pattern(i) >> 8) << 8) :
                                large_pattern(i);
                            if (sim.sram[(pbus + i) & 0x7fffu] != expected)
                                fail("large-count GIO-to-DSP payload/order changed");
                        }
                        if (sim.sram[(pbus - 1) & 0x7fffu] != 0x00112233u ||
                            sim.sram[(pbus + count) & 0x7fffu] != 0x00445566u)
                            fail("large-count GIO-to-DSP crossed a PBUS sentinel");
                    }
                    ++cases;
                }
            }
        }
    }
    if (cases != 88)
        fail("large-count matrix case census changed");
}

uint32_t max_count_sram_pattern(unsigned address) {
    return (0x003c0000u ^ (address * 0x00013579u) ^
            (address << 11)) & 0x00ffffffu;
}

uint16_t max_count_half_pattern(unsigned ordinal) {
    return uint16_t(0x8001u ^ (ordinal * 0x01f5u) ^ (ordinal >> 3));
}

void test_max_count_and_full_sram_wrap() {
    constexpr uint16_t count = 0xffff;
    constexpr unsigned folded_mem_count = (unsigned(count) + 1) / 2;

    // Normal DSP-to-GIO: a maximum descriptor traverses the complete 32K
    // physical SRAM twice minus one word.  Every accepted word and its host
    // lane must remain exact across both address wraps.
    {
        Sim sim;
        sim.reset();
        constexpr uint16_t pbus = 0xfffb;
        constexpr uint32_t gio = 0x08600004;
        for (unsigned addr = 0; addr < sim.sram.size(); ++addr)
            sim.sram[addr] = max_count_sram_pattern(addr);
        sim.start(true, count, gio, pbus, 0, false);
        sim.run_to_done(2000000);
        if (sim.sram_reads != count || sim.mem_writes != count ||
            sim.sram_writes != 0 || sim.mem_reads != 0)
            fail("maximum-count normal DSP-to-GIO transaction census changed");
        for (unsigned i = 0; i < count; ++i) {
            const uint32_t expected =
                max_count_sram_pattern((unsigned(pbus) + i) & 0x7fffu);
            if (mem_word(sim, gio + 4 * i) != expected)
                fail("maximum-count normal DSP-to-GIO payload/order changed");
        }
    }

    // Normal GIO-to-DSP: retain the last accepted value for every physical
    // SRAM word after the same two-wrap-minus-one traversal.
    {
        Sim sim;
        sim.reset();
        constexpr uint16_t pbus = 0xfff9;
        constexpr uint32_t gio = 0x08700000;
        std::array<uint32_t, 32768> expected{};
        for (unsigned i = 0; i < count; ++i) {
            const uint32_t value = max_count_sram_pattern(i);
            set_mem_word(sim, gio + 4 * i, 0xc0000000u | value);
            expected[(unsigned(pbus) + i) & 0x7fffu] = value;
        }
        sim.start(false, count, gio, pbus, 0, false);
        sim.run_to_done(2000000);
        if (sim.mem_reads != count || sim.sram_writes != count ||
            sim.mem_writes != 0 || sim.sram_reads != 0)
            fail("maximum-count normal GIO-to-DSP transaction census changed");
        for (unsigned addr = 0; addr < sim.sram.size(); ++addr) {
            if (sim.sram[addr] != expected[addr])
                fail("maximum-count normal GIO-to-DSP wrap payload changed");
        }
    }

    // Folded DSP-to-GIO: 65,535 SRAM words must become exactly 32,768 host
    // writes.  The final odd word updates only the high half of its host word.
    {
        Sim sim;
        sim.reset();
        constexpr uint16_t pbus = 0xfffd;
        constexpr uint32_t gio = 0x08800004;
        for (unsigned addr = 0; addr < sim.sram.size(); ++addr)
            sim.sram[addr] = uint32_t(max_count_half_pattern(addr)) << 8;
        for (unsigned i = 0; i < folded_mem_count; ++i)
            set_mem_word(sim, gio + 4 * i, 0x5a5a0000u | (i & 0xffffu));
        sim.start(true, count, gio, pbus, 0, true);
        sim.run_to_done(2000000);
        if (sim.sram_reads != count || sim.mem_writes != folded_mem_count ||
            sim.sram_writes != 0 || sim.mem_reads != 0)
            fail("maximum-count folded DSP-to-GIO transaction census changed");
        for (unsigned i = 0; i < folded_mem_count; ++i) {
            const unsigned first = 2 * i;
            uint32_t expected = uint32_t(max_count_half_pattern(
                (unsigned(pbus) + first) & 0x7fffu)) << 16;
            if (first + 1 < count) {
                expected |= max_count_half_pattern(
                    (unsigned(pbus) + first + 1) & 0x7fffu);
            } else {
                expected |= i & 0xffffu;
            }
            if (mem_word(sim, gio + 4 * i) != expected)
                fail("maximum-count folded DSP-to-GIO payload/order changed");
        }
    }

    // Folded GIO-to-DSP: prove exact unpacking, physical SRAM wrap, and the
    // absence of a spurious second write after the odd terminal sample.
    {
        Sim sim;
        sim.reset();
        constexpr uint16_t pbus = 0xfff7;
        constexpr uint32_t gio = 0x08900000;
        std::array<uint32_t, 32768> expected{};
        for (unsigned i = 0; i < folded_mem_count; ++i) {
            const uint16_t high = max_count_half_pattern(2 * i);
            const uint16_t low = max_count_half_pattern(2 * i + 1);
            set_mem_word(sim, gio + 4 * i,
                         (uint32_t(high) << 16) | uint32_t(low));
            const unsigned first = 2 * i;
            expected[(unsigned(pbus) + first) & 0x7fffu] =
                uint32_t(high) << 8;
            if (first + 1 < count) {
                expected[(unsigned(pbus) + first + 1) & 0x7fffu] =
                    uint32_t(low) << 8;
            }
        }
        sim.start(false, count, gio, pbus, 0, true);
        sim.run_to_done(2000000);
        if (sim.mem_reads != folded_mem_count || sim.sram_writes != count ||
            sim.mem_writes != 0 || sim.sram_reads != 0)
            fail("maximum-count folded GIO-to-DSP transaction census changed");
        for (unsigned addr = 0; addr < sim.sram.size(); ++addr) {
            if (sim.sram[addr] != expected[addr])
                fail("maximum-count folded GIO-to-DSP wrap payload changed");
        }
    }
}

void test_reset_while_busy_and_restart() {
    Sim sim;
    sim.reset();
    sim.sram[0x4100] = 0x0055aa33;
    sim.start(true, 8, 0x08420000, 0xc100, 0);
    if (!sim.top.dma_active)
        fail("reset-while-busy setup did not become active");

    sim.top.rst_n = 0;
    sim.tick();
    if (sim.top.dma_active || sim.top.dma_done ||
        sim.top.dma_count_current != 0 || sim.top.sram_rd_en ||
        sim.top.sram_wr_en || sim.top.mem_valid)
        fail("reset did not cancel the active DMA boundary cleanly");

    sim.top.dma_go = 0;
    sim.top.rst_n = 1;
    sim.tick();
    sim.sram[0x4120] = 0x00123456;
    const unsigned pre_sram_reads = sim.sram_reads;
    const unsigned pre_mem_writes = sim.mem_writes;
    sim.start(true, 1, 0x08421004, 0xc120, 0);
    sim.run_to_done();
    if (sim.sram_reads - pre_sram_reads != 1 ||
        sim.mem_writes - pre_mem_writes != 1 ||
        mem_word(sim, 0x08421004) != 0x00123456)
        fail("DMA did not restart exactly after reset cancellation");
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    test_stock_48_word_round_trip();
    test_lanes_and_sign_modes();
    test_folded_pair_packing_and_odd_tail();
    test_zero_count();
    test_count_sweep_and_latched_busy_controls();
    test_large_count_lane_wrap_matrix();
    test_max_count_and_full_sram_wrap();
    test_reset_while_busy_and_restart();
    std::puts("PASS DSP normal DMA: stock 48-word round trip, stable stalls, "
              "lanes, sign modes, 1..17 plus 88-case large-count/lane/wrap "
              "matrix, maximum 65535-word normal/folded transfers, latched "
              "busy controls, reset restart, zero count, and folded pair "
              "packing");
    return 0;
}
