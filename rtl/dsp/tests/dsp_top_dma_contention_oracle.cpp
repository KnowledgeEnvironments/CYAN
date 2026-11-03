// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

// Production dsp56001_top host/DMA/shared-SRAM contention oracle.

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <unordered_map>

#include "Vdsp56001_top.h"
#include "verilated.h"

namespace {

[[noreturn]] void fail(const char *message) {
    std::fprintf(stderr, "FAIL DSP top DMA contention: %s\n", message);
    std::exit(1);
}

uint64_t merge_bytes(uint64_t old_value, uint64_t new_value, uint8_t mask) {
    for (unsigned byte = 0; byte < 8; ++byte) {
        if (mask & (0x80u >> byte)) {
            const unsigned shift = 56 - 8 * byte;
            old_value &= ~(uint64_t(0xff) << shift);
            old_value |= ((new_value >> shift) & 0xffu) << shift;
        }
    }
    return old_value;
}

uint32_t sample24(unsigned ordinal) {
    return (0x005a0000u ^ (ordinal * 0x00013579u) ^
            (ordinal << 13)) & 0x00ffffffu;
}

uint16_t sample16(unsigned ordinal) {
    return uint16_t(0x8101u ^ (ordinal * 0x1271u) ^ (ordinal >> 2));
}

struct Case {
    bool folded;
    bool to_host;
    unsigned count;
    unsigned lane;
};

constexpr std::array<Case, 8> kCases{{
    {false, false, 48, 0}, {false, false, 48, 4},
    {false, true,  48, 0}, {false, true,  48, 4},
    {true,  false, 47, 0}, {true,  false, 48, 4},
    {true,  true,  47, 4}, {true,  true,  48, 0},
}};

class Sim {
public:
    Vdsp56001_top top;
    std::unordered_map<uint32_t, uint64_t> memory;
    uint64_t cycles = 0;
    unsigned mem_reads = 0, mem_writes = 0;
    unsigned cpu_reads = 0, cpu_writes = 0;
    unsigned cpu_completions_while_dma = 0;

    bool mem_pending = false;
    unsigned mem_delay = 0;
    uint32_t mem_addr = 0;
    uint64_t mem_wdata = 0;
    uint8_t mem_wmask = 0;
    bool mem_write = false;

    bool cpu_pending = false;
    bool cpu_write = false;
    uint16_t cpu_addr = 0;
    uint32_t cpu_wdata = 0;
    uint32_t cpu_rdata = 0;

    Sim() {
        top.clk = 0;
        top.rst_n = 0;
        top.cpu_reg_addr = 0;
        top.cpu_reg_wdata = 0;
        top.cpu_reg_rd_en = 0;
        top.cpu_reg_wr_en = 0;
        top.cpu_sram_addr = 0;
        top.cpu_sram_wdata = 0;
        top.cpu_sram_wr_en = 0;
        top.cpu_sram_rd_en = 0;
        top.ssi_tx_ready = 0;
        top.ssi_rx_valid = 0;
        top.ssi_rx_data = 0;
        top.rtc_rdata = 0;
        top.dma_mem_rdata = 0;
        top.dma_mem_ready = 0;
        top.dbg_mem_wr_en = 0;
        top.dbg_mem_space = 0;
        top.dbg_mem_addr = 0;
        top.dbg_mem_wdata = 0;
        top.dbg_reg_wr_en = 0;
        top.dbg_reg_wr_addr = 0;
        top.dbg_reg_wr_data = 0;
        for (unsigned i = 0; i < 4; ++i)
            tick();
        top.rst_n = 1;
        tick();
    }

    void tick() {
        top.clk = 0;
        top.dma_mem_ready = 0;
        top.eval();

        if (!mem_pending && top.dma_mem_valid) {
            mem_pending = true;
            mem_delay = 1 + unsigned(cycles % 5);
            mem_addr = top.dma_mem_addr;
            mem_wdata = top.dma_mem_wdata;
            mem_wmask = top.dma_mem_wmask;
            mem_write = top.dma_mem_write;
        }
        if (mem_pending) {
            if (!top.dma_mem_valid || top.dma_mem_addr != mem_addr ||
                uint64_t(top.dma_mem_wdata) != mem_wdata ||
                uint8_t(top.dma_mem_wmask) != mem_wmask ||
                bool(top.dma_mem_write) != mem_write)
                fail("DMA memory request changed while stalled");
            if (mem_delay != 0) {
                --mem_delay;
            } else {
                top.dma_mem_rdata = memory[mem_addr];
                top.dma_mem_ready = 1;
            }
        }
        top.eval();

        const bool cpu_complete = cpu_pending && top.cpu_sram_ready;
        if (cpu_complete)
            cpu_rdata = top.cpu_sram_rdata & 0x00ffffffu;
        const bool mem_complete = mem_pending && top.dma_mem_valid &&
                                  top.dma_mem_ready;
        const bool dma_was_active = top.dma_mem_active;

        top.clk = 1;
        top.eval();

        if (mem_complete) {
            if (mem_write) {
                memory[mem_addr] = merge_bytes(memory[mem_addr], mem_wdata,
                                                mem_wmask);
                ++mem_writes;
            } else {
                ++mem_reads;
            }
            mem_pending = false;
        }
        if (cpu_complete) {
            if (cpu_write)
                ++cpu_writes;
            else
                ++cpu_reads;
            if (dma_was_active)
                ++cpu_completions_while_dma;
            cpu_pending = false;
            top.cpu_sram_wr_en = 0;
            top.cpu_sram_rd_en = 0;
        }
        top.clk = 0;
        top.eval();
        ++cycles;
    }

    void write_reg(uint8_t address, uint32_t data) {
        top.cpu_reg_addr = address;
        top.cpu_reg_wdata = data & 0x00ffffffu;
        top.cpu_reg_wr_en = 1;
        tick();
        top.cpu_reg_wr_en = 0;
    }

    uint32_t read_reg(uint8_t address) {
        top.cpu_reg_addr = address;
        top.cpu_reg_rd_en = 1;
        top.eval();
        const uint32_t value = top.cpu_reg_rdata & 0x00ffffffu;
        top.cpu_reg_rd_en = 0;
        return value;
    }

    void start_cpu(bool write, uint16_t address, uint32_t data = 0) {
        if (cpu_pending)
            fail("overlapped CPU SRAM requests in test fixture");
        cpu_pending = true;
        cpu_write = write;
        cpu_addr = address;
        cpu_wdata = data & 0x00ffffffu;
        top.cpu_sram_addr = address;
        top.cpu_sram_wdata = cpu_wdata;
        top.cpu_sram_wr_en = write;
        top.cpu_sram_rd_en = !write;
    }

    uint32_t cpu_access(bool write, uint16_t address, uint32_t data = 0) {
        start_cpu(write, address, data);
        for (unsigned i = 0; i < 100 && cpu_pending; ++i)
            tick();
        if (cpu_pending)
            fail("CPU SRAM request did not complete");
        return cpu_rdata;
    }

    void set_mem_word(uint32_t address, uint32_t value) {
        uint64_t &raw = memory[address & ~uint32_t(7)];
        if (address & 4)
            raw = (raw & 0xffffffff00000000ULL) | value;
        else
            raw = (raw & 0x00000000ffffffffULL) |
                  (uint64_t(value) << 32);
    }

    uint32_t mem_word(uint32_t address) const {
        const auto it = memory.find(address & ~uint32_t(7));
        const uint64_t raw = it == memory.end() ? 0 : it->second;
        return (address & 4) ? uint32_t(raw) : uint32_t(raw >> 32);
    }
};

void run_case(const Case &test) {
    Sim sim;
    constexpr uint16_t pbus = 0xc000;
    constexpr uint16_t physical = 0x4000;
    const uint32_t gio = 0x01000000u + test.lane;
    const unsigned transactions = test.folded ? (test.count + 1) / 2 :
                                                test.count;

    for (unsigned i = 0; i < test.count; ++i) {
        const uint32_t value = test.folded ? uint32_t(sample16(i)) << 8 :
                                             sample24(i);
        sim.cpu_access(true, physical + i,
                       test.to_host ? value : 0x00abcdefu);
    }
    for (unsigned i = 0; i < transactions; ++i) {
        const uint32_t value = test.folded ?
            (uint32_t(sample16(2 * i)) << 16) |
            uint32_t(2 * i + 1 < test.count ? sample16(2 * i + 1) :
                                             0x5aa5u) :
            (0xc3000000u | sample24(i));
        sim.set_mem_word(gio + 4 * i,
                         test.to_host ? (0xa5000000u ^ (i * 0x01010101u)) :
                                        value);
    }

    sim.write_reg(0x00, test.count);
    sim.write_reg(0x01, gio & 0xffffu);
    sim.write_reg(0x02, gio >> 16);
    sim.write_reg(0x03, pbus);
    sim.write_reg(0x04, 1u | (unsigned(test.to_host) << 1) |
                              (unsigned(test.folded) << 2));

    unsigned competitor_ordinal = 0;
    for (unsigned limit = 0; limit < 20000; ++limit) {
        if (!sim.cpu_pending && (limit % 4 == 0) &&
            competitor_ordinal < 64) {
            const uint16_t address = uint16_t(0x7000 + competitor_ordinal);
            const bool write = competitor_ordinal & 1;
            sim.start_cpu(write, address,
                          0x00700000u | competitor_ordinal);
            ++competitor_ordinal;
        }
        sim.tick();
        if (!sim.top.dma_mem_active && !(sim.read_reg(0x04) & 1) &&
            !sim.cpu_pending)
            break;
        if (limit == 19999)
            fail("DMA did not complete under CPU SRAM contention");
    }

    if (sim.mem_pending)
        fail("DMA completed with a memory response outstanding");
    if (sim.cpu_completions_while_dma < 4)
        fail("CPU competitor did not overlap the active DMA session");
    const uint32_t final_count = sim.read_reg(0x00);
    const uint32_t final_control = sim.read_reg(0x04);
    const uint32_t final_status = sim.read_reg(0x08);
    if (final_count != test.count || (final_control & 1) != 0 ||
        (final_status & 1) == 0) {
        std::fprintf(stderr,
            "completion boundary count=%04x control=%02x status=%02x "
            "active=%u mem=%uR/%uW cpu-overlap=%u cycles=%llu\n",
            final_count, final_control, final_status,
            sim.top.dma_mem_active, sim.mem_reads, sim.mem_writes,
            sim.cpu_completions_while_dma,
            static_cast<unsigned long long>(sim.cycles));
        fail("host DMA count/GO/completion status did not retire exactly");
    }

    if (test.to_host) {
        if (sim.mem_writes != transactions || sim.mem_reads != 0)
            fail("DSP-to-host memory transaction count changed");
        for (unsigned i = 0; i < transactions; ++i) {
            uint32_t expected;
            if (!test.folded) {
                expected = sample24(i);
            } else {
                expected = uint32_t(sample16(2 * i)) << 16;
                if (2 * i + 1 < test.count)
                    expected |= sample16(2 * i + 1);
                else
                    expected |= (0xa5000000u ^
                                 (i * 0x01010101u)) & 0xffffu;
            }
            if (sim.mem_word(gio + 4 * i) != expected)
                fail("DSP-to-host payload/lane/odd-tail changed");
        }
    } else {
        if (sim.mem_reads != transactions || sim.mem_writes != 0)
            fail("host-to-DSP memory transaction count changed");
        for (unsigned i = 0; i < test.count; ++i) {
            const uint32_t expected = test.folded ?
                uint32_t(sample16(i)) << 8 : sample24(i);
            if (sim.cpu_access(false, physical + i) != expected)
                fail("host-to-DSP SRAM payload/order changed");
        }
    }

    for (unsigned i = 0; i < competitor_ordinal; ++i) {
        if ((i & 1) && sim.cpu_access(false, 0x7000 + i) !=
                       (0x00700000u | i))
            fail("CPU competitor write was lost or cross-routed");
    }
    sim.write_reg(0x08, 0x06); // W0C completion while preserving TX/RX.
    if (sim.read_reg(0x08) & 1)
        fail("host DMA completion status did not clear");

    std::printf("PASS DSP top DMA mode=%s dir=%s count=%u lane=%s "
                "mem=%uR/%uW cpu=%uR/%uW overlap=%u competitors=%u\n",
                test.folded ? "folded" : "normal",
                test.to_host ? "DSP-to-host" : "host-to-DSP", test.count,
                test.lane ? "low" : "high", sim.mem_reads, sim.mem_writes,
                sim.cpu_reads, sim.cpu_writes,
                sim.cpu_completions_while_dma, competitor_ordinal);
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    for (const auto &test : kCases)
        run_case(test);
    std::puts("PASS DSP top DMA contention cases=8");
    return 0;
}
