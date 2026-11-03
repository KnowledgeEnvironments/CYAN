// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

#include "Vdsp56001_host.h"

#include <cstdint>
#include <cstdio>
#include <stdexcept>

// DSP INTSTAT is distinct from CPU CINTSTAT. Its defined event positions are
// DMA_DONE=0, HANDTX acknowledgement=5, and HANDRX arrival=6. This exercises
// the already admitted event-wins W0C contract at the remaining DMA boundary.
namespace {
constexpr unsigned DMACTRL = 4, HANDTX = 6, HANDRX = 7, STATUS = 8, MASK = 9;
unsigned completed = 0;
unsigned seed_case, clear_case, host_case, dma_case, mask_case;

unsigned events(unsigned value) {
    return (value & 1u) | ((value & 2u) << 4) | ((value & 4u) << 4);
}

void expect(const char *boundary, unsigned actual, unsigned wanted) {
    if (actual == wanted) return;
    std::fprintf(stderr,
        "FAIL dsp_host_dsp_dma_w0c_collision: %s actual=0x%06x expected=0x%06x "
        "old=0x%02x clear=0x%02x host=%u dma=%u mask=0x%02x completed=%u\n",
        boundary, actual, wanted, seed_case, clear_case, host_case,
        dma_case, mask_case, completed);
    throw std::runtime_error(boundary);
}

void clear_requests(Vdsp56001_host &top) {
    top.cpu_rd_en = top.cpu_wr_en = 0;
    top.dsp_rd_en = top.dsp_wr_en = 0;
    top.dma_done = 0;
}

void tick(Vdsp56001_host &top) {
    top.clk = 0; top.eval();
    top.clk = 1; top.eval();
    top.clk = 0; top.eval();
}

void finish_request(Vdsp56001_host &top) {
    tick(top);
    clear_requests(top);
    top.eval();
}

void cpu_write(Vdsp56001_host &top, unsigned address, unsigned value) {
    clear_requests(top);
    top.cpu_addr = address; top.cpu_wdata = value; top.cpu_wr_en = 1;
    finish_request(top);
}

void dsp_write(Vdsp56001_host &top, unsigned address, unsigned value) {
    clear_requests(top);
    top.dsp_addr = address; top.dsp_wdata = value; top.dsp_wr_en = 1;
    finish_request(top);
}

unsigned dsp_status(Vdsp56001_host &top) {
    clear_requests(top);
    top.dsp_addr = STATUS; top.eval();
    return top.dsp_rdata;
}

unsigned cpu_status(Vdsp56001_host &top) {
    clear_requests(top);
    top.cpu_addr = STATUS; top.eval();
    return top.cpu_rdata;
}

void run_case(Vdsp56001_host &top, unsigned old, unsigned preserve,
              unsigned host_event, bool dma_event, unsigned mask) {
    seed_case = old; clear_case = preserve; host_case = host_event;
    dma_case = dma_event; mask_case = mask;
    clear_requests(top);
    top.cpu_addr = top.dsp_addr = 0;
    top.cpu_wdata = top.dsp_wdata = 0;
    top.dma_count_current = 0;
    top.rst_n = 0; tick(top); tick(top);
    top.rst_n = 1; tick(top);
    expect("reset DSP status", dsp_status(top), 0);
    if (old & 1u) {
        top.dma_done = 1; finish_request(top);
    }
    if (old & 0x20u) {
        top.cpu_addr = HANDTX; top.cpu_rd_en = 1; finish_request(top);
    }
    if (old & 0x40u) cpu_write(top, HANDRX, 0x1234);
    expect("independent seeded DSP causes", dsp_status(top), old);
    cpu_write(top, STATUS, 0); // CPU acknowledgement cannot clear DSP causes.
    cpu_write(top, MASK, 1);   // Independent CPU DMA mask.
    cpu_write(top, DMACTRL, 1);
    dsp_write(top, MASK, mask);
    expect("seed survived CPU W0C", dsp_status(top), old);
    expect("seed DSP IRQ mask", top.irq_to_dsp, (old & mask) != 0);
    expect("seed CPU status clear", cpu_status(top), 0);
    expect("seed DMA GO", top.dma_go, 1);

    // One real peripheral write and at most one CPU register operation share
    // the edge with the independent engine's one-cycle completion pulse.
    clear_requests(top);
    top.dsp_addr = STATUS;
    top.dsp_wdata = 0xffff00u | preserve; // High bits cannot create causes.
    top.dsp_wr_en = 1;
    top.dma_done = dma_event;
    unsigned new_events = dma_event ? 1u : 0u;
    if (host_event == 1) {
        top.cpu_addr = HANDTX; top.cpu_rd_en = 1;
        new_events |= 0x20;
    } else if (host_event == 2) {
        top.cpu_addr = HANDRX; top.cpu_wdata = 0xcafe; top.cpu_wr_en = 1;
        new_events |= 0x40;
    }
    finish_request(top);
    const unsigned expected_status = (old & preserve) | new_events;
    // Check the completed engine/CPU boundary first to distinguish a missing
    // DMA event from a DSP-side event overwritten by the W0C operation.
    expect("DMA event reached CPU status", cpu_status(top), dma_event ? 1 : 0);
    expect("DMA event reached CPU IRQ", top.irq_to_cpu, dma_event ? 1 : 0);
    expect("DMA completion cleared GO", top.dma_go, dma_event ? 0 : 1);
    expect("new DMA/host events survive DSP W0C", dsp_status(top), expected_status);
    expect("new event drives selected DSP IRQ", top.irq_to_dsp,
           (expected_status & mask) != 0);

    // No pulse may be recreated. Ones preserve the result; a later ordinary
    // zero write must clear it without disturbing CPU status or DMA GO.
    for (unsigned i = 0; i < 3; ++i) tick(top);
    expect("completion persisted after pulse", dsp_status(top), expected_status);
    dsp_write(top, STATUS, 0xffffff);
    expect("all-one W0C preserves", dsp_status(top), expected_status);
    dsp_write(top, STATUS, 0);
    expect("ordinary zero W0C clears", dsp_status(top), 0);
    expect("ordinary W0C clears IRQ", top.irq_to_dsp, 0);
    expect("DSP acknowledgement preserves CPU status", cpu_status(top), dma_event ? 1 : 0);
    expect("DSP acknowledgement preserves GO", top.dma_go, dma_event ? 0 : 1);
    ++completed;
}
} // namespace

int main() {
    Vdsp56001_host top;
    try {
        // Minimal fail-before: preserve old HANDRX, clear old DMA, and publish
        // a new completion. Only DMA is enabled for DSP IRQ. Old RTL returns
        // 0x40 while CPU completion/IRQ and GO clearing are already correct.
        run_case(top, 0x40, 0x40, 0, true, 1);
        for (unsigned old = 0; old < 8; ++old)
            for (unsigned preserve = 0; preserve < 8; ++preserve)
                for (unsigned host = 0; host < 3; ++host)
                    for (unsigned dma = 0; dma < 2; ++dma)
                        for (unsigned mask = 0; mask < 8; ++mask)
                            run_case(top, events(old), events(preserve), host,
                                     dma != 0, events(mask));
        expect("complete matrix", completed, 3073);
    } catch (const std::exception &) {
        top.final();
        return 1;
    }
    top.final();
    std::printf("PASS dsp_host_dsp_dma_w0c_collision cases=%u "
                "old=8 clear=8 host=3 dma=2 mask=8 event_wins=1 "
                "cpu_independent=1 go=checked post_clear=checked\n", completed);
    return 0;
}
