// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 system-stack error exception boundary oracle
//
// Motorola specifies that the zero-to-one transition of SP.SE raises the
// nonmaskable IPL-3 exception at P:$0002.  Single and double stack operations
// retain their natural six-bit result ($10/$11 overflow, $3f/$3e underflow),
// and SE/UF remain sticky until software writes SP.  Exercise those rules
// through the production DSP core, memory, peripheral, and host hierarchy.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>
#include <initializer_list>

#include "Vdsp56001_core.h"
#include "Vdsp56001_core___024root.h"
#include "verilated.h"

namespace {

constexpr uint16_t kProgram = 0x0020;
constexpr uint16_t kHandler = 0x0040;

class StackHarness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};
    unsigned cycles = 0;
    bool saw_handler = false;

    StackHarness() {
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

        // Reset vector enters the test body. The stack-error vector is a
        // non-returning long jump because the fault is nonrecoverable.
        write_p(0x0000, 0x0af080); // JMP >$0020
        write_p(0x0001, kProgram);
        write_p(0x0002, 0x0af080); // stack-error vector: JMP >$0040
        write_p(0x0003, kHandler);
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;
        core.clk = 1;
        core.eval();
        if (fetch_en)
            core.p_fetch_data = p[fetch_addr & 0xff] & 0x00ffffff;
        ++cycles;
        saw_handler |= core.pc_out >= kHandler && core.pc_out < kHandler + 8;
    }

    void write_p(uint16_t addr, uint32_t word) {
        p[addr & 0xff] = word & 0x00ffffff;
    }

    void load(uint16_t addr, std::initializer_list<uint32_t> words) {
        for (uint32_t word : words)
            write_p(addr++, word);
    }

    void release_and_run(unsigned bound = 320) {
        core.run = 1;
        for (unsigned i = 0; i < bound; ++i)
            tick();
    }

    uint8_t sp() const {
        return core.rootp->dsp56001_core__DOT__sp;
    }

    uint16_t r(unsigned index) const {
        return core.rootp->dsp56001_core__DOT__r_reg[index & 7];
    }

    uint16_t la() const {
        return core.rootp->dsp56001_core__DOT__la;
    }

    uint32_t lc() const {
        return core.rootp->dsp56001_core__DOT__lc;
    }

    uint64_t ss0() const {
        return core.rootp->dsp56001_core__DOT__ss[0];
    }

    bool ss_any_nonzero() const {
        for (unsigned i = 0; i < 15; ++i) {
            if (core.rootp->dsp56001_core__DOT__ss[i] != 0)
                return true;
        }
        return false;
    }

    void stable_handler() {
        load(kHandler, {
            0x0af080, kHandler, // JMP >$0040
        });
    }
};

constexpr uint32_t movec_imm_sp(uint8_t value) {
    return 0x0500bb | (static_cast<uint32_t>(value) << 8);
}

constexpr uint32_t movec_imm_ssh(uint8_t value) {
    return 0x0500bc | (static_cast<uint32_t>(value) << 8);
}

bool single_push_overflow() {
    StackHarness h;
    h.load(kProgram, {
        movec_imm_sp(15),
        movec_imm_ssh(0x11),
        0x375500, // must not retire before the vector
        0x0af080, kProgram + 3,
    });
    h.load(kHandler, {
        movec_imm_ssh(0x22), // sticky SE blocks another push
        0x0af080, kHandler + 1,
    });
    h.release_and_run();

    const bool pass = h.saw_handler && h.sp() == 0x10 &&
                      !h.ss_any_nonzero() && h.r(7) == 0;
    std::printf("%s stack single-push overflow: handler=%u SP=$%02x "
                "SS-any=%u R7=$%04x cycles=%u\n",
                pass ? "PASS" : "FAIL", h.saw_handler, h.sp(),
                h.ss_any_nonzero(), h.r(7), h.cycles);
    return pass;
}

bool single_pull_underflow() {
    StackHarness h;
    h.load(kProgram, {
        movec_imm_sp(0),
        0x0450bc, // MOVEC SSH,R0: implicit postdecrement
        0x375500,
        0x0af080, kProgram + 3,
    });
    h.stable_handler();
    h.release_and_run();

    const bool pass = h.saw_handler && h.sp() == 0x3f &&
                      !h.ss_any_nonzero() && h.r(7) == 0;
    std::printf("%s stack single-pull underflow: handler=%u SP=$%02x "
                "SS-any=%u R7=$%04x cycles=%u\n",
                pass ? "PASS" : "FAIL", h.saw_handler, h.sp(),
                h.ss_any_nonzero(), h.r(7), h.cycles);
    return pass;
}

bool jsr_overflow() {
    StackHarness h;
    h.load(kProgram, {
        movec_imm_sp(15),
        0x0bf080, 0x0030, // JSR >$0030: implicit single push
        0x375500,
        0x0af080, kProgram + 4,
    });
    h.load(0x0030, {
        0x375500,
        0x0af080, 0x0031,
    });
    h.stable_handler();
    h.release_and_run();

    const bool pass = h.saw_handler && h.sp() == 0x10 &&
                      !h.ss_any_nonzero() && h.r(7) == 0;
    std::printf("%s stack JSR overflow: handler=%u SP=$%02x "
                "SS-any=%u R7=$%04x cycles=%u\n",
                pass ? "PASS" : "FAIL", h.saw_handler, h.sp(),
                h.ss_any_nonzero(), h.r(7), h.cycles);
    return pass;
}

bool rts_underflow() {
    StackHarness h;
    h.load(kProgram, {
        movec_imm_sp(0),
        0x00000c, // RTS: implicit single pull
        0x375500,
        0x0af080, kProgram + 3,
    });
    h.stable_handler();
    h.release_and_run();

    const bool pass = h.saw_handler && h.sp() == 0x3f &&
                      !h.ss_any_nonzero() && h.r(7) == 0;
    std::printf("%s stack RTS underflow: handler=%u SP=$%02x "
                "SS-any=%u R7=$%04x cycles=%u\n",
                pass ? "PASS" : "FAIL", h.saw_handler, h.sp(),
                h.ss_any_nonzero(), h.r(7), h.cycles);
    return pass;
}

bool double_push_overflow(uint8_t initial_sp, uint8_t expected_sp) {
    StackHarness h;
    h.load(kProgram, {
        movec_imm_sp(initial_sp),
        0x060180, 0x0025, // DO #1,$0025: implicit double push
        0x375500,
        0x0af080, kProgram + 4,
    });
    h.stable_handler();
    h.release_and_run();

    const bool pass = h.saw_handler && h.sp() == expected_sp &&
                      !h.ss_any_nonzero() && h.r(7) == 0;
    std::printf("%s stack DO overflow from $%02x: handler=%u SP=$%02x "
                "expected=$%02x SS-any=%u cycles=%u\n",
                pass ? "PASS" : "FAIL", initial_sp, h.saw_handler,
                h.sp(), expected_sp, h.ss_any_nonzero(),
                h.cycles);
    return pass;
}

bool double_pull_underflow(uint8_t initial_sp, uint8_t expected_sp) {
    StackHarness h;
    h.load(kProgram, {
        movec_imm_sp(initial_sp),
        0x00008c, // ENDDO: implicit double pull
        0x375500,
        0x0af080, kProgram + 3,
    });
    h.stable_handler();
    h.release_and_run();

    const bool pass = h.saw_handler && h.sp() == expected_sp &&
                      !h.ss_any_nonzero() && h.r(7) == 0;
    std::printf("%s stack ENDDO underflow from $%02x: handler=%u SP=$%02x "
                "expected=$%02x SS-any=%u cycles=%u\n",
                pass ? "PASS" : "FAIL", initial_sp, h.saw_handler,
                h.sp(), expected_sp, h.ss_any_nonzero(),
                h.cycles);
    return pass;
}

bool explicit_sp_write_clears_sticky_state() {
    StackHarness h;
    h.load(kProgram, {
        movec_imm_sp(15),
        movec_imm_ssh(0x11),
        0x0af080, kProgram + 2,
    });
    h.load(kHandler, {
        movec_imm_sp(0),       // clear SE/UF deliberately
        movec_imm_ssh(0x33),   // ordinary push works again
        0x0af080, kHandler + 2,
    });
    h.release_and_run();

    const uint64_t expected_ss0 = 0x000033000000ULL;
    const bool pass = h.saw_handler && h.sp() == 0x01 &&
                      h.ss0() == expected_ss0;
    std::printf("%s explicit SP write clears stack error: handler=%u "
                "SP=$%02x SS0=%012llx cycles=%u\n",
                pass ? "PASS" : "FAIL", h.saw_handler, h.sp(),
                static_cast<unsigned long long>(h.ss0()), h.cycles);
    return pass;
}

bool forced_uf_single_push_normal() {
    StackHarness h;
    h.load(kProgram, {
        movec_imm_sp(0x20),
        movec_imm_ssh(0x55),
        0x0af080, kProgram + 2,
    });
    h.stable_handler();
    h.release_and_run();

    const uint64_t expected_ss0 = 0x000055000000ULL;
    const bool pass = !h.saw_handler && h.sp() == 0x01 &&
                      h.ss0() == expected_ss0;
    std::printf("%s forced-UF single push normal: handler=%u SP=$%02x "
                "SS0=%012llx cycles=%u\n", pass ? "PASS" : "FAIL",
                h.saw_handler, h.sp(),
                static_cast<unsigned long long>(h.ss0()), h.cycles);
    return pass;
}

bool forced_uf_single_pull_normal() {
    StackHarness h;
    h.load(kProgram, {
        movec_imm_ssh(0x55), // populate the real top at SP=1
        movec_imm_sp(0x21),  // force UF while preserving pointer one
        0x0450bc,            // MOVEC SSH,R0: correcting single pull
        0x000000,
        0x0af080, kProgram + 4,
    });
    h.stable_handler();
    h.release_and_run();

    const uint64_t expected_ss0 = 0x000055000000ULL;
    const bool pass = !h.saw_handler && h.sp() == 0x00 &&
                      h.r(0) == 0x0055 &&
                      h.ss0() == expected_ss0;
    std::printf("%s forced-UF single pull normal: handler=%u SP=$%02x "
                "R0=$%04x SS0=%012llx cycles=%u\n",
                pass ? "PASS" : "FAIL", h.saw_handler, h.sp(),
                h.r(0),
                static_cast<unsigned long long>(h.ss0()), h.cycles);
    return pass;
}

bool forced_uf_single_pull_error() {
    StackHarness h;
    h.load(kProgram, {
        movec_imm_sp(0x20),
        0x0450bc, // MOVEC SSH,R0: correcting pull underflows
        0x375500, // must not retire before P:$0002
        0x0af080, kProgram + 3,
    });
    h.stable_handler();
    h.release_and_run();

    const bool pass = h.saw_handler && h.sp() == 0x3f &&
                      h.r(0) == 0 && h.r(7) == 0 &&
                      !h.ss_any_nonzero();
    std::printf("%s forced-UF single pull error: handler=%u SP=$%02x "
                "R0=$%04x R7=$%04x SS-any=%u cycles=%u\n",
                pass ? "PASS" : "FAIL", h.saw_handler, h.sp(),
                h.r(0), h.r(7), h.ss_any_nonzero(),
                h.cycles);
    return pass;
}

bool forced_uf_double_push_normal() {
    StackHarness h;
    h.load(kProgram, {
        movec_imm_sp(0x20),
        0x060180, 0x0030, // DO #1,$0030: correcting double push
        0x0af080, kProgram + 3,
    });
    h.stable_handler();
    h.release_and_run();

    const bool pass = !h.saw_handler && h.sp() == 0x02 &&
                      h.la() == 0x0030 && h.lc() == 1 &&
                      (h.core.sr_out & 0x008000) != 0 &&
                      h.ss_any_nonzero();
    std::printf("%s forced-UF double push normal: handler=%u SP=$%02x "
                "LA=$%04x LC=$%06x LF=%u SS-any=%u cycles=%u\n",
                pass ? "PASS" : "FAIL", h.saw_handler, h.sp(),
                h.la(), h.lc(),
                (h.core.sr_out >> 15) & 1, h.ss_any_nonzero(),
                h.cycles);
    return pass;
}

bool forced_uf_double_push_error() {
    StackHarness h;
    h.load(kProgram, {
        movec_imm_sp(0x2e),
        0x060180, 0x0030, // normalized pointer 14 overflows to $10
        0x375500,
        0x0af080, kProgram + 4,
    });
    h.stable_handler();
    h.release_and_run();

    const bool pass = h.saw_handler && h.sp() == 0x10 &&
                      h.la() == 0 && h.lc() == 0 &&
                      h.r(7) == 0 && !h.ss_any_nonzero();
    std::printf("%s forced-UF double push error: handler=%u SP=$%02x "
                "LA=$%04x LC=$%06x R7=$%04x SS-any=%u cycles=%u\n",
                pass ? "PASS" : "FAIL", h.saw_handler, h.sp(),
                h.la(), h.lc(), h.r(7),
                h.ss_any_nonzero(), h.cycles);
    return pass;
}

bool forced_uf_double_pull_normal() {
    StackHarness h;
    h.load(kProgram, {
        0x060180, 0x0030, // create a valid two-entry DO frame
        movec_imm_sp(0x22),
        0x00008c,          // ENDDO: correcting double pull
        0x0af080, kProgram + 4,
    });
    h.stable_handler();
    h.release_and_run();

    const bool pass = !h.saw_handler && h.sp() == 0x00 &&
                      h.la() == 0 && h.lc() == 0 &&
                      (h.core.sr_out & 0x008000) == 0;
    std::printf("%s forced-UF double pull normal: handler=%u SP=$%02x "
                "LA=$%04x LC=$%06x LF=%u cycles=%u\n",
                pass ? "PASS" : "FAIL", h.saw_handler, h.sp(),
                h.la(), h.lc(),
                (h.core.sr_out >> 15) & 1, h.cycles);
    return pass;
}

bool forced_uf_double_pull_error() {
    StackHarness h;
    h.load(kProgram, {
        movec_imm_sp(0x20),
        0x00008c, // ENDDO: normalized pointer zero underflows to $3e
        0x375500,
        0x0af080, kProgram + 3,
    });
    h.stable_handler();
    h.release_and_run();

    const bool pass = h.saw_handler && h.sp() == 0x3e &&
                      h.la() == 0 && h.lc() == 0 &&
                      h.r(7) == 0 && !h.ss_any_nonzero();
    std::printf("%s forced-UF double pull error: handler=%u SP=$%02x "
                "LA=$%04x LC=$%06x R7=$%04x SS-any=%u cycles=%u\n",
                pass ? "PASS" : "FAIL", h.saw_handler, h.sp(),
                h.la(), h.lc(), h.r(7),
                h.ss_any_nonzero(), h.cycles);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    unsigned failures = 0;
    failures += !single_push_overflow();
    failures += !single_pull_underflow();
    failures += !jsr_overflow();
    failures += !rts_underflow();
    failures += !double_push_overflow(14, 0x10);
    failures += !double_push_overflow(15, 0x11);
    failures += !double_pull_underflow(0, 0x3e);
    failures += !double_pull_underflow(1, 0x3f);
    failures += !explicit_sp_write_clears_sticky_state();
    failures += !forced_uf_single_push_normal();
    failures += !forced_uf_single_pull_normal();
    failures += !forced_uf_single_pull_error();
    failures += !forced_uf_double_push_normal();
    failures += !forced_uf_double_push_error();
    failures += !forced_uf_double_pull_normal();
    failures += !forced_uf_double_pull_error();
    if (failures != 0)
        std::printf("FAIL DSP56001 stack-error boundary failures=%u/16\n",
                    failures);
    else
        std::printf("PASS DSP56001 stack-error boundary cases=16\n");
    return failures == 0 ? 0 : 1;
}
