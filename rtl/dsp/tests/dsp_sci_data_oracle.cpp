// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 SCI character/status engine oracle
//
// This is the word boundary between the architectural STX/SRX double buffers
// and a physical pin serializer.  It proves the programming-visible state
// transitions independently of baud/pin timing: TX handoff/completion,
// normal/error/overrun RX, the SSR-then-SRX error clear sequence, and the
// one-shot idle request lifecycle.
///////////////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <cstdio>

#include "Vdsp56001_periph.h"
#include "verilated.h"

namespace {

constexpr uint8_t A_PCC  = 0x21;
constexpr uint8_t A_SCR  = 0x30;
constexpr uint8_t A_SSR  = 0x31;
constexpr uint8_t A_STXL = 0x34;

class Harness {
public:
    Vdsp56001_periph dut;

    Harness() {
        dut.clk = 0;
        dut.rst_n = 0;
        dut.clock_enable = 1;
        dut.soft_reset = 0;
        dut.stop_reset = 0;
        dut.addr = 0;
        dut.wdata = 0;
        dut.rd_en = 0;
        dut.wr_en = 0;
        dut.ssi_tx_ready = 0;
        dut.ssi_rx_valid = 0;
        dut.ssi_rx_data = 0;
        dut.sci_timer_ack = 0;
        dut.sci_idle_ack = 0;
        dut.sci_tx_ready = 0;
        dut.sci_tx_done = 0;
        dut.sci_rx_valid = 0;
        dut.sci_rx_data = 0;
        dut.sci_rx_r8 = 0;
        dut.sci_rx_framing_error = 0;
        dut.sci_rx_parity_error = 0;
        dut.sci_rx_idle = 0;
        for (unsigned i = 0; i < 3; ++i)
            tick();
        dut.rst_n = 1;
        tick();
    }

    void tick() {
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        dut.clk = 0;
        dut.eval();
    }

    void write(uint8_t addr, uint32_t data) {
        dut.addr = addr;
        dut.wdata = data & 0x00ffffff;
        dut.wr_en = 1;
        tick();
        dut.wr_en = 0;
        tick();
    }

    uint32_t read(uint8_t addr) {
        dut.addr = addr;
        dut.rd_en = 1;
        dut.clk = 0;
        dut.eval();
        const uint32_t value = dut.rdata & 0x00ffffff;
        dut.clk = 1;
        dut.eval();
        dut.clk = 0;
        dut.eval();
        dut.rd_en = 0;
        return value;
    }

    void rx(uint8_t data, bool r8 = false, bool fe = false, bool pe = false) {
        dut.sci_rx_data = data;
        dut.sci_rx_r8 = r8;
        dut.sci_rx_framing_error = fe;
        dut.sci_rx_parity_error = pe;
        dut.sci_rx_valid = 1;
        tick();
        dut.sci_rx_valid = 0;
        dut.sci_rx_framing_error = 0;
        dut.sci_rx_parity_error = 0;
        tick();
    }

    uint32_t read_while_rx(uint8_t data, bool r8 = false,
                           bool fe = false, bool pe = false) {
        dut.addr = A_STXL;
        dut.rd_en = 1;
        dut.sci_rx_data = data;
        dut.sci_rx_r8 = r8;
        dut.sci_rx_framing_error = fe;
        dut.sci_rx_parity_error = pe;
        dut.sci_rx_valid = 1;
        dut.clk = 0;
        dut.eval();
        const uint32_t value = dut.rdata & 0x00ffffff;
        dut.clk = 1;
        dut.eval();
        dut.clk = 0;
        dut.eval();
        dut.rd_en = 0;
        dut.sci_rx_valid = 0;
        dut.sci_rx_framing_error = 0;
        dut.sci_rx_parity_error = 0;
        dut.eval();
        return value;
    }
};

bool tx_lifecycle() {
    Harness h;
    h.write(A_PCC, 1u << 1); // TXD pin
    h.write(A_SCR, (1u << 12) | (1u << 9)); // TIE | TE
    h.write(A_STXL, 0xa5);
    const uint32_t queued = h.read(A_SSR);

    h.dut.sci_tx_ready = 1;
    h.tick();
    const bool handoff = h.dut.sci_tx_valid && h.dut.sci_tx_data == 0xa5 &&
                         !h.dut.sci_tx_address;
    h.dut.sci_tx_ready = 0;
    h.tick();
    const uint32_t shifting = h.read(A_SSR);

    h.dut.sci_tx_done = 1;
    h.tick();
    h.dut.sci_tx_done = 0;
    h.tick();
    const uint32_t done = h.read(A_SSR);

    const bool ok = (queued & 0x3) == 0 && handoff &&
                    (shifting & 0x3) == 0x2 && h.dut.irq_sci_tx &&
                    (done & 0x3) == 0x3;
    std::printf("%s SCI TX character lifecycle queued=%02x shift=%02x done=%02x "
                "data=%02x\n", ok ? "PASS" : "FAIL", queued & 0xff,
                shifting & 0xff, done & 0xff, h.dut.sci_tx_data);
    return ok;
}

bool receive_lifecycle() {
    Harness h;
    h.write(A_PCC, 1u << 0); // RXD pin
    // Select an 11-bit parity format so the injected PE is architecturally
    // meaningful; WDS=0 is synchronous and requires PE to remain clear.
    h.write(A_SCR, (1u << 11) | (1u << 8) | 4u); // RIE | RE | even parity

    h.rx(0xc3, true);
    const uint32_t normal = h.read(A_SSR);
    const bool normal_irq = h.dut.irq_sci_rx && !h.dut.irq_sci_rx_exception;
    const uint32_t normal_data = h.read(A_STXL);
    const uint32_t after_normal = h.read(A_SSR);

    h.rx(0x5a, false, false, true);
    const uint32_t parity = h.read(A_SSR); // arms error clear
    const bool exception_irq = !h.dut.irq_sci_rx &&
                               h.dut.irq_sci_rx_exception;
    const uint32_t parity_data = h.read(A_STXL);
    const uint32_t after_parity = h.read(A_SSR);

    h.rx(0x11);
    h.rx(0x22, false, true, true); // overrun wins; preserve first byte
    const uint32_t overrun = h.read(A_SSR);
    const uint32_t preserved = h.read(A_STXL);
    const uint32_t after_overrun = h.read(A_SSR);

    const bool ok = (normal & 0x84) == 0x84 && normal_irq &&
                    normal_data == 0xc3 && (after_normal & 0x04) == 0 &&
                    (parity & 0x24) == 0x24 && exception_irq &&
                    parity_data == 0x5a && (after_parity & 0x74) == 0 &&
                    (overrun & 0x14) == 0x14 &&
                    (overrun & 0x60) == 0 && preserved == 0x11 &&
                    (after_overrun & 0x74) == 0;
    std::printf("%s SCI RX normal/error/overrun normal=%02x parity=%02x "
                "overrun=%02x data=%02x\n", ok ? "PASS" : "FAIL",
                normal & 0xff, parity & 0xff, overrun & 0xff, preserved);
    return ok;
}

bool receive_error_format_lifecycle() {
    // DSP56001UM11 sections 11.2.2.2.6--7 make receive-error visibility
    // format dependent.  PE exists only in the even/odd parity formats
    // (WDS=4/5); it must remain clear in synchronous, 10-bit asynchronous,
    // and multidrop modes.  FE exists only in asynchronous formats and must
    // remain clear in synchronous mode.  Keep positive controls for both
    // flags so an implementation cannot satisfy this oracle by discarding
    // every physical error indication.
    Harness ten_bit;
    ten_bit.write(A_PCC, 1u << 0);
    ten_bit.write(A_SCR, (1u << 11) | (1u << 8) | 2u);
    ten_bit.rx(0x12, false, false, true);
    const bool ten_bit_normal_irq = ten_bit.dut.irq_sci_rx &&
                                    !ten_bit.dut.irq_sci_rx_exception;
    const uint32_t ten_bit_status = ten_bit.read(A_SSR);
    const uint32_t ten_bit_data = ten_bit.read(A_STXL);

    Harness multidrop;
    multidrop.write(A_PCC, 1u << 0);
    multidrop.write(A_SCR, (1u << 11) | (1u << 8) | 6u);
    multidrop.rx(0x34, true, false, true);
    const bool multidrop_normal_irq = multidrop.dut.irq_sci_rx &&
                                      !multidrop.dut.irq_sci_rx_exception;
    const uint32_t multidrop_status = multidrop.read(A_SSR);
    const uint32_t multidrop_data = multidrop.read(A_STXL);

    Harness synchronous;
    synchronous.write(A_PCC, 1u << 0);
    synchronous.write(A_SCR, (1u << 11) | (1u << 8));
    synchronous.rx(0x56, false, true, true);
    const bool synchronous_normal_irq = synchronous.dut.irq_sci_rx &&
                                        !synchronous.dut.irq_sci_rx_exception;
    const uint32_t synchronous_status = synchronous.read(A_SSR);
    const uint32_t synchronous_data = synchronous.read(A_STXL);

    Harness parity;
    parity.write(A_PCC, 1u << 0);
    parity.write(A_SCR, (1u << 11) | (1u << 8) | 4u);
    parity.rx(0x78, false, false, true);
    const bool parity_exception_irq = !parity.dut.irq_sci_rx &&
                                      parity.dut.irq_sci_rx_exception;
    const uint32_t parity_status = parity.read(A_SSR);
    const uint32_t parity_data = parity.read(A_STXL);

    Harness framing;
    framing.write(A_PCC, 1u << 0);
    framing.write(A_SCR, (1u << 11) | (1u << 8) | 2u);
    framing.rx(0x9a, false, true, false);
    const bool framing_exception_irq = !framing.dut.irq_sci_rx &&
                                       framing.dut.irq_sci_rx_exception;
    const uint32_t framing_status = framing.read(A_SSR);
    const uint32_t framing_data = framing.read(A_STXL);

    const bool ok = ten_bit_normal_irq &&
                    (ten_bit_status & 0x64) == 0x04 &&
                    ten_bit_data == 0x12 && multidrop_normal_irq &&
                    (multidrop_status & 0xe4) == 0x84 &&
                    multidrop_data == 0x34 && synchronous_normal_irq &&
                    (synchronous_status & 0x64) == 0x04 &&
                    synchronous_data == 0x56 && parity_exception_irq &&
                    (parity_status & 0x64) == 0x24 &&
                    parity_data == 0x78 && framing_exception_irq &&
                    (framing_status & 0x64) == 0x44 &&
                    framing_data == 0x9a;
    std::printf("%s SCI RX format errors ten=%02x multi=%02x sync=%02x "
                "parity=%02x framing=%02x irq=%u%u%u%u%u\n",
                ok ? "PASS" : "FAIL", ten_bit_status & 0xff,
                multidrop_status & 0xff, synchronous_status & 0xff,
                parity_status & 0xff, framing_status & 0xff,
                ten_bit_normal_irq, multidrop_normal_irq,
                synchronous_normal_irq, parity_exception_irq,
                framing_exception_irq);
    return ok;
}

bool receive_event_clear_priority() {
    // DSP56001UM11 sections 11.2.2.2.3 and 11.2.2.2.5 define RDRF and OR
    // as receive events, cleared by reading SRX (with the preceding SSR read
    // additionally required for OR).  At this normalized character boundary,
    // a fresh event on the clearing edge is newer state and must survive.
    Harness arrival;
    arrival.write(A_PCC, 1u << 0);
    arrival.write(A_SCR, (1u << 11) | (1u << 8) | 2u);
    const uint32_t prior = arrival.read_while_rx(0x5a);
    const uint32_t arrival_status = arrival.read(A_SSR);
    const bool arrival_irq = arrival.dut.irq_sci_rx &&
                             !arrival.dut.irq_sci_rx_exception;
    const uint32_t arrived = arrival.read(A_STXL);
    const bool arrival_ok = prior == 0 &&
                            (arrival_status & 0x04) == 0x04 &&
                            arrival_irq && arrived == 0x5a;
    std::printf("%s SCI coincident RX arrival/read: prior=%02x "
                "status=%02x irq=%u received=%02x\n",
                arrival_ok ? "PASS" : "FAIL", prior & 0xff,
                arrival_status & 0xff, arrival_irq, arrived & 0xff);

    Harness overrun;
    overrun.write(A_PCC, 1u << 0);
    overrun.write(A_SCR, (1u << 11) | (1u << 8) | 2u);
    overrun.rx(0x11);
    overrun.rx(0x22);
    (void)overrun.read(A_SSR); // arm the documented OR clear sequence
    const uint32_t consumed = overrun.read_while_rx(0x33);
    const uint32_t collision_status = overrun.read(A_SSR);
    overrun.rx(0x44);
    const uint32_t accepted_status = overrun.read(A_SSR);
    const bool exception = !overrun.dut.irq_sci_rx &&
                           overrun.dut.irq_sci_rx_exception;
    const uint32_t accepted = overrun.read(A_STXL);
    const bool overrun_ok = consumed == 0x11 &&
                            (collision_status & 0x14) == 0x10 &&
                            (accepted_status & 0x14) == 0x14 &&
                            exception && accepted == 0x44;
    std::printf("%s SCI coincident OR event/clear: consumed=%02x "
                "collision=%02x accepted=%02x exception=%u data=%02x\n",
                overrun_ok ? "PASS" : "FAIL", consumed & 0xff,
                collision_status & 0xff, accepted_status & 0xff,
                exception, accepted & 0xff);

    // Consuming SRX without first reading SSR clears RDRF but deliberately
    // leaves the old error indication set.  A later SSR read arms the error
    // clear sequence.  If a new errored character reaches the empty receiver
    // on the clearing SRX-read edge, its FE/PE state is newer than the armed
    // clear and must remain visible with the newly accepted byte.
    Harness parity_collision;
    parity_collision.write(A_PCC, 1u << 0);
    parity_collision.write(A_SCR, (1u << 11) | (1u << 8) | 4u);
    parity_collision.rx(0x51, false, false, true);
    const uint32_t old_parity_data = parity_collision.read(A_STXL);
    (void)parity_collision.read(A_SSR);
    const uint32_t parity_prior =
        parity_collision.read_while_rx(0x62, false, false, true);
    const uint32_t parity_collision_status = parity_collision.read(A_SSR);
    const bool parity_collision_irq = !parity_collision.dut.irq_sci_rx &&
                                      parity_collision.dut.irq_sci_rx_exception;
    const uint32_t new_parity_data = parity_collision.read(A_STXL);
    const bool parity_collision_ok = old_parity_data == 0x51 &&
        parity_prior == 0x51 &&
        (parity_collision_status & 0x24) == 0x24 &&
        parity_collision_irq && new_parity_data == 0x62;
    std::printf("%s SCI coincident PE event/clear: prior=%02x status=%02x "
                "exception=%u data=%02x\n",
                parity_collision_ok ? "PASS" : "FAIL", parity_prior & 0xff,
                parity_collision_status & 0xff, parity_collision_irq,
                new_parity_data & 0xff);

    Harness framing_collision;
    framing_collision.write(A_PCC, 1u << 0);
    framing_collision.write(A_SCR, (1u << 11) | (1u << 8) | 4u);
    framing_collision.rx(0x73, false, false, true);
    const uint32_t old_framing_data = framing_collision.read(A_STXL);
    (void)framing_collision.read(A_SSR);
    const uint32_t framing_prior =
        framing_collision.read_while_rx(0x84, false, true, false);
    const uint32_t framing_collision_status = framing_collision.read(A_SSR);
    const bool framing_collision_irq = !framing_collision.dut.irq_sci_rx &&
                                       framing_collision.dut.irq_sci_rx_exception;
    const uint32_t new_framing_data = framing_collision.read(A_STXL);
    const bool framing_collision_ok = old_framing_data == 0x73 &&
        framing_prior == 0x73 &&
        (framing_collision_status & 0x44) == 0x44 &&
        framing_collision_irq && new_framing_data == 0x84;
    std::printf("%s SCI coincident FE event/clear: prior=%02x status=%02x "
                "exception=%u data=%02x\n",
                framing_collision_ok ? "PASS" : "FAIL", framing_prior & 0xff,
                framing_collision_status & 0xff, framing_collision_irq,
                new_framing_data & 0xff);

    return arrival_ok && overrun_ok && parity_collision_ok &&
           framing_collision_ok;
}

bool idle_lifecycle() {
    Harness h;
    h.write(A_PCC, 1u << 0);
    h.write(A_SCR, (1u << 10) | (1u << 8)); // ILIE | RE

    h.dut.sci_rx_idle = 1;
    h.tick();
    h.dut.sci_rx_idle = 0;
    const bool rejected_before_start = !h.dut.irq_sci_idle;

    h.rx(0x44);
    (void)h.read(A_STXL);
    h.dut.sci_rx_idle = 1;
    h.tick();
    h.dut.sci_rx_idle = 0;
    h.tick();
    const uint32_t idle = h.read(A_SSR);
    const bool asserted = h.dut.irq_sci_idle;

    h.dut.sci_idle_ack = 1;
    h.tick();
    h.dut.sci_idle_ack = 0;
    h.tick();
    const bool acked = !h.dut.irq_sci_idle && (h.read(A_SSR) & 0x08);

    h.rx(0x55);
    const bool start_clears_status = (h.read(A_SSR) & 0x08) == 0;
    const bool ok = rejected_before_start && (idle & 0x08) && asserted &&
                    acked && start_clears_status;
    std::printf("%s SCI idle one-shot pre=%u idle=%02x irq=%u ack=%u clear=%u\n",
                ok ? "PASS" : "FAIL", rejected_before_start, idle & 0xff,
                asserted, acked, start_clears_status);
    return ok;
}

bool wakeup_lifecycle() {
    // Motorola section 11.2.2.1.5: with RWU set in asynchronous mode, all
    // receive flags except IDLE are disabled.  WAKE=0 clears RWU after an
    // idle string; only the following character is transferred to SRX.
    Harness idle;
    idle.write(A_PCC, 1u << 0);
    idle.write(A_SCR, (1u << 11) | (1u << 10) | (1u << 8) | (1u << 6) |
                       2u);
    idle.rx(0x11);
    const uint32_t idle_sleep_status = idle.read(A_SSR);
    const bool idle_ignored = !(idle_sleep_status & 0x04) &&
                              !idle.dut.irq_sci_rx;

    idle.dut.sci_rx_idle = 1;
    idle.tick();
    idle.dut.sci_rx_idle = 0;
    idle.tick();
    const uint32_t idle_wake_scr = idle.read(A_SCR);
    const uint32_t idle_wake_status = idle.read(A_SSR);
    const bool idle_woke = !(idle_wake_scr & (1u << 6)) &&
                           (idle_wake_status & 0x08) &&
                           !idle.dut.irq_sci_idle;

    idle.rx(0x22);
    const uint32_t idle_data_status = idle.read(A_SSR);
    const bool idle_data_irq = idle.dut.irq_sci_rx &&
                               !idle.dut.irq_sci_rx_exception;
    const uint32_t idle_data = idle.read(A_STXL);

    // WAKE=1 uses R8 in 11-bit multidrop mode (WDS=6).  A data character
    // keeps the receiver asleep; the first address character clears RWU and
    // is itself transferred to SRX with R8 set.
    Harness address;
    address.write(A_PCC, 1u << 0);
    address.write(A_SCR, (1u << 11) | (1u << 8) | (1u << 6) |
                         (1u << 5) | 6u);
    address.rx(0x33, false);
    const uint32_t address_sleep_scr = address.read(A_SCR);
    const uint32_t address_sleep_status = address.read(A_SSR);
    const bool data_ignored = (address_sleep_scr & (1u << 6)) &&
                              !(address_sleep_status & 0x04) &&
                              !address.dut.irq_sci_rx;

    address.rx(0xa5, true);
    const uint32_t address_wake_scr = address.read(A_SCR);
    const uint32_t address_wake_status = address.read(A_SSR);
    const bool address_irq = address.dut.irq_sci_rx &&
                             !address.dut.irq_sci_rx_exception;
    const uint32_t address_data = address.read(A_STXL);

    // In the 10-bit asynchronous format (WDS=2), D7 is the address bit.
    Harness data_bit;
    data_bit.write(A_PCC, 1u << 0);
    data_bit.write(A_SCR, (1u << 11) | (1u << 8) | (1u << 6) |
                          (1u << 5) | 2u);
    data_bit.rx(0x7f);
    const bool low_data_ignored =
        (data_bit.read(A_SCR) & (1u << 6)) &&
        !(data_bit.read(A_SSR) & 0x04);
    data_bit.rx(0x80);
    const uint32_t data_bit_wake_scr = data_bit.read(A_SCR);
    const uint32_t data_bit_status = data_bit.read(A_SSR);
    const uint32_t data_bit_data = data_bit.read(A_STXL);

    const bool ok = idle_ignored && idle_woke &&
                    (idle_data_status & 0x04) && idle_data_irq &&
                    idle_data == 0x22 && data_ignored &&
                    !(address_wake_scr & (1u << 6)) &&
                    (address_wake_status & 0x84) == 0x84 && address_irq &&
                    address_data == 0xa5 && low_data_ignored &&
                    !(data_bit_wake_scr & (1u << 6)) &&
                    (data_bit_status & 0x04) && data_bit_data == 0x80;
    std::printf("%s SCI wakeup idle_ignore=%u idle_scr=%04x idle_ssr=%02x "
                "idle_data=%02x address_ignore=%u address_scr=%04x "
                "address_ssr=%02x address_data=%02x d7_ignore=%u "
                "d7_scr=%04x d7_ssr=%02x d7_data=%02x\n",
                ok ? "PASS" : "FAIL", idle_ignored,
                idle_wake_scr & 0xffff, idle_wake_status & 0xff,
                idle_data & 0xff, data_ignored,
                address_wake_scr & 0xffff, address_wake_status & 0xff,
                address_data & 0xff, low_data_ignored,
                data_bit_wake_scr & 0xffff, data_bit_status & 0xff,
                data_bit_data & 0xff);
    return ok;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const bool tx = tx_lifecycle();
    const bool rx = receive_lifecycle();
    const bool format_errors = receive_error_format_lifecycle();
    const bool event_clear = receive_event_clear_priority();
    const bool idle = idle_lifecycle();
    const bool wakeup = wakeup_lifecycle();
    return tx && rx && format_errors && event_clear && idle && wakeup ? 0 : 1;
}
