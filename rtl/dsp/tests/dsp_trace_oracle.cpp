// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

///////////////////////////////////////////////////////////////////////////////
// DSP56001 trace-mode exception boundary oracle
//
// SR.T requests the nonmaskable P:$0004 exception after each completed
// instruction.  Motorola inserts three instruction cycles (six oscillator
// clocks) to flush the pipeline.  Trace outranks SWI, is deferred across an
// entire REP package, and a long-vector JSR clears T while stacking the
// successor PC and original SR for RTI restoration.
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vdsp56001_core.h"
#include "verilated.h"

namespace {

constexpr uint16_t kTraceVector = 0x0004;
constexpr uint16_t kSwiVector = 0x0006;
constexpr uint16_t kIrqaVector = 0x0008;
constexpr uint16_t kHandler = 0x0060;
constexpr uint32_t kTraceSr = 0x002000; // SR.T (bit 13), IPL0.
constexpr uint32_t kReset = 0x000084;
constexpr uint32_t kResetSuccessor = 0x200013; // CLR A.
constexpr uint64_t kResetInitialA = 0x00123456789abcULL;
constexpr uint32_t kWait = 0x000086;
constexpr uint32_t kStop = 0x000087;

class TraceHarness {
public:
    Vdsp56001_core core;
    std::array<uint32_t, 256> p{};
    unsigned cycle = 0;
    unsigned first_trace_cycle = 0;
    unsigned quiet_cycles = 0;
    unsigned trace_first_fetches = 0;
    unsigned trace_second_fetches = 0;
    unsigned swi_first_fetches = 0;
    unsigned irqa_first_fetches = 0;
    unsigned handler_fetches = 0;
    unsigned successor_fetches = 0;
    unsigned wait_instruction_fetches = 0;
    uint16_t first_vector = 0xffff;
    bool watching_flush = false;
    bool flush_observed = false;
    bool handler_t_clear = false;
    bool handler_ipl3 = false;
    bool restored_t = false;
    bool first_handler_acc_a_captured = false;
    uint64_t first_handler_acc_a = 0;

    explicit TraceHarness(bool swi = false, bool repeated = false,
                          bool reset = false, bool wait = false,
                          bool stop = false) {
        if (repeated) {
            p[0x00] = 0x0603a0; // REP #3.
            p[0x01] = 0x375500; // MOVE #<$55,R7, one traced package.
            p[0x02] = 0x0af080; // Successor must wait for trace RTI.
            p[0x03] = 0x000002;
        } else if (reset) {
            p[0x00] = kReset;
            // If RESET is not traced at its own completion boundary, this
            // successor destroys A before the later trace becomes visible.
            p[0x01] = kResetSuccessor;
            p[0x02] = 0x0af080;
            p[0x03] = 0x000002;
        } else if (wait) {
            // First retire and trace an ordinary instruction, then return
            // directly to WAIT.  This separates WAIT's own trace boundary
            // from any trace taken while the direct-core harness starts.
            p[0x00] = 0x375500; // MOVE #<$55,R7.
            p[0x01] = kWait;
            p[0x02] = kResetSuccessor;
            p[0x03] = 0x0af080;
            p[0x04] = 0x000003;
        } else if (stop) {
            p[0x00] = kStop;
            p[0x01] = kResetSuccessor;
            p[0x02] = 0x0af080;
            p[0x03] = 0x000002;
        } else {
            p[0x00] = swi ? 0x000006 : 0x375500; // SWI or MOVE #<$55,R7.
            p[0x01] = 0x0af080; // Successor must wait for trace RTI.
            p[0x02] = 0x000001;
        }
        p[kTraceVector] = 0x0bf080; // JSR >$0060.
        p[kTraceVector + 1] = kHandler;
        p[kSwiVector] = 0x0bf080; // Competing SWI vector.
        p[kSwiVector + 1] = 0x0070;
        p[kHandler] = 0x000004; // RTI restores T and successor PC.
        p[0x70] = 0x000004;

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
        core.irq_priority = stop ? 0x0003 : 0;
        core.dbg_reg_wr_en = 0;
        core.dbg_reg_wr_addr = 0;
        core.dbg_reg_wr_data = 0;

        for (unsigned i = 0; i < 4; ++i)
            tick();
        core.rst_n = 1;
        core.dbg_reg_wr_en = 1;
        core.dbg_reg_wr_addr = 0x39;
        core.dbg_reg_wr_data = kTraceSr;
        tick();
        if (reset || wait || stop) {
            core.dbg_reg_wr_addr = 0x0e;
            core.dbg_reg_wr_data = kResetInitialA;
            tick();
        }
        if (stop) {
            core.dbg_reg_wr_addr = 0x3a;
            core.dbg_reg_wr_data = 0x000040; // OMR.SD: short recovery.
            tick();
        }
        core.dbg_reg_wr_en = 0;
        core.run = 1;
    }

    void tick() {
        core.clk = 0;
        core.eval();
        const bool fetch_en = core.p_fetch_en;
        const uint16_t fetch_addr = core.p_fetch_addr;

        const bool started_flush = !watching_flush && core.state_out == 4 &&
            (core.cur_inst_out == 0x375500 ||
             core.cur_inst_out == 0x000006 ||
             core.cur_inst_out == kReset ||
             core.cur_inst_out == kWait);
        if (started_flush) {
            watching_flush = true;
            first_trace_cycle = cycle;
        }
        if (watching_flush && !started_flush && !fetch_en)
            ++quiet_cycles;

        if (fetch_en) {
            if (fetch_addr == kTraceVector) {
                if (first_vector == 0xffff)
                    first_vector = fetch_addr;
                ++trace_first_fetches;
                flush_observed = true;
                watching_flush = false;
            } else if (fetch_addr == kTraceVector + 1) {
                ++trace_second_fetches;
            } else if (fetch_addr == kSwiVector) {
                if (first_vector == 0xffff)
                    first_vector = fetch_addr;
                ++swi_first_fetches;
            } else if (fetch_addr == kIrqaVector) {
                if (first_vector == 0xffff)
                    first_vector = fetch_addr;
                ++irqa_first_fetches;
            } else if (fetch_addr == kHandler) {
                ++handler_fetches;
                if (!first_handler_acc_a_captured) {
                    first_handler_acc_a_captured = true;
                    first_handler_acc_a = core.acc_a_out;
                }
            } else if (fetch_addr == 0x0001 || fetch_addr == 0x0002) {
                ++successor_fetches;
                if (fetch_addr == 0x0001)
                    ++wait_instruction_fetches;
                restored_t |= ((core.sr_out >> 13) & 1) != 0;
            }
            if (watching_flush && fetch_addr != kTraceVector) {
                quiet_cycles = 0;
                watching_flush = false;
            }
        }

        core.clk = 1;
        core.eval();
        if (fetch_en) {
            core.p_fetch_data = p[fetch_addr & 0xff] & 0x00ffffff;
            if (fetch_addr == kHandler) {
                handler_t_clear |= ((core.sr_out >> 13) & 1) == 0;
                handler_ipl3 |= ((core.sr_out >> 8) & 3) == 3;
            }
        }
        ++cycle;
    }

    void run(unsigned count) {
        for (unsigned i = 0; i < count; ++i)
            tick();
    }
};

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);

    TraceHarness ordinary;
    ordinary.run(180);
    const bool long_trace = ordinary.first_vector == kTraceVector &&
                            ordinary.trace_first_fetches >= 2 &&
                            ordinary.trace_first_fetches ==
                                ordinary.trace_second_fetches &&
                            ordinary.handler_fetches >= 2 &&
                            ordinary.handler_t_clear && ordinary.handler_ipl3 &&
                            ordinary.restored_t;
    const bool flush = ordinary.flush_observed && ordinary.quiet_cycles == 6;
    std::printf("%s trace long-vector/RTI: first=$%04x vector=%u/%u "
                "handler=%u T-clear=%u IPL3=%u restored-T=%u\n",
                long_trace ? "PASS" : "FAIL", ordinary.first_vector,
                ordinary.trace_first_fetches, ordinary.trace_second_fetches,
                ordinary.handler_fetches, ordinary.handler_t_clear,
                ordinary.handler_ipl3, ordinary.restored_t);
    std::printf("%s trace three-instruction flush: observed=%u quiet-clocks=%u "
                "required=6\n", flush ? "PASS" : "FAIL",
                ordinary.flush_observed,
                ordinary.quiet_cycles);

    TraceHarness swi(true, false);
    swi.run(140);
    const bool trace_over_swi = swi.first_vector == kTraceVector &&
                                swi.trace_first_fetches != 0 &&
                                swi.swi_first_fetches == 0;
    std::printf("%s coincident TRACE/SWI priority: first=$%04x trace=%u "
                "swi=%u\n", trace_over_swi ? "PASS" : "FAIL",
                swi.first_vector, swi.trace_first_fetches,
                swi.swi_first_fetches);

    TraceHarness repeated(false, true);
    repeated.run(180);
    const bool rep_deferred = repeated.first_vector == kTraceVector &&
                              repeated.trace_first_fetches != 0 &&
                              repeated.trace_first_fetches ==
                                  repeated.trace_second_fetches;
    std::printf("%s REP/TRACE package boundary: first=$%04x vector=%u/%u "
                "successor-fetch=%u\n", rep_deferred ? "PASS" : "FAIL",
                repeated.first_vector, repeated.trace_first_fetches,
                repeated.trace_second_fetches, repeated.successor_fetches);

    // RESET is a software peripheral/IPR reset, not a processor reset.
    // DSP56001UMA2 A-190 explicitly leaves trace enabled, and UM06 6.4.2.10
    // requires a trace exception after any instruction that began with T=1.
    // CLR A is placed immediately after RESET so a trace taken one instruction
    // late cannot masquerade as the required RESET completion boundary.
    TraceHarness reset(false, false, true);
    reset.run(220);
    const bool reset_traced = reset.first_vector == kTraceVector &&
                              reset.trace_first_fetches != 0 &&
                              reset.first_handler_acc_a_captured &&
                              reset.first_handler_acc_a == kResetInitialA;
    std::printf("%s RESET/TRACE completion boundary: first=$%04x "
                "trace=%u first-handler-A=%014llx/%014llx\n",
                reset_traced ? "PASS" : "FAIL", reset.first_vector,
                reset.trace_first_fetches,
                static_cast<unsigned long long>(reset.first_handler_acc_a),
                static_cast<unsigned long long>(kResetInitialA));

    // A trace request is the highest-priority ordinary exception and is
    // unmasked while WAIT executes.  UMA2 A-220 requires a pending unmasked
    // interrupt to be processed instead of entering the wait state; UM08
    // 8.2.1.3 says trace is pending while traced code executes.  As above,
    // the destructive successor distinguishes the WAIT boundary from a trace
    // incorrectly delayed until after the following instruction.
    TraceHarness wait(false, false, false, true);
    // Consume the known-good trace of the setup MOVE and wait until its RTI
    // refetches WAIT.  Observations below therefore describe WAIT itself,
    // not the preceding instruction or direct-core startup.
    for (unsigned i = 0; i < 180 && wait.handler_fetches == 0; ++i)
        wait.tick();
    wait.wait_instruction_fetches = 0;
    for (unsigned i = 0; i < 80 && wait.wait_instruction_fetches == 0; ++i)
        wait.tick();
    const bool wait_reached = wait.handler_fetches != 0 &&
                              wait.wait_instruction_fetches != 0;
    wait.first_vector = 0xffff;
    wait.trace_first_fetches = 0;
    wait.trace_second_fetches = 0;
    wait.irqa_first_fetches = 0;
    wait.handler_fetches = 0;
    wait.first_handler_acc_a_captured = false;
    wait.first_handler_acc_a = 0;
    wait.run(220);
    const bool wait_traced = wait_reached &&
                             wait.first_vector == kTraceVector &&
                             wait.trace_first_fetches != 0 &&
                             wait.first_handler_acc_a_captured &&
                             wait.first_handler_acc_a == kResetInitialA;
    std::printf("%s WAIT/TRACE pending-exception boundary: reached=%u "
                "state=%u first=$%04x "
                "trace=%u first-handler-A=%014llx/%014llx\n",
                wait_traced ? "PASS" : "FAIL", wait_reached,
                wait.core.state_out, wait.first_vector,
                wait.trace_first_fetches,
                static_cast<unsigned long long>(wait.first_handler_acc_a),
                static_cast<unsigned long long>(kResetInitialA));

    // UM06 6.4.2.10 generates trace after every instruction that begins with
    // T set.  UM08 8.5 explicitly preserves a trace pending at STOP entry.
    // STOP must therefore hold until IRQA restarts the clock, then service
    // trace ahead of the maskable IRQA request and before the destructive
    // successor executes.
    TraceHarness stop(false, false, false, false, true);
    stop.run(100);
    const unsigned stop_hold_state = stop.core.state_out;
    const unsigned stop_pre_wake_traces = stop.trace_first_fetches;
    const uint64_t stop_pre_wake_acc_a = stop.core.acc_a_out;
    const bool stop_held = stop_hold_state == 5 &&
                           stop.irqa_first_fetches == 0 &&
                           stop_pre_wake_acc_a == kResetInitialA;
    // Isolate wake ordering from the already-serviced bootstrap trace that
    // precedes the first fetched program word in this direct-core harness.
    stop.first_vector = 0xffff;
    stop.trace_first_fetches = 0;
    stop.trace_second_fetches = 0;
    stop.irqa_first_fetches = 0;
    stop.first_handler_acc_a_captured = false;
    stop.first_handler_acc_a = 0;
    stop.core.irq_a = 1;
    stop.run(160);
    stop.core.irq_a = 0;
    const bool stop_trace_preserved = stop_held &&
                                      stop.first_vector == kTraceVector &&
                                      stop.trace_first_fetches != 0 &&
                                      stop.first_handler_acc_a_captured &&
                                      stop.first_handler_acc_a ==
                                          kResetInitialA;
    std::printf("%s STOP/TRACE preserved-pending boundary: held=%u state=%u "
                "pre-trace=%u pre-A=%014llx "
                "first=$%04x trace=%u irqa=%u first-handler-A=%014llx/%014llx\n",
                stop_trace_preserved ? "PASS" : "FAIL", stop_held,
                stop_hold_state, stop_pre_wake_traces,
                static_cast<unsigned long long>(stop_pre_wake_acc_a),
                stop.first_vector, stop.trace_first_fetches,
                stop.irqa_first_fetches,
                static_cast<unsigned long long>(stop.first_handler_acc_a),
                static_cast<unsigned long long>(kResetInitialA));

    return long_trace && flush && trace_over_swi && rep_deferred &&
           reset_traced && wait_traced && stop_trace_preserved ? 0 : 1;
}
