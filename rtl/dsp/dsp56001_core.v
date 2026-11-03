// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

//////////////////////////////////////////////////////////////////////////////
// DSP56001 Core
//
// 2-clock pipeline with integrated register file, PC management, stack,
// loop control, and interrupt controller.
//
// Pipeline states:
//   S_WB_FETCH:    Writeback + present PC for next fetch
//   S_DECODE_EXEC: Latch instruction + decode + compute + present data addrs
//   S_MEM:         Wait for memory response (1 clock for block RAM)
//   S_LATCH_EXT:   Latch extension word + decode + present data addrs
//
// Timing:
//   1-word non-memory: 2 clocks (DECODE_EXEC → WB_FETCH)
//   1-word memory:     3 clocks (DECODE_EXEC → MEM → WB_FETCH)
//   2-word non-memory: 3 clocks (DECODE_EXEC → LATCH_EXT → WB_FETCH)
//   2-word memory:     4 clocks (DECODE_EXEC → LATCH_EXT → MEM → WB_FETCH)
//
// Instantiates: dsp56001_decode, dsp56001_alu, dsp56001_agu (all combinational)
//
// Register file:
//   - A, B: 56-bit accumulators
//   - X (X1:X0), Y (Y1:Y0): 24-bit data register halves
//   - R0-R7, N0-N7, M0-M7: 16-bit AGU registers
//   - SR, OMR, SP, LA, LC: control registers
//   - System stack: 15 entries x 48-bit (SSH:SSL)
//   - PC: 16-bit program counter
//////////////////////////////////////////////////////////////////////////////

module dsp56001_core (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clock_enable,      // Architectural DSP clock step

    // ========================================================================
    // Control
    // ========================================================================
    input  wire        halt,
    input  wire        run,               // Start execution after reset
    output wire        software_reset,    // RESET instruction peripheral pulse
    output wire        stop_reset,        // STOP individual-peripheral reset

    // ========================================================================
    // P-space shared-port readiness.  Fetch and MOVEM/MOVEP data requests
    // are mutually exclusive but observe the same physical acceptance input.
    // ========================================================================
    output reg  [15:0] p_fetch_addr,
    output reg         p_fetch_en,
    input  wire [23:0] p_fetch_data,
    input  wire        p_fetch_ready,

    // ========================================================================
    // X-space Data Access
    // ========================================================================
    output reg  [15:0] x_addr,
    output reg         x_rd_en,
    output reg         x_wr_en,
    output reg  [23:0] x_wdata,
    input  wire [23:0] x_rdata,
    input  wire        x_ready,

    // ========================================================================
    // Y-space Data Access
    // ========================================================================
    output reg  [15:0] y_addr,
    output reg         y_rd_en,
    output reg         y_wr_en,
    output reg  [23:0] y_wdata,
    input  wire [23:0] y_rdata,
    input  wire        y_ready,

    // ========================================================================
    // P-space Data Access (MOVEM)
    // ========================================================================
    output reg  [15:0] p_data_addr,
    output reg         p_data_rd_en,
    output reg         p_data_wr_en,
    output reg  [23:0] p_data_wdata,
    input  wire [23:0] p_data_rdata,

    // ========================================================================
    // Interrupt Inputs
    // ========================================================================
    input  wire        irq_a,
    input  wire        irq_b,
    input  wire        irq_ssi_tx,
    input  wire        irq_ssi_rx,
    input  wire        irq_ssi_tx_exception,
    input  wire        irq_ssi_rx_exception,
    input  wire        irq_sci_tx,
    input  wire        irq_sci_rx,
    input  wire        irq_sci_rx_exception,
    input  wire        irq_sci_idle,
    input  wire        irq_sci_timer,
    output wire        sci_idle_ack,
    output wire        sci_timer_ack,
    input  wire [15:0] irq_priority,

    // ========================================================================
    // Debug Interface
    // ========================================================================
    input  wire        dbg_reg_wr_en,
    input  wire [5:0]  dbg_reg_wr_addr,
    input  wire [55:0] dbg_reg_wr_data,
    output wire [15:0] pc_out,
    output wire [23:0] sr_out,
    output wire        data_rom_enable,
    output wire [3:0]  state_out,
    output wire        busy,
    output wire [23:0] cur_inst_out,     // Current instruction (debug)
    output wire [55:0] acc_a_out,
    output wire [55:0] acc_b_out
);

    // ========================================================================
    // State Machine
    // ========================================================================
    localparam [3:0] S_RESET       = 4'd0;
    localparam [3:0] S_IDLE        = 4'd1;
    localparam [3:0] S_DECODE_EXEC = 4'd2;  // Latch + decode + compute + present data addrs
    localparam [3:0] S_MEM         = 4'd3;  // Wait for memory data (1 clock)
    localparam [3:0] S_WB_FETCH    = 4'd4;  // Writeback + present next fetch addr
    localparam [3:0] S_HALT        = 4'd5;
    localparam [3:0] S_LATCH_EXT   = 4'd6;  // Latch ext word + decode + present data addrs
    localparam [3:0] S_P_WRITE     = 4'd7;  // Complete peripheral-to-P MOVEP before fetch
    localparam [3:0] S_IRQ_FETCH   = 4'd8;  // Fetch vector after a deferred REP boundary
    // Quiet clocks complete multi-cycle instruction timing without repeating
    // architectural work.  REP and RESET use the two REP states; MOVEP also
    // chains into the DO states where its P-memory/EA timing requires it.
    // after their ordinary decode/memory work without duplicating writeback.
    localparam [3:0] S_REP_WAIT1   = 4'd9;
    localparam [3:0] S_REP_WAIT2   = 4'd10;
    localparam [3:0] S_DO_WAIT1    = 4'd11; // DO setup is six clocks plus memory move
    localparam [3:0] S_DO_WAIT2    = 4'd12;
    localparam [3:0] S_DO_WAIT3    = 4'd13;
    // X and Y are independent buses architecturally, but CYAN's external
    // X/Y SRAM views share one inferred BRAM port.  Serialize the Y half of
    // L: moves and dual-external X:Y moves after the ordinary X response so
    // both transfers commit; independent-bus X:Y traffic remains concurrent.
    localparam [3:0] S_L_MEM_Y_REQ = 4'd14;
    localparam [3:0] S_L_MEM_Y_WB  = 4'd15;

    reg [3:0] state, next_state;
    // X/Y ready may be a one-fabric-clock response while this core advances
    // only on clock_enable.  Retain each accepted half of an issued transfer
    // so a response between architectural clocks cannot be lost and a
    // concurrent X/Y half is not repeated while the other half waits.
    reg x_req_outstanding;
    reg y_req_outstanding;
    reg x_req_accepted;
    reg y_req_accepted;
    assign state_out = state;
    assign busy = (state != S_IDLE) && (state != S_HALT) && (state != S_RESET);

    // ========================================================================
    // Register File
    // ========================================================================
    reg [55:0] acc_a, acc_b;
    reg [23:0] x0_reg, x1_reg, y0_reg, y1_reg;
    reg [15:0] r_reg [0:7];
    reg [15:0] n_reg [0:7];
    reg [15:0] m_reg [0:7];
    // DSP56001UMA3 A.8.6: an address register explicitly changed by a
    // move-type instruction is not available as a pointer to the immediately
    // following instruction.  Retain that register's pre-write value for one
    // AGU lookup.  Addressing-mode R updates are expressly excluded.
    localparam [1:0] ADDR_DELAY_R = 2'd0;
    localparam [1:0] ADDR_DELAY_N = 2'd1;
    localparam [1:0] ADDR_DELAY_M = 2'd2;
    reg        addr_delay_valid;
    reg [1:0]  addr_delay_kind;
    reg [2:0]  addr_delay_index;
    reg [15:0] addr_delay_value;

    function [15:0] visible_r;
        input [2:0] index;
        begin
            visible_r = r_reg[index];
            if (addr_delay_valid && addr_delay_kind == ADDR_DELAY_R &&
                addr_delay_index == index)
                visible_r = addr_delay_value;
        end
    endfunction

    function [15:0] visible_n;
        input [2:0] index;
        begin
            visible_n = n_reg[index];
            if (addr_delay_valid && addr_delay_kind == ADDR_DELAY_N &&
                addr_delay_index == index)
                visible_n = addr_delay_value;
        end
    endfunction

    function [15:0] visible_m;
        input [2:0] index;
        begin
            visible_m = m_reg[index];
            if (addr_delay_valid && addr_delay_kind == ADDR_DELAY_M &&
                addr_delay_index == index)
                visible_m = addr_delay_value;
        end
    endfunction
    reg [23:0] sr;    // Status register [7:0]=CCR, [15:8]=MR
    reg [7:0]  omr;   // Operating mode register
    reg [15:0] pc;    // Program counter (16-bit for DSP56001)
    reg [15:0] la;    // Loop address
    // LC is architecturally 16 bits.  Keep the zero-extended 24-bit backing
    // shape used by the system-stack datapath, but never expose or retain
    // payload bits 23:16.
    reg [23:0] lc;
    reg [5:0]  sp;    // Stack pointer (6-bit, 0-15)
    reg [47:0] ss [0:14];  // System stack (15 entries)
    // UF may be set directly while SE remains clear.  Motorola defines the
    // next pointer-changing stack operation from the low four pointer bits;
    // that operation replaces the stale UF state with its natural six-bit
    // result.  SE, in contrast, freezes implicit stack activity.
    wire stack_sp_operable = !sp[4];
    wire [5:0] stack_sp_base = {2'b00, sp[3:0]};
    reg        move_wr_en_c;
    reg [5:0]  move_wr_code_c;
    reg [23:0] move_wr_data_c;

    // DSP56001UM06 Figure 6-6: SR is 16 bits wide; bits 14, 12, and 7 are
    // reserved.  Use one architectural representation at every public,
    // stack, and move boundary.
    localparam [23:0] SR_ARCH_MASK = 24'h00af7f;
    wire [23:0] sr_arch = sr & SR_ARCH_MASK;

    // Debug outputs
    assign pc_out = pc;
    assign sr_out = sr_arch;
    assign data_rom_enable = omr[2];
    assign acc_a_out = acc_a;
    assign acc_b_out = acc_b;

    // ========================================================================
    // Instruction Registers
    // ========================================================================
    reg [23:0] cur_inst;      // Current instruction word
    reg [23:0] ext_word;      // Extension word
    // The external P/X/Y memories are synchronous.  Their registered result
    // is valid in S_MEM, but the memory decoder can select another source by
    // S_WB_FETCH.  Hold all three buses at the architectural response boundary
    // so writeback never consumes a later decoder selection.
    reg [23:0] p_data_hold;
    reg [23:0] x_data_hold;
    reg [23:0] y_data_hold;
    reg        fetching_ext;  // True when next S_DECODE_EXEC latches ext word
    reg        irq_vector_active;
    reg        irq_vector_second_slot;
    reg        irq_vector_discard_second;
    reg [15:0] irq_vector_base;
    reg [1:0]  irq_vector_level;
    reg        p_fetch_stalled;
    reg [15:0] p_fetch_addr_hold;
    reg        p_fetch_irq_hold;
    reg [15:0] p_fetch_irq_vector_hold;
    reg [1:0]  p_fetch_irq_level_hold;
    reg        irq_a_prev;
    reg        irq_b_prev;
    reg        irq_a_edge_pending;
    reg        irq_b_edge_pending;
    reg        rep_active;
    reg [15:0] rep_fetch_addr;
    reg [23:0] rep_saved_lc;
    reg        rep_boundary_pending;
    reg        rep_illegal_pending;
    reg        wait_active;
    reg [3:0]  wait_pending_steps;
    reg        stop_active;
    reg        stop_recovering;
    reg [16:0] stop_recovery_steps;
    reg        stop_trace_pending;
    reg        trace_flush_active;
    reg [2:0]  trace_flush_steps;
    reg [2:0]  swi_wait_steps;
    wire       dec_loop_is_rep;
    wire       dec_is_swi;
    wire       dec_is_illegal;
    wire       dec_is_tcc;
    wire       dec_tcc_dual;
    wire [2:0] dec_tcc_alu_src;
    wire       dec_tcc_alu_dest;
    wire [2:0] dec_tcc_r_src;
    wire [2:0] dec_tcc_r_dest;
    assign cur_inst_out = cur_inst;

    // IPR uses one three-bit field per external input: mode in the high bit
    // and disabled/IPL0/IPL1/IPL2 in the low two bits. The board host path
    // uses level-triggered IRQA (IPR=$3). Level code N is serviceable
    // precisely when it is greater than the current SR interrupt mask.
    wire [1:0] irq_a_level = irq_priority[1:0];
    wire [1:0] irq_b_level = irq_priority[4:3];
    wire irq_a_edge_mode = irq_priority[2];
    wire irq_b_edge_mode = irq_priority[5];
    wire irq_a_enabled = irq_a_level != 2'b00;
    wire irq_b_enabled = irq_b_level != 2'b00;
    wire irq_a_request = irq_a_edge_mode ? irq_a_edge_pending : irq_a;
    wire irq_b_request = irq_b_edge_mode ? irq_b_edge_pending : irq_b;
    wire irq_a_serviceable = irq_a_request && irq_a_enabled &&
                             (irq_a_level > sr[9:8]);
    wire irq_b_serviceable = irq_b_request && irq_b_enabled &&
                             (irq_b_level > sr[9:8]);
    wire [1:0] irq_ssi_level = irq_priority[13:12];
    wire irq_ssi_enabled = irq_ssi_level != 2'b00;
    wire irq_ssi_rx_serviceable = irq_ssi_rx && irq_ssi_enabled &&
                                  (irq_ssi_level > sr[9:8]);
    wire irq_ssi_tx_serviceable = irq_ssi_tx && irq_ssi_enabled &&
                                  (irq_ssi_level > sr[9:8]);
    wire [1:0] irq_sci_level = irq_priority[15:14];
    wire irq_sci_enabled = irq_sci_level != 2'b00;
    wire irq_sci_rx_exception_serviceable = irq_sci_rx_exception &&
                                             irq_sci_enabled &&
                                             (irq_sci_level > sr[9:8]);
    wire irq_sci_rx_serviceable = irq_sci_rx && irq_sci_enabled &&
                                  (irq_sci_level > sr[9:8]);
    wire irq_sci_tx_serviceable = irq_sci_tx && irq_sci_enabled &&
                                  (irq_sci_level > sr[9:8]);
    wire irq_sci_idle_serviceable = irq_sci_idle && irq_sci_enabled &&
                                    (irq_sci_level > sr[9:8]);
    wire irq_sci_timer_serviceable = irq_sci_timer && irq_sci_enabled &&
                                     (irq_sci_level > sr[9:8]);
    wire irq_sci_any_serviceable = irq_sci_rx_exception_serviceable ||
                                   irq_sci_rx_serviceable ||
                                   irq_sci_tx_serviceable ||
                                   irq_sci_idle_serviceable ||
                                   irq_sci_timer_serviceable;
    wire irq_any_serviceable = irq_a_serviceable || irq_b_serviceable ||
                               irq_ssi_rx_serviceable ||
                               irq_ssi_tx_serviceable ||
                               irq_sci_any_serviceable;
    // Select the highest unmasked IPL on every arbitration.  Motorola's fixed
    // within-level order is IRQA, IRQB, host, SSI RX exception, SSI RX,
    // SSI TX exception, SSI TX, SCI RX exception, SCI RX, SCI TX, SCI idle,
    // and SCI timer.  IP20 maps the external host event onto IRQA.
    wire irq_a_wins = irq_a_serviceable &&
                      (!irq_b_serviceable || (irq_a_level >= irq_b_level)) &&
                      (!irq_ssi_rx_serviceable ||
                       (irq_a_level >= irq_ssi_level)) &&
                      (!irq_ssi_tx_serviceable ||
                       (irq_a_level >= irq_ssi_level)) &&
                      (!irq_sci_any_serviceable ||
                       (irq_a_level >= irq_sci_level));
    wire irq_b_wins = !irq_a_wins && irq_b_serviceable &&
                      (!irq_ssi_rx_serviceable ||
                       (irq_b_level >= irq_ssi_level)) &&
                      (!irq_ssi_tx_serviceable ||
                       (irq_b_level >= irq_ssi_level)) &&
                      (!irq_sci_any_serviceable ||
                       (irq_b_level >= irq_sci_level));
    wire irq_ssi_rx_wins = !irq_a_wins && !irq_b_wins &&
                           irq_ssi_rx_serviceable &&
                           (!irq_sci_any_serviceable ||
                            (irq_ssi_level >= irq_sci_level));
    wire irq_ssi_tx_wins = !irq_a_wins && !irq_b_wins &&
                           !irq_ssi_rx_wins && irq_ssi_tx_serviceable &&
                           (!irq_sci_any_serviceable ||
                            (irq_ssi_level >= irq_sci_level));
    wire irq_sci_rx_exception_wins = !irq_a_wins && !irq_b_wins &&
                           !irq_ssi_rx_wins && !irq_ssi_tx_wins &&
                           irq_sci_rx_exception_serviceable;
    wire irq_sci_rx_wins = !irq_a_wins && !irq_b_wins &&
                           !irq_ssi_rx_wins && !irq_ssi_tx_wins &&
                           !irq_sci_rx_exception_wins &&
                           irq_sci_rx_serviceable;
    wire irq_sci_tx_wins = !irq_a_wins && !irq_b_wins &&
                           !irq_ssi_rx_wins && !irq_ssi_tx_wins &&
                           !irq_sci_rx_exception_wins && !irq_sci_rx_wins &&
                           irq_sci_tx_serviceable;
    wire irq_sci_idle_wins = !irq_a_wins && !irq_b_wins &&
                             !irq_ssi_rx_wins && !irq_ssi_tx_wins &&
                             !irq_sci_rx_exception_wins &&
                             !irq_sci_rx_wins && !irq_sci_tx_wins &&
                             irq_sci_idle_serviceable;
    wire irq_sci_timer_wins = !irq_a_wins && !irq_b_wins &&
                              !irq_ssi_rx_wins && !irq_ssi_tx_wins &&
                              !irq_sci_rx_exception_wins &&
                              !irq_sci_rx_wins && !irq_sci_tx_wins &&
                              !irq_sci_idle_wins &&
                              irq_sci_timer_serviceable;
    wire rep_final_iteration = rep_active && (lc[15:0] == 16'd1);
    // SWI is an instruction-boundary, nonmaskable IPL-3 interrupt.  Reuse the
    // architectural long-vector package machinery so the vector JSR stacks
    // the already-incremented (post-SWI) PC and RTI restores the saved SR.
    wire swi_accept = (state == S_WB_FETCH) && !irq_vector_active &&
                      dec_is_swi;
    // The fatal illegal-instruction exception is nonmaskable IPL 3.  The
    // ordinary fetch/decode path has already advanced PC at this boundary;
    // rewind it so a long-vector JSR stacks the offending instruction's
    // address, as the DSP56001 specifies.
    wire illegal_accept = (state == S_WB_FETCH) && !irq_vector_active &&
                          !rep_active && dec_is_illegal;
    // A repeated illegal instruction is a repeated NOP.  Its exception is
    // deferred until the REP package completes, after the successor has been
    // fetched but before that successor can decode or execute.
    wire rep_boundary_illegal_accept =
        (state == S_DECODE_EXEC) && rep_boundary_pending &&
        rep_illegal_pending && !irq_vector_active;
    wire irq_accept = (state == S_WB_FETCH) && !irq_vector_active &&
                      !dec_loop_is_rep && !rep_active &&
                      !rep_boundary_pending &&
                      !dec_is_stop && !dec_is_wait &&
                      irq_any_serviceable;
    wire wait_irq_accept = (state == S_HALT) && wait_active &&
                           (wait_pending_steps == 4'd0) &&
                           !irq_vector_active &&
                           irq_any_serviceable;
    // One architectural clock step is one DSP oscillator clock (two T
    // states).  OMR[6]=1 selects a 16T stabilization interval; OMR[6]=0
    // selects 131,072T.  Motorola then requires 17T before the first vector
    // or successor fetch, rounded up to the next oscillator clock here.
    wire [16:0] stop_recovery_period = omr[6] ? 17'd17 : 17'd65545;
    wire stop_recovery_complete = (state == S_HALT) && stop_active &&
                                  stop_recovering &&
                                  (stop_recovery_steps == 17'd1);
    wire stop_irq_accept = stop_recovery_complete &&
                           (irq_a_serviceable || irq_b_serviceable);
    // Motorola defines a REP package as uninterruptible and extends that
    // exclusion across immediately consecutive REP packages.  At the final
    // follower writeback the next instruction has already been fetched.  Use
    // that look-ahead only to distinguish another REP from an ordinary
    // successor; an interrupt diverts before the ordinary instruction has any
    // architectural effect.
    wire rep_boundary_irq_accept =
        (state == S_DECODE_EXEC) && rep_boundary_pending &&
        !irq_vector_active && !dec_loop_is_rep &&
        irq_any_serviceable;
    wire [15:0] irq_accept_vector = irq_a_wins ? 16'h0008 :
                                          irq_b_wins ? 16'h000a :
                                     irq_ssi_rx_wins ?
                                          (irq_ssi_rx_exception ?
                                           16'h000e : 16'h000c) :
                                     irq_ssi_tx_wins ?
                                          (irq_ssi_tx_exception ?
                                           16'h0012 : 16'h0010) :
                            irq_sci_rx_exception_wins ? 16'h0016 :
                                     irq_sci_rx_wins ? 16'h0014 :
                                     irq_sci_tx_wins ? 16'h0018 :
                                   irq_sci_idle_wins ? 16'h001a :
                                  irq_sci_timer_wins ? 16'h001c : 16'h0000;
    wire [1:0] irq_accept_level = irq_a_wins ? irq_a_level :
                                      irq_b_wins ? irq_b_level :
                                      (irq_ssi_rx_wins || irq_ssi_tx_wins) ?
                                                   irq_ssi_level :
                                                   irq_sci_level;
    // A denied P transaction is already externally visible and its address
    // cannot be replaced by later interrupt arbitration.  Retain an interrupt
    // that selected the original request as well, so a newly arriving higher
    // priority source cannot retarget that same bus transaction.
    wire irq_fetch_selected = !illegal_accept && !stack_error_accept &&
                              !swi_accept && irq_accept;
    wire wb_irq_accept = (p_fetch_stalled && p_fetch_irq_hold) ?
                         1'b1 : irq_accept;
    wire [15:0] wb_irq_vector =
        (p_fetch_stalled && p_fetch_irq_hold) ?
        p_fetch_irq_vector_hold : irq_accept_vector;
    wire [1:0] wb_irq_level =
        (p_fetch_stalled && p_fetch_irq_hold) ?
        p_fetch_irq_level_hold : irq_accept_level;
    wire irq_arrived_during_p_wait = p_fetch_stalled &&
                                     !p_fetch_irq_hold &&
                                     irq_fetch_selected;
    // STOP oscillator recovery is initiated only by IRQA.  Preserve the
    // existing external A/B arbitration at the recovery boundary; internal
    // peripheral requests cannot wake STOP.
    wire stop_irq_a_wins = irq_a_serviceable &&
                           (!irq_b_serviceable ||
                            (irq_a_level >= irq_b_level));
    wire [15:0] stop_accept_vector = stop_irq_a_wins ? 16'h0008 : 16'h000a;
    wire [1:0] stop_accept_level = stop_irq_a_wins ? irq_a_level : irq_b_level;
    // The interrupt controller always fetches both vector locations.  A
    // two-word instruction consumes the second location as its extension; a
    // fast package executes it as the second one-word instruction.  If the
    // first word changes program flow, the second location is still fetched
    // for interrupt completion but is discarded.  Motorola defines that
    // second-location fetch, not handler entry or RTI, as the edge-mode
    // acknowledgement and re-arm point.
    wire irq_vector_first_one_word_fetch =
        (state == S_WB_FETCH) && irq_vector_active &&
        !irq_vector_second_slot && !dec_is_two_word;
    wire irq_vector_second_fetch =
        ((state == S_DECODE_EXEC) && irq_vector_active &&
         !fetching_ext && dec_is_two_word) ||
        irq_vector_first_one_word_fetch;
    wire irq_a_edge_ack = irq_vector_second_fetch && p_fetch_ready &&
                          (irq_vector_base == 16'h0008);
    wire irq_b_edge_ack = irq_vector_second_fetch && p_fetch_ready &&
                          (irq_vector_base == 16'h000a);
    assign sci_timer_ack = irq_vector_second_fetch && p_fetch_ready &&
                           (irq_vector_base == 16'h001c);
    assign sci_idle_ack = irq_vector_second_fetch && p_fetch_ready &&
                          (irq_vector_base == 16'h001a);

    // The package pins are active-low; irq_a/irq_b are synchronized logical
    // assertions at this boundary, so their physical high-to-low edge appears
    // here as a zero-to-one transition.  Disabled edge inputs keep their latch
    // reset, while an enabled pending request survives SR priority masking.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            irq_a_prev <= 1'b0;
            irq_b_prev <= 1'b0;
            irq_a_edge_pending <= 1'b0;
            irq_b_edge_pending <= 1'b0;
        end else if (clock_enable) begin
            irq_a_prev <= irq_a;
            irq_b_prev <= irq_b;

            // STOP discards all pending external interrupt state and ignores
            // new edges during clock recovery.  A level IRQA still wakes the
            // oscillator and can be arbitrated at the end of the delay.
            if (stop_active || ((state == S_WB_FETCH) && dec_is_stop)) begin
                irq_a_edge_pending <= 1'b0;
                irq_b_edge_pending <= 1'b0;
            end else if (!irq_a_edge_mode || !irq_a_enabled)
                irq_a_edge_pending <= 1'b0;
            else begin
                if (irq_a_edge_ack)
                    irq_a_edge_pending <= 1'b0;
                if (irq_a && !irq_a_prev)
                    irq_a_edge_pending <= 1'b1;
            end

            if (stop_active || ((state == S_WB_FETCH) && dec_is_stop)) begin
                // Cleared together above.
            end else if (!irq_b_edge_mode || !irq_b_enabled)
                irq_b_edge_pending <= 1'b0;
            else begin
                if (irq_b_edge_ack)
                    irq_b_edge_pending <= 1'b0;
                if (irq_b && !irq_b_prev)
                    irq_b_edge_pending <= 1'b1;
            end
        end
    end

    // ========================================================================
    // Decoder Input Mux
    // ========================================================================
    // In S_DECODE_EXEC: decode directly from P-memory output (not yet latched)
    // In S_LATCH_EXT: decode cur_inst + P-memory output as ext_word
    // Otherwise: decode from latched cur_inst + ext_word
    wire [23:0] decode_inst = (state == S_DECODE_EXEC && !fetching_ext) ?
                               p_fetch_data : cur_inst;
    wire [23:0] decode_ext  = (state == S_LATCH_EXT) ? p_fetch_data : ext_word;

    // ========================================================================
    // Instruction Decoder Instance
    // ========================================================================
    wire        dec_is_two_word;
    wire        dec_is_parallel;
    wire        dec_is_nop;
    wire        dec_is_alu_op;
    wire        dec_is_move_op;
    wire        dec_is_branch_op;
    wire        dec_is_loop_op;
    wire        dec_is_bit_op;
    wire        dec_is_control_op;
    wire [5:0]  dec_alu_opcode;
    wire        dec_alu_dest;
    wire [2:0]  dec_alu_src_jjj;
    wire [2:0]  dec_alu_mpy_qq;
    wire        dec_alu_negate;
    wire        dec_alu_update_ccr;
    wire        dec_alu_no_writeback;
    wire [3:0]  dec_move_type;
    wire [5:0]  dec_move_src_reg;
    wire [5:0]  dec_move_dst_reg;
    wire        dec_move_src_mem;
    wire        dec_move_dst_mem;
    wire        dec_move_mem_space;
    wire [23:0] dec_move_imm_data;
    wire        dec_move_is_imm;
    wire        dec_move_addr_update;
    wire [2:0]  dec_addr_mode_mmm;
    wire [2:0]  dec_addr_reg_rrr;
    wire [5:0]  dec_addr_absolute;
    wire [3:0]  dec_branch_cond;
    wire        dec_branch_uncond;
    wire        dec_branch_push_pc;
    wire        dec_branch_use_ea;
    wire [23:0] dec_branch_target;
    wire        dec_loop_is_do;
    wire        dec_loop_is_enddo;
    wire [1:0]  dec_bit_op_type;
    wire [4:0]  dec_bit_op_bit;
    wire        dec_bit_op_mem;
    wire        dec_bit_op_space;
    wire [5:0]  dec_bit_op_addr;
    wire [1:0]  dec_bit_op_amode;
    wire        dec_is_jclr;
    wire        dec_is_jset;
    wire        dec_is_movec;
    wire        dec_is_movec_rr;
    wire        dec_is_movem;
    wire        dec_is_movep;
    wire        dec_is_andi_ori;
    wire        dec_andi_ori_is_and;
    wire [7:0]  dec_andi_ori_imm;
    wire [1:0]  dec_andi_ori_target;
    wire        dec_is_rts;
    wire        dec_is_rti;
    wire        dec_is_stop;
    wire        dec_is_wait;
    wire        dec_is_reset_inst;
    wire        dec_is_norm;
    wire [2:0]  dec_norm_rrr;
    wire        dec_is_lua;
    wire [2:0]  dec_lua_mmm;
    wire [2:0]  dec_lua_rrr;
    wire [4:0]  dec_lua_ddddd;

    dsp56001_decode decoder (
        .inst           (decode_inst),
        .ext_word       (decode_ext),
        .decode_en      (1'b1),
        .is_two_word    (dec_is_two_word),
        .is_parallel    (dec_is_parallel),
        .is_nop         (dec_is_nop),
        .is_alu_op      (dec_is_alu_op),
        .is_move_op     (dec_is_move_op),
        .is_branch_op   (dec_is_branch_op),
        .is_loop_op     (dec_is_loop_op),
        .is_bit_op      (dec_is_bit_op),
        .is_control_op  (dec_is_control_op),
        .alu_opcode     (dec_alu_opcode),
        .alu_dest       (dec_alu_dest),
        .alu_src_jjj    (dec_alu_src_jjj),
        .alu_mpy_qq     (dec_alu_mpy_qq),
        .alu_negate     (dec_alu_negate),
        .alu_update_ccr (dec_alu_update_ccr),
        .alu_no_writeback(dec_alu_no_writeback),
        .move_type      (dec_move_type),
        .move_src_reg   (dec_move_src_reg),
        .move_dst_reg   (dec_move_dst_reg),
        .move_src_mem   (dec_move_src_mem),
        .move_dst_mem   (dec_move_dst_mem),
        .move_mem_space (dec_move_mem_space),
        .move_imm_data  (dec_move_imm_data),
        .move_is_imm    (dec_move_is_imm),
        .move_addr_update(dec_move_addr_update),
        .addr_mode_mmm  (dec_addr_mode_mmm),
        .addr_reg_rrr   (dec_addr_reg_rrr),
        .addr_absolute  (dec_addr_absolute),
        .branch_cond    (dec_branch_cond),
        .branch_uncond  (dec_branch_uncond),
        .branch_push_pc (dec_branch_push_pc),
        .branch_use_ea  (dec_branch_use_ea),
        .branch_target  (dec_branch_target),
        .loop_is_do     (dec_loop_is_do),
        .loop_is_rep    (dec_loop_is_rep),
        .loop_is_enddo  (dec_loop_is_enddo),
        .bit_op_type    (dec_bit_op_type),
        .bit_op_bit     (dec_bit_op_bit),
        .bit_op_mem     (dec_bit_op_mem),
        .bit_op_space   (dec_bit_op_space),
        .bit_op_addr    (dec_bit_op_addr),
        .bit_op_amode   (dec_bit_op_amode),
        .is_jclr        (dec_is_jclr),
        .is_jset        (dec_is_jset),
        .is_movec       (dec_is_movec),
        .is_movec_rr    (dec_is_movec_rr),
        .is_movem       (dec_is_movem),
        .is_movep       (dec_is_movep),
        .is_andi_ori    (dec_is_andi_ori),
        .andi_ori_is_and(dec_andi_ori_is_and),
        .andi_ori_imm   (dec_andi_ori_imm),
        .andi_ori_target(dec_andi_ori_target),
        .is_rts         (dec_is_rts),
        .is_rti         (dec_is_rti),
        .is_stop        (dec_is_stop),
        .is_wait        (dec_is_wait),
        .is_swi         (dec_is_swi),
        .is_illegal     (dec_is_illegal),
        .is_tcc         (dec_is_tcc),
        .tcc_dual       (dec_tcc_dual),
        .tcc_alu_src    (dec_tcc_alu_src),
        .tcc_alu_dest   (dec_tcc_alu_dest),
        .tcc_r_src      (dec_tcc_r_src),
        .tcc_r_dest     (dec_tcc_r_dest),
        .is_reset_inst  (dec_is_reset_inst),
        .is_norm        (dec_is_norm),
        .norm_rrr       (dec_norm_rrr),
        .is_lua         (dec_is_lua),
        .lua_mmm        (dec_lua_mmm),
        .lua_rrr        (dec_lua_rrr),
        .lua_ddddd      (dec_lua_ddddd)
    );

    // RESET is an internal peripheral/IPR reset, not a processor reset.  The
    // pulse is synchronous to one architectural DSP clock step and is routed
    // only to the on-chip peripheral block by dsp56001_top.
    assign software_reset = clock_enable && !p_fetch_wait &&
                            (state == S_WB_FETCH) && dec_is_reset_inst;
    assign stop_reset = stop_active;

    // ========================================================================
    // ALU Instance (combinational)
    // ========================================================================
    wire [55:0] alu_result;
    wire        alu_result_valid;
    wire [7:0]  alu_ccr_out;
    wire        alu_ccr_valid;
    wire        alu_norm_shift_dir;
    wire        alu_norm_did_shift;

    dsp56001_alu alu_inst (
        .acc_a          (acc_a),
        .acc_b          (acc_b),
        .x0             (x0_reg),
        .x1             (x1_reg),
        .y0             (y0_reg),
        .y1             (y1_reg),
        .alu_opcode     (dec_alu_opcode),
        .dest_sel       (dec_alu_dest),
        .src_jjj        (dec_alu_src_jjj),
        .negate         (dec_alu_negate),
        .mpy_src_qq     (dec_alu_mpy_qq),
        .sr_in          (sr),
        .result         (alu_result),
        .result_valid   (alu_result_valid),
        .ccr_out        (alu_ccr_out),
        .ccr_valid      (alu_ccr_valid),
        .norm_shift_dir (alu_norm_shift_dir),
        .norm_did_shift (alu_norm_did_shift)
    );

    // ========================================================================
    // AGU Instance (combinational)
    // ========================================================================
    wire [15:0] agu_eff_addr;
    wire        agu_eff_addr_valid;
    wire [15:0] agu_r_out;
    wire        agu_r_wr_en;
    wire [2:0]  agu_r_wr_idx;

    dsp56001_agu agu_inst (
        .r_in           (visible_r(dec_addr_reg_rrr)),
        .n_in           (visible_n(dec_addr_reg_rrr)),
        .m_in           (visible_m(dec_addr_reg_rrr)),
        .r_out          (agu_r_out),
        .r_wr_en        (agu_r_wr_en),
        .r_wr_idx       (agu_r_wr_idx),
        .agu_rrr        (dec_addr_reg_rrr),
        .agu_mmm        (dec_addr_mode_mmm),
        .eff_addr       (agu_eff_addr),
        .eff_addr_valid (agu_eff_addr_valid)
    );

    // ========================================================================
    // Condition Code Evaluator (combinational)
    // ========================================================================
    reg cond_result;
    always @(*) begin
        case (dec_branch_cond)
            4'h0: cond_result = !sr[0];                      // CC: !C
            4'h1: cond_result = (sr[3] == sr[1]);             // GE: N==V
            4'h2: cond_result = !sr[2];                       // NE: !Z
            4'h3: cond_result = !sr[3];                       // PL: !N
            4'h4: cond_result = !sr[2] && !((!sr[4]) && (!sr[5])); // NN
            4'h5: cond_result = !sr[5];                       // EC: !E
            4'h6: cond_result = !sr[6];                       // LC: !L
            4'h7: cond_result = !sr[2] && (sr[3] == sr[1]);   // GT
            4'h8: cond_result = sr[0];                        // CS: C
            4'h9: cond_result = (sr[3] != sr[1]);             // LT: N!=V
            4'hA: cond_result = sr[2];                        // EQ: Z
            4'hB: cond_result = sr[3];                        // MI: N
            4'hC: cond_result = sr[2] || ((!sr[4]) && (!sr[5])); // NR
            4'hD: cond_result = sr[5];                        // ES: E
            4'hE: cond_result = sr[6];                        // LS: L
            4'hF: cond_result = sr[2] || (sr[3] != sr[1]);    // LE
        endcase
    end
    wire branch_taken = dec_branch_uncond || cond_result;
    // Tcc uses the internal Data ALU paths and therefore transfers full
    // accumulator width without the data-bus limiter.  A 24-bit X/Y source
    // occupies the signed A1/B1 position exactly as it does for TFR.
    reg [55:0] tcc_alu_source;
    always @(*) begin
        case (dec_tcc_alu_src)
            3'b000: tcc_alu_source = dec_tcc_alu_dest ? acc_a : acc_b;
            3'b100: tcc_alu_source = {{8{x0_reg[23]}}, x0_reg, 24'h0};
            3'b101: tcc_alu_source = {{8{y0_reg[23]}}, y0_reg, 24'h0};
            3'b110: tcc_alu_source = {{8{x1_reg[23]}}, x1_reg, 24'h0};
            3'b111: tcc_alu_source = {{8{y1_reg[23]}}, y1_reg, 24'h0};
            default: tcc_alu_source = 56'h0;
        endcase
    end
    // One-word JMP/JSR/Jcc/JScc effective-address forms select their target
    // through the AGU.  The decoder's branch_target remains the extension
    // word for the absolute two-word form; it may still contain an unrelated
    // predecessor's extension when branch_use_ea is asserted.
    wire [15:0] branch_effective_target = dec_branch_use_ea ?
                                                agu_eff_addr :
                                                dec_branch_target[15:0];

    // ========================================================================
    // Loop Status
    // ========================================================================
    wire loop_active = sr[15];  // LF bit in SR
    // LA identifies the final fetched program word.  At writeback PC already
    // points one word beyond that word, including when it is an extension.
    wire at_loop_end = loop_active && (pc - 16'd1 == la);
    wire loop_last = at_loop_end && (lc[15:0] == 16'd1);
    wire [15:0] loop_start_addr = ss[sp[3:0] - 4'd1][39:24];

    // A full accumulator transferred through a 24-bit data bus uses the
    // DSP56001 data shifter/limiter.  Return {limited,data} so every move
    // path, including ordinary X/Y/P memory moves, shares one interpretation.
    function [24:0] acc_word_bus;
        input [55:0] value;
        input [1:0] scaling;
        reg [23:0] data;
        reg limited;
        begin
            data = value[47:24];
            limited = 1'b0;
            case (scaling)
                2'b00: begin
                    data = value[47:24];
                    limited = (value[55:47] != 9'h000) &&
                              (value[55:47] != 9'h1ff);
                end
                2'b01: begin
                    data = value[48:25];
                    limited = (value[55:48] != 8'h00) &&
                              (value[55:48] != 8'hff);
                end
                2'b10: begin
                    data = value[46:23];
                    limited = (value[55:46] != 10'h000) &&
                              (value[55:46] != 10'h3ff);
                end
                default: begin
                    data = value[47:24];
                    limited = 1'b0;
                end
            endcase
            if (limited)
                data = value[55] ? 24'h800000 : 24'h7fffff;
            acc_word_bus = {limited, data};
        end
    endfunction

    wire [24:0] acc_a_bus_word = acc_word_bus(acc_a, sr[11:10]);
    wire [24:0] acc_b_bus_word = acc_word_bus(acc_b, sr[11:10]);

    // ========================================================================
    // Register Read by 6-bit Address
    // ========================================================================
    function [55:0] read_reg;
        input [5:0] addr;
        begin
            case (addr)
                6'h04: read_reg = {32'h0, x0_reg};
                6'h05: read_reg = {32'h0, x1_reg};
                6'h06: read_reg = {32'h0, y0_reg};
                6'h07: read_reg = {32'h0, y1_reg};
                6'h08: read_reg = {32'h0, acc_a[23:0]};      // A0
                6'h09: read_reg = {32'h0, acc_b[23:0]};      // B0
                6'h0A: read_reg = {48'h0, acc_a[55:48]};     // A2
                6'h0B: read_reg = {48'h0, acc_b[55:48]};     // B2
                6'h0C: read_reg = {32'h0, acc_a[47:24]};     // A1
                6'h0D: read_reg = {32'h0, acc_b[47:24]};     // B1
                6'h0E: read_reg = {32'h0, acc_a_bus_word[23:0]}; // A -> bus
                6'h0F: read_reg = {32'h0, acc_b_bus_word[23:0]}; // B -> bus
                6'h10: read_reg = {40'h0, r_reg[0]};
                6'h11: read_reg = {40'h0, r_reg[1]};
                6'h12: read_reg = {40'h0, r_reg[2]};
                6'h13: read_reg = {40'h0, r_reg[3]};
                6'h14: read_reg = {40'h0, r_reg[4]};
                6'h15: read_reg = {40'h0, r_reg[5]};
                6'h16: read_reg = {40'h0, r_reg[6]};
                6'h17: read_reg = {40'h0, r_reg[7]};
                6'h18: read_reg = {40'h0, n_reg[0]};
                6'h19: read_reg = {40'h0, n_reg[1]};
                6'h1A: read_reg = {40'h0, n_reg[2]};
                6'h1B: read_reg = {40'h0, n_reg[3]};
                6'h1C: read_reg = {40'h0, n_reg[4]};
                6'h1D: read_reg = {40'h0, n_reg[5]};
                6'h1E: read_reg = {40'h0, n_reg[6]};
                6'h1F: read_reg = {40'h0, n_reg[7]};
                6'h20: read_reg = {40'h0, m_reg[0]};
                6'h21: read_reg = {40'h0, m_reg[1]};
                6'h22: read_reg = {40'h0, m_reg[2]};
                6'h23: read_reg = {40'h0, m_reg[3]};
                6'h24: read_reg = {40'h0, m_reg[4]};
                6'h25: read_reg = {40'h0, m_reg[5]};
                6'h26: read_reg = {40'h0, m_reg[6]};
                6'h27: read_reg = {40'h0, m_reg[7]};
                6'h39: read_reg = {32'h0, sr_arch};
                6'h3A: read_reg = {48'h0, omr};
                6'h3B: read_reg = {50'h0, sp};
                // SS is stored zero-based: architectural stack location SP
                // resides at ss[SP-1] after a push.
                6'h3C: read_reg = stack_sp_operable ? {40'h0,
                    ss[(sp[3:0] == 4'd0) ? 4'd0 :
                       sp[3:0] - 4'd1][39:24]} : 56'h0;        // SSH
                6'h3D: read_reg = stack_sp_operable ? {40'h0,
                    ss[(sp[3:0] == 4'd0) ? 4'd0 :
                       sp[3:0] - 4'd1][15:0]} : 56'h0;         // SSL
                6'h3E: read_reg = {40'h0, la};
                6'h3F: read_reg = {40'h0, lc[15:0]};
                default: read_reg = 56'h0;
            endcase
        end
    endfunction

    // The A/B L: operands transfer one signed 48-bit quantity.  Scaling is
    // applied across both halves; saturation produces the architectural
    // 7FFFFF:FFFFFF or 800000:000000 endpoint.
    function [48:0] acc_long_bus;
        input [55:0] value;
        input [1:0] scaling;
        reg [47:0] data;
        reg limited;
        begin
            data = value[47:0];
            limited = 1'b0;
            case (scaling)
                2'b00: begin
                    data = value[47:0];
                    limited = (value[55:47] != 9'h000) &&
                              (value[55:47] != 9'h1ff);
                end
                2'b01: begin
                    data = value[48:1];
                    limited = (value[55:48] != 8'h00) &&
                              (value[55:48] != 8'hff);
                end
                2'b10: begin
                    data = {value[46:0], 1'b0};
                    limited = (value[55:46] != 10'h000) &&
                              (value[55:46] != 10'h3ff);
                end
                default: begin
                    data = value[47:0];
                    limited = 1'b0;
                end
            endcase
            if (limited)
                data = value[55] ? 48'h800000_000000 :
                                   48'h7fffff_ffffff;
            acc_long_bus = {limited, data};
        end
    endfunction

    // ========================================================================
    // Instruction needs memory access (determines S_MEM vs direct S_WB_FETCH)
    // ========================================================================
    // DSP56000 Family Manual MOVEP formats are selected by bits 7:6:
    //   00 = register reference, 01 = P-memory reference,
    //   1S = X/Y-memory reference (S selects the effective-address space).
    wire movep_reg_ref = dec_is_movep && (decode_inst[7:6] == 2'b00);
    wire movep_p_ref   = dec_is_movep && (decode_inst[7:6] == 2'b01);
    wire movep_xy_ref  = dec_is_movep && decode_inst[7];
    wire movep_xy_immediate = movep_xy_ref && decode_inst[15] &&
                              (decode_inst[13:8] == 6'b110100);
    // MOVEC 0x05 forms select a five-bit control register in bits 4:0.
    // Bit 14 distinguishes effective-address from six-bit short-absolute
    // memory forms; effective address 110100 is long immediate data.
    wire movec_short_immediate = dec_is_movec && !dec_is_movec_rr &&
                                 (decode_inst[7:5] == 3'b101);
    wire movec_long_immediate = dec_is_movec && !dec_is_movec_rr &&
                                !movec_short_immediate &&
                                decode_inst[15] && decode_inst[14] &&
                                (decode_inst[13:8] == 6'b110100);
    wire movec_mem_ref = dec_is_movec && !dec_is_movec_rr &&
                         !movec_short_immediate && !movec_long_immediate;
    wire [15:0] movec_mem_addr = !decode_inst[14] ?
                                 {10'h0, decode_inst[13:8]} :
                                 (decode_inst[13:11] == 3'b110) ?
                                 decode_ext[15:0] : agu_eff_addr;
    wire [55:0] movep_reg_value = read_reg(decode_inst[13:8]);
    wire movep_reg_full_acc_source = movep_reg_ref && decode_inst[15] &&
                                     (decode_inst[13:8] == 6'h0e ||
                                      decode_inst[13:8] == 6'h0f);
    wire [55:0] movep_acc_source = decode_inst[8] ? acc_b : acc_a;
    reg [23:0] movep_acc_data;
    reg        movep_acc_limit;

    // A/B (as distinct from A0/A1/A2/B0/B1/B2) pass through the
    // DSP56001 data shifter/limiter when read onto a 24-bit data bus.
    // SR[11:10] selects no scaling, arithmetic scale-down, or scale-up.
    // The reserved 2'b11 mode is deterministic here but architecturally
    // undefined and is not an admitted operating mode.
    always @(*) begin
        movep_acc_data  = movep_acc_source[47:24];
        movep_acc_limit = 1'b0;
        case (sr[11:10])
            2'b00: begin
                movep_acc_data  = movep_acc_source[47:24];
                movep_acc_limit = (movep_acc_source[55:47] != 9'h000) &&
                                  (movep_acc_source[55:47] != 9'h1ff);
            end
            2'b01: begin
                movep_acc_data  = movep_acc_source[48:25];
                movep_acc_limit = (movep_acc_source[55:48] != 8'h00) &&
                                  (movep_acc_source[55:48] != 8'hff);
            end
            2'b10: begin
                movep_acc_data  = movep_acc_source[46:23];
                movep_acc_limit = (movep_acc_source[55:46] != 10'h000) &&
                                  (movep_acc_source[55:46] != 10'h3ff);
            end
            default: begin
                movep_acc_data  = movep_acc_source[47:24];
                movep_acc_limit = 1'b0;
            end
        endcase
        if (movep_acc_limit)
            movep_acc_data = movep_acc_source[55] ? 24'h800000 :
                                                   24'h7fffff;
    end

    wire [23:0] movep_reg_write_data = movep_reg_full_acc_source ?
                                       movep_acc_data :
                                       movep_reg_value[23:0];

    // L: long-memory moves use both X and Y memory at one address.  The LLL
    // field names either a literal pair or one/two limited accumulators.
    wire dec_is_l_move = dec_is_parallel && dec_move_type == 4'd6;
    wire dec_is_xy_move = dec_is_parallel && dec_move_type == 4'd5;
    wire dec_is_xr_ry_class1 = dec_is_parallel && dec_move_type == 4'd9;
    wire dec_is_xr_ry_class2 = dec_is_parallel && dec_move_type == 4'd10;
    wire dec_is_class1_immediate = dec_is_xr_ry_class1 && decode_inst[15] &&
                                   decode_inst[13:8] == 6'b110100;
    wire [2:0] l_move_lll = {decode_inst[19], decode_inst[17:16]};
    wire l_move_load = decode_inst[15];
    wire [15:0] l_move_addr = !decode_inst[14] ?
                              {10'h0, decode_inst[13:8]} :
                              (decode_inst[13:11] == 3'b110) ?
                              decode_ext[15:0] : agu_eff_addr;
    wire [24:0] l_acc_a_word = acc_word_bus(acc_a, sr[11:10]);
    wire [24:0] l_acc_b_word = acc_word_bus(acc_b, sr[11:10]);
    wire [48:0] l_acc_a_long = acc_long_bus(acc_a, sr[11:10]);
    wire [48:0] l_acc_b_long = acc_long_bus(acc_b, sr[11:10]);
    reg [23:0] l_move_x_source;
    reg [23:0] l_move_y_source;
    reg        l_move_limit;

    // X:Y dual-memory moves encode two independent restricted address modes.
    // The X side names either R0--R3 or R4--R7 directly; the Y side names the
    // opposite bank through a two-bit subindex.  Convert the restricted
    // (Rn)/(Rn)+Nn/(Rn)-/(Rn)+ codes to the ordinary AGU encoding so both
    // sides retain linear, modulo and bit-reverse behavior.
    function [2:0] xy_mode_to_mmm;
        input [1:0] mode;
        begin
            case (mode)
                2'b00: xy_mode_to_mmm = 3'b100; // (Rn)
                2'b01: xy_mode_to_mmm = 3'b001; // (Rn)+Nn
                2'b10: xy_mode_to_mmm = 3'b010; // (Rn)-
                default: xy_mode_to_mmm = 3'b011; // (Rn)+
            endcase
        end
    endfunction

    wire [2:0] xy_x_rrr = decode_inst[10:8];
    wire [2:0] xy_y_rrr = {~decode_inst[10], decode_inst[14:13]};
    wire [2:0] xy_x_mmm = xy_mode_to_mmm(decode_inst[12:11]);
    wire [2:0] xy_y_mmm = xy_mode_to_mmm(decode_inst[21:20]);
    wire [5:0] xy_x_reg = (decode_inst[19:18] == 2'b00) ? 6'h04 :
                          (decode_inst[19:18] == 2'b01) ? 6'h05 :
                          (decode_inst[19:18] == 2'b10) ? 6'h0e : 6'h0f;
    wire [5:0] xy_y_reg = (decode_inst[17:16] == 2'b00) ? 6'h06 :
                          (decode_inst[17:16] == 2'b01) ? 6'h07 :
                          (decode_inst[17:16] == 2'b10) ? 6'h0e : 6'h0f;
    wire [15:0] xy_x_eff_addr;
    wire [15:0] xy_y_eff_addr;
    wire [15:0] xy_x_r_out;
    wire [15:0] xy_y_r_out;
    wire        xy_x_r_wr_en;
    wire        xy_y_r_wr_en;
    wire [2:0]  xy_x_r_wr_idx;
    wire [2:0]  xy_y_r_wr_idx;
    // Mirror the memory router's X/Y external decode.  The $0100-$01ff ROM
    // window becomes external when OMR.DE disables the data ROMs.
    wire xy_x_external = (xy_x_eff_addr >= 16'h0200 ||
                          (!data_rom_enable &&
                           xy_x_eff_addr >= 16'h0100)) &&
                         xy_x_eff_addr < 16'hffc0;
    wire xy_y_external = (xy_y_eff_addr >= 16'h0200 ||
                          (!data_rom_enable &&
                           xy_y_eff_addr >= 16'h0100)) &&
                         xy_y_eff_addr < 16'hffc0;
    wire xy_both_external = xy_x_external && xy_y_external;
    wire l_move_external = (l_move_addr >= 16'h0200 ||
                            (!data_rom_enable && l_move_addr >= 16'h0100)) &&
                           l_move_addr < 16'hffc0;

    dsp56001_agu xy_x_agu_inst (
        .r_in           (visible_r(xy_x_rrr)),
        .n_in           (visible_n(xy_x_rrr)),
        .m_in           (visible_m(xy_x_rrr)),
        .r_out          (xy_x_r_out),
        .r_wr_en        (xy_x_r_wr_en),
        .r_wr_idx       (xy_x_r_wr_idx),
        .agu_rrr        (xy_x_rrr),
        .agu_mmm        (xy_x_mmm),
        .eff_addr       (xy_x_eff_addr),
        .eff_addr_valid ()
    );

    dsp56001_agu xy_y_agu_inst (
        .r_in           (visible_r(xy_y_rrr)),
        .n_in           (visible_n(xy_y_rrr)),
        .m_in           (visible_m(xy_y_rrr)),
        .r_out          (xy_y_r_out),
        .r_wr_en        (xy_y_r_wr_en),
        .r_wr_idx       (xy_y_r_wr_idx),
        .agu_rrr        (xy_y_rrr),
        .agu_mmm        (xy_y_mmm),
        .eff_addr       (xy_y_eff_addr),
        .eff_addr_valid ()
    );

    always @(*) begin
        l_move_x_source = 24'h0;
        l_move_y_source = 24'h0;
        l_move_limit = 1'b0;
        case (l_move_lll)
            3'b000: begin // A10 = A1:A0, no limiting
                l_move_x_source = acc_a[47:24];
                l_move_y_source = acc_a[23:0];
            end
            3'b001: begin // B10 = B1:B0, no limiting
                l_move_x_source = acc_b[47:24];
                l_move_y_source = acc_b[23:0];
            end
            3'b010: begin // X = X1:X0
                l_move_x_source = x1_reg;
                l_move_y_source = x0_reg;
            end
            3'b011: begin // Y = Y1:Y0
                l_move_x_source = y1_reg;
                l_move_y_source = y0_reg;
            end
            3'b100: begin // A, one limited 48-bit quantity
                l_move_x_source = l_acc_a_long[47:24];
                l_move_y_source = l_acc_a_long[23:0];
                l_move_limit = l_acc_a_long[48];
            end
            3'b101: begin // B, one limited 48-bit quantity
                l_move_x_source = l_acc_b_long[47:24];
                l_move_y_source = l_acc_b_long[23:0];
                l_move_limit = l_acc_b_long[48];
            end
            3'b110: begin // AB, independently limited A then B
                l_move_x_source = l_acc_a_word[23:0];
                l_move_y_source = l_acc_b_word[23:0];
                l_move_limit = l_acc_a_word[24] | l_acc_b_word[24];
            end
            3'b111: begin // BA, independently limited B then A
                l_move_x_source = l_acc_b_word[23:0];
                l_move_y_source = l_acc_a_word[23:0];
                l_move_limit = l_acc_a_word[24] | l_acc_b_word[24];
            end
        endcase
    end

    // REP source formats from the model-specific DSP56001 encoding.  The
    // low-byte 0xA? immediate form takes precedence because its count bits
    // may otherwise resemble the register format in bits 15:14.
    wire rep_imm_form = dec_loop_is_rep && decode_inst[7:4] == 4'ha;
    wire rep_reg_form = dec_loop_is_rep && !rep_imm_form &&
                        decode_inst[15:14] == 2'b11;
    wire rep_ea_form = dec_loop_is_rep && !rep_imm_form &&
                       decode_inst[15:14] == 2'b01;
    wire rep_short_form = dec_loop_is_rep && !rep_imm_form &&
                          decode_inst[15:14] == 2'b00;
    wire rep_mem_form = rep_ea_form || rep_short_form;
    wire rep_mem_space = decode_inst[6];
    wire rep_slow_ea = rep_ea_form &&
                       (dec_addr_mode_mmm == 3'b101 ||
                        dec_addr_mode_mmm == 3'b111);
    wire rep_fast_mem = rep_mem_form && !rep_slow_ea;
    wire [55:0] rep_reg_value = read_reg(decode_inst[13:8]);
    wire rep_reg_full_acc_source = rep_reg_form &&
                                   (decode_inst[13:8] == 6'h0e ||
                                    decode_inst[13:8] == 6'h0f);
    reg [15:0] rep_source_count;

    always @(*) begin
        if (rep_imm_form)
            rep_source_count = {4'h0, decode_inst[3:0], decode_inst[15:8]};
        else if (rep_reg_form)
            rep_source_count = rep_reg_full_acc_source ?
                               movep_acc_data[15:0] :
                               rep_reg_value[15:0];
        else if (rep_mem_form)
            rep_source_count = rep_mem_space ?
                               (rep_fast_mem ? y_rdata[15:0] :
                                               y_data_hold[15:0]) :
                               (rep_fast_mem ? x_rdata[15:0] :
                                               x_data_hold[15:0]);
        else
            rep_source_count = 16'h0;
    end

    // DO uses the adjacent low-byte encodings and the same four count-source
    // classes as REP.  Its first stack push precedes architectural sampling:
    // DO SP sees old SP+1, and DO SSL sees the LC just saved by that push.
    wire do_imm_form = dec_loop_is_do && decode_inst[7:4] == 4'h8;
    wire do_reg_form = dec_loop_is_do && !do_imm_form &&
                       decode_inst[15:14] == 2'b11;
    wire do_ea_form = dec_loop_is_do && !do_imm_form &&
                      decode_inst[15:14] == 2'b01;
    wire do_short_form = dec_loop_is_do && !do_imm_form &&
                         decode_inst[15:14] == 2'b00;
    wire do_mem_form = do_ea_form || do_short_form;
    wire do_mem_space = decode_inst[6];
    wire do_slow_ea = do_ea_form &&
                      (dec_addr_mode_mmm == 3'b101 ||
                       dec_addr_mode_mmm == 3'b111);
    wire do_fast_mem = do_mem_form && !do_slow_ea;
    wire [55:0] do_reg_value = read_reg(decode_inst[13:8]);
    wire do_reg_full_acc_source = do_reg_form &&
                                  (decode_inst[13:8] == 6'h0e ||
                                   decode_inst[13:8] == 6'h0f);
    reg [15:0] do_source_count;

    always @(*) begin
        if (do_imm_form)
            do_source_count = {4'h0, decode_inst[3:0], decode_inst[15:8]};
        else if (do_reg_form && decode_inst[13:8] == 6'h3b)
            do_source_count = {10'h0, sp} + 16'd1;
        else if (do_reg_form && decode_inst[13:8] == 6'h3d)
            do_source_count = lc[15:0];
        else if (do_reg_form)
            do_source_count = do_reg_full_acc_source ?
                              movep_acc_data[15:0] :
                              do_reg_value[15:0];
        else if (do_mem_form)
            do_source_count = do_mem_space ?
                              (do_fast_mem ? y_rdata[15:0] :
                                             y_data_hold[15:0]) :
                              (do_fast_mem ? x_rdata[15:0] :
                                             x_data_hold[15:0]);
        else
            do_source_count = 16'h0;
    end

    wire needs_xy_mem = dec_is_l_move || dec_is_xy_move || movec_mem_ref ||
                        (dec_move_src_mem || dec_move_dst_mem) ||
                        movep_reg_ref || movep_xy_ref ||
                        (dec_is_bit_op && dec_bit_op_mem) || rep_mem_form ||
                        do_mem_form;
    wire needs_p_data = dec_is_movem || movep_p_ref;
    wire needs_mem = needs_xy_mem || needs_p_data;
    wire xy_issue_state = state == S_DECODE_EXEC ||
                          state == S_LATCH_EXT ||
                          state == S_L_MEM_Y_REQ;
    wire x_req_complete = x_req_accepted;
    wire y_req_complete = y_req_accepted;
    wire y_response_complete = y_req_accepted ||
                               (y_req_outstanding && y_ready);
    wire p_fetch_wait = clock_enable && p_fetch_en && !p_fetch_ready;
    wire p_data_issue_state = state == S_DECODE_EXEC ||
                              state == S_LATCH_EXT ||
                              state == S_P_WRITE;
    wire p_data_issue_wait = clock_enable && p_data_issue_state &&
                             (p_data_rd_en || p_data_wr_en) &&
                             !p_fetch_ready;
    wire xy_issue_wait = clock_enable && xy_issue_state &&
                         (((x_rd_en || x_wr_en) && !x_ready) ||
                          ((y_rd_en || y_wr_en) && !y_ready));
    wire data_issue_wait = xy_issue_wait || p_data_issue_wait;
    // Modifying bit operations and MOVEP memory-reference forms can issue
    // their X/Y write only at the later architectural writeback boundary.
    // An independently ready P fetch must not retire that instruction and
    // discard a denied data write.
    wire xy_writeback_wait = clock_enable && state == S_WB_FETCH &&
                             ((x_wr_en && !x_ready) ||
                              (y_wr_en && !y_ready));
    // UMA3 Tables A-6/A-7/A-13/A-14: parallel data moves share the
    // two-clock base instruction.  Ordinary zero-wait X/Y traffic completes
    // at writeback directly from the registered memory response; indexed and
    // predecrement modes add two clocks.  L: and X:Y serialize only when both
    // architectural buses reach CYAN's one external SRAM port.
    wire parallel_single_mem = dec_is_parallel && !dec_is_l_move &&
                               !dec_is_xy_move &&
                               (dec_move_src_mem || dec_move_dst_mem);
    wire parallel_single_ea = dec_is_xr_ry_class1 ||
                              dec_is_xr_ry_class2 || decode_inst[14];
    wire parallel_slow_ea = parallel_single_mem && parallel_single_ea &&
                            (dec_addr_mode_mmm == 3'b101 ||
                             dec_addr_mode_mmm == 3'b111);
    wire parallel_fast_mem = parallel_single_mem && !dec_is_two_word &&
                             !parallel_slow_ea;
    wire l_move_slow_ea = dec_is_l_move && decode_inst[14] &&
                          (dec_addr_mode_mmm == 3'b101 ||
                           dec_addr_mode_mmm == 3'b111);
    wire l_move_fast_internal = dec_is_l_move && !l_move_external &&
                                !dec_is_two_word && !l_move_slow_ea;
    wire xy_move_fast = dec_is_xy_move && !xy_both_external;
    // UMA3 A-231: indexed-by-Nn and predecrement effective addresses add two
    // clocks.  Absolute addressing is two-word and carries the same increment.
    wire movep_slow_ea = dec_is_movep &&
                         (dec_addr_mode_mmm == 3'b101 ||
                          dec_addr_mode_mmm == 3'b111);
    // UMA3 Tables A-6/A-13: MOVEM is six clocks plus effective-address
    // and P-memory penalties.  Indexed and predecrement modes add two clocks;
    // absolute is handled separately as a two-word instruction below.
    wire movem_slow_ea = dec_is_movem && decode_inst[14] &&
                         (dec_addr_mode_mmm == 3'b101 ||
                          dec_addr_mode_mmm == 3'b111);
    // MOVEC's two-clock ordinary memory forms issue their SRAM request in
    // decode and consume the registered response in writeback.  Indexed and
    // predecrement effective addresses carry the manual-defined two-clock EA
    // penalty; absolute and long-immediate forms are two-word instructions.
    wire movec_slow_ea = movec_mem_ref && decode_inst[14] &&
                         (dec_addr_mode_mmm == 3'b101 ||
                          dec_addr_mode_mmm == 3'b111);
    wire movec_fast_mem = movec_mem_ref && !dec_is_two_word &&
                          !movec_slow_ea;
    wire bit_data_op = dec_is_bit_op && !dec_is_jclr && !dec_is_jset;
    wire bit_slow_ea = bit_data_op && dec_bit_op_mem &&
                       dec_bit_op_amode == 2'b01 &&
                       (dec_bit_op_addr[5:3] == 3'b101 ||
                        dec_bit_op_addr[5:3] == 3'b111);
    // UMA3 Tables A-6/A-10/A-11/A-13: all ordinary jump/call forms occupy
    // four clocks plus the EA term; bit-conditioned jump/call forms occupy
    // six.  Indexed and predecrement addressing add two clocks.  Jbit uses
    // the bit-operation EA field for its source while every form consumes a
    // separate extension word for the program target.
    wire jump_slow_ea = dec_is_branch_op && dec_branch_use_ea &&
                        (dec_addr_mode_mmm == 3'b101 ||
                         dec_addr_mode_mmm == 3'b111);
    wire jbit_flow = dec_is_bit_op && (dec_is_jclr || dec_is_jset);
    wire jbit_slow_ea = jbit_flow && dec_bit_op_mem &&
                        dec_bit_op_amode == 2'b01 &&
                        (dec_bit_op_addr[5:3] == 3'b101 ||
                         dec_bit_op_addr[5:3] == 3'b111);

    // JCLR/JSET and JSCLR/JSSET test the selected source bit without using or
    // changing CCR.C.  Their JS forms use branch_push_pc only to request the
    // conditional stack operation.
    wire [55:0] j_bit_reg_raw = read_reg(dec_bit_op_addr);
    wire [23:0] j_bit_reg_value =
        (dec_bit_op_addr == 6'h0a) ? {{16{acc_a[55]}}, acc_a[55:48]} :
        (dec_bit_op_addr == 6'h0b) ? {{16{acc_b[55]}}, acc_b[55:48]} :
        (dec_bit_op_addr == 6'h0e) ? acc_a[23:0] :
        (dec_bit_op_addr == 6'h0f) ? acc_b[23:0] :
                                     j_bit_reg_raw[23:0];
    reg [23:0] b_bit_reg_value;
    always @(*) begin
        case (dec_bit_op_addr)
            6'h08: b_bit_reg_value = acc_a[23:0];
            6'h09: b_bit_reg_value = acc_b[23:0];
            6'h0a: b_bit_reg_value = {{16{acc_a[55]}}, acc_a[55:48]};
            6'h0b: b_bit_reg_value = {{16{acc_b[55]}}, acc_b[55:48]};
            6'h0c: b_bit_reg_value = acc_a[47:24];
            6'h0d: b_bit_reg_value = acc_b[47:24];
            // B-operation bit numbers are 0..23, so full A/B address the
            // low accumulator piece.  Data-move scaling/limiting does not
            // apply to this read-modify-write instruction family.
            6'h0e: b_bit_reg_value = acc_a[23:0];
            6'h0f: b_bit_reg_value = acc_b[23:0];
            6'h29: b_bit_reg_value = sr_arch;
            6'h2a: b_bit_reg_value = {16'h0, omr};
            6'h2b: b_bit_reg_value = {18'h0, sp};
            6'h2c: b_bit_reg_value = stack_sp_operable ? {8'h0,
                ss[(sp[3:0] == 4'd0) ? 4'd0 :
                   sp[3:0] - 4'd1][39:24]} : 24'h0;
            6'h2d: b_bit_reg_value = stack_sp_operable ? {8'h0,
                ss[(sp[3:0] == 4'd0) ? 4'd0 :
                   sp[3:0] - 4'd1][15:0]} : 24'h0;
            6'h2e: b_bit_reg_value = {8'h0, la};
            6'h2f: b_bit_reg_value = {8'h0, lc[15:0]};
            default: b_bit_reg_value = j_bit_reg_value;
        endcase
    end
    wire jclr_bit_val = dec_bit_op_mem ?
                        (!dec_bit_op_space ?
                         x_data_hold[dec_bit_op_bit] :
                         y_data_hold[dec_bit_op_bit]) :
                        ((dec_is_jclr || dec_is_jset) ?
                         j_bit_reg_value[dec_bit_op_bit] :
                         b_bit_reg_value[dec_bit_op_bit]);
    reg [23:0] bit_reg_result;
    always @(*) begin
        case (dec_bit_op_type)
            2'b11: bit_reg_result = b_bit_reg_value |
                                       (24'd1 << dec_bit_op_bit);
            2'b10: bit_reg_result = b_bit_reg_value &
                                       ~(24'd1 << dec_bit_op_bit);
            2'b01: bit_reg_result = b_bit_reg_value ^
                                       (24'd1 << dec_bit_op_bit);
            default: bit_reg_result = b_bit_reg_value;
        endcase
    end
    wire taking_jclr = dec_is_jclr && !jclr_bit_val;
    wire taking_jset = dec_is_jset && jclr_bit_val;
    wire taking_js_bit = (taking_jclr || taking_jset) &&
                         dec_branch_push_pc;

    // The system stack is a 15-entry array addressed by a six-bit SP whose
    // upper bits are sticky status: SE=SP[4], UF=SP[5].  Every implicit
    // single/double stack operation derives its result from the low pointer;
    // this corrects a user-forced UF=1,SE=0 state.  A transition to SE=1
    // raises the nonmaskable P:$0002 exception; once SE is set, implicit
    // operations leave SP and the backing array untouched until software
    // explicitly writes SP.
    wire stack_ssh_push = move_wr_en_c && move_wr_code_c == 6'h3c;
    wire stack_ssh_pop =
        (movep_reg_ref && decode_inst[15] &&
         decode_inst[13:8] == 6'h3c) ||
        (dec_is_movec_rr && movec_rr_src_reg == 6'h3c) ||
        (dec_is_movem && !decode_inst[15] &&
         decode_inst[5:0] == 6'h3c) ||
        (movec_mem_ref && !decode_inst[15] &&
         decode_inst[4:0] == 5'h1c) ||
        (dec_loop_is_rep && rep_reg_form &&
         decode_inst[13:8] == 6'h3c) ||
        (dec_is_bit_op && !dec_bit_op_mem &&
         !dec_is_jclr && !dec_is_jset &&
         dec_bit_op_addr == 6'h2c && dec_bit_op_type == 2'b00);
    wire stack_single_push =
        (dec_branch_push_pc && branch_taken) || taking_js_bit ||
        stack_ssh_push;
    wire stack_single_pull = dec_is_rts || dec_is_rti || stack_ssh_pop;
    wire stack_double_push = dec_loop_is_do;
    wire stack_double_pull = dec_loop_is_enddo ||
        (!rep_active && !dec_loop_is_rep && loop_last &&
         !dec_loop_is_enddo);
    wire stack_overflow_event = state == S_WB_FETCH && stack_sp_operable &&
        ((stack_single_push && stack_sp_base == 6'd15) ||
         (stack_double_push && stack_sp_base >= 6'd14));
    wire stack_underflow_event = state == S_WB_FETCH && stack_sp_operable &&
        ((stack_single_pull && stack_sp_base == 6'd0) ||
         (stack_double_pull && stack_sp_base < 6'd2));
    // Direct BSET/BCHG of SP.SE is a user write, but the documented zero-to-
    // one transition still raises the nonmaskable stack-error exception.
    // Clearing SE and writing an already-set SE do not retrigger it.
    wire stack_direct_se_set_event = state == S_WB_FETCH &&
        dec_is_bit_op && !dec_bit_op_mem &&
        !dec_is_jclr && !dec_is_jset && dec_bit_op_addr == 6'h2b &&
        dec_bit_op_bit == 5'd4 && dec_bit_op_type != 2'b00 &&
        !sp[4] && bit_reg_result[4];
    wire stack_error_accept = !irq_vector_active &&
                              (stack_overflow_event ||
                               stack_underflow_event ||
                               stack_direct_se_set_event);
    // SR.T raises the nonmaskable P:$0004 exception after each completed
    // instruction.  REP and its follower(s) form one traced package.  Three
    // instruction cycles equal six oscillator clocks in this 2T-step model;
    // hold the sequential pipeline quiet for those clocks before fetching the
    // vector.  III and stack error outrank trace; trace in turn outranks SWI
    // and every maskable source.
    wire trace_start = state == S_WB_FETCH && sr[13] &&
                       !irq_vector_active && !illegal_accept &&
                       !stack_error_accept && !dec_loop_is_rep &&
                       !rep_active && !rep_boundary_pending &&
                       !dec_is_stop && !dec_is_reset_inst;
    wire rep_boundary_trace_start = state == S_DECODE_EXEC &&
                                    rep_boundary_pending && sr[13] &&
                                    !rep_illegal_pending &&
                                    !irq_vector_active && !dec_loop_is_rep;
    wire trace_flush_complete = state == S_HALT && trace_flush_active &&
                                trace_flush_steps == 3'd1;

    // ========================================================================
    // State Machine
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= S_RESET;
        else if (clock_enable)
            state <= next_state;
    end

    // Ready/hold contract for program fetches.  Once a request is denied,
    // preserve its address until the accepting edge.  Interrupt edges remain
    // live while waiting, but cannot replace an already-issued transaction.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_fetch_stalled <= 1'b0;
            p_fetch_addr_hold <= 16'h0;
            p_fetch_irq_hold <= 1'b0;
            p_fetch_irq_vector_hold <= 16'h0;
            p_fetch_irq_level_hold <= 2'b00;
        end else if (clock_enable && p_fetch_en) begin
            if (!p_fetch_ready) begin
                if (!p_fetch_stalled) begin
                    p_fetch_stalled <= 1'b1;
                    p_fetch_addr_hold <= p_fetch_addr;
                    p_fetch_irq_hold <= irq_fetch_selected;
                    p_fetch_irq_vector_hold <= irq_accept_vector;
                    p_fetch_irq_level_hold <= irq_accept_level;
                end
            end else if (p_fetch_stalled) begin
                p_fetch_stalled <= 1'b0;
                p_fetch_irq_hold <= 1'b0;
            end
        end
    end

    // Bridge one-clock ready responses into the clock-enabled architectural
    // domain.  A channel that completes while its companion is still waiting
    // remains accepted until the whole issuing state can advance.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_req_outstanding <= 1'b0;
            y_req_outstanding <= 1'b0;
            x_req_accepted <= 1'b0;
            y_req_accepted <= 1'b0;
        end else begin
            if (x_req_outstanding && (x_rd_en || x_wr_en) && x_ready) begin
                x_req_outstanding <= 1'b0;
                x_req_accepted <= 1'b1;
            end
            if (y_req_outstanding && (y_rd_en || y_wr_en) && y_ready) begin
                y_req_outstanding <= 1'b0;
                y_req_accepted <= 1'b1;
            end

            if (clock_enable) begin
                if (!xy_issue_state || !xy_issue_wait) begin
                    x_req_outstanding <= 1'b0;
                    y_req_outstanding <= 1'b0;
                    x_req_accepted <= 1'b0;
                    y_req_accepted <= 1'b0;
                end else begin
                    if ((x_rd_en || x_wr_en) && x_ready)
                        x_req_accepted <= 1'b1;
                    else if ((x_rd_en || x_wr_en) && !x_ready)
                        x_req_outstanding <= 1'b1;

                    if ((y_rd_en || y_wr_en) && y_ready)
                        y_req_accepted <= 1'b1;
                    else if ((y_rd_en || y_wr_en) && !y_ready)
                        y_req_outstanding <= 1'b1;
                end
            end
        end
    end

    always @(*) begin
        next_state = state;
        case (state)
            S_RESET:       next_state = run ? S_WB_FETCH : S_RESET;
            S_IDLE:        next_state = halt ? S_HALT : S_WB_FETCH;
            S_WB_FETCH:    next_state = (p_fetch_wait || xy_writeback_wait) ?
                                        S_WB_FETCH :
                                        trace_start ? S_HALT :
                                        dec_is_stop ? S_HALT :
                                        dec_is_wait ? S_HALT :
                                        irq_arrived_during_p_wait ?
                                        S_IRQ_FETCH :
                                        (irq_vector_first_one_word_fetch &&
                                         dec_is_branch_op && branch_taken) ?
                                        S_IRQ_FETCH : S_DECODE_EXEC;
            S_DECODE_EXEC: begin
                if (p_fetch_wait)
                    next_state = S_DECODE_EXEC;
                else if (data_issue_wait)
                    next_state = S_DECODE_EXEC;
                else if (rep_boundary_illegal_accept)
                    next_state = S_IRQ_FETCH;
                else if (rep_boundary_trace_start)
                    next_state = S_HALT;
                else if (rep_boundary_irq_accept)
                    next_state = S_IRQ_FETCH;
                else if (fetching_ext)
                    // Just latched ext word; instruction fully decoded
                    next_state = needs_mem ? S_MEM : S_WB_FETCH;
                else if (dec_is_two_word)
                    // Need extension word: stay to latch it
                    next_state = S_LATCH_EXT;
                else if (dec_is_reset_inst)
                    // RESET occupies four oscillator clocks.  Its successor
                    // has already been fetched and is retained across the two
                    // quiet states, exactly as for the four-clock REP setup.
                    next_state = S_REP_WAIT1;
                else if (dec_loop_is_rep && (!needs_mem || rep_fast_mem))
                    next_state = S_REP_WAIT1;
                else if (bit_data_op && !dec_bit_op_mem)
                    // Register bit operations are four clocks: the ordinary
                    // decode/writeback path needs two quiet completion clocks.
                    next_state = S_DO_WAIT2;
                else if (dec_is_rts || dec_is_rti || dec_is_lua)
                    // UMA3 A-6/A-12: zero-wait RTS, RTI, and LUA each occupy
                    // four clocks.  Their architectural effects remain at
                    // the single writeback boundary after two quiet clocks.
                    next_state = S_DO_WAIT2;
                else if (dec_is_swi)
                    // UMA3 Table A-6: SWI occupies eight clocks.  Hold the
                    // decoded instruction for six quiet clocks before its
                    // ordinary writeback boundary starts vector service.
                    next_state = S_REP_WAIT1;
                else if (dec_is_branch_op)
                    // One-word ordinary jumps are four clocks.  Indexed and
                    // predecrement EA forms carry an additional two clocks.
                    next_state = jump_slow_ea ? S_REP_WAIT1 : S_DO_WAIT2;
                else if (dec_is_l_move && l_move_external &&
                         !l_move_slow_ea)
                    // Both L buses reach the one external SRAM port.  The X
                    // request is live in decode; issue Y on the next clock.
                    next_state = S_L_MEM_Y_REQ;
                else if (dec_is_xy_move && xy_both_external)
                    // X:Y has the same external-port serialization penalty.
                    next_state = S_L_MEM_Y_REQ;
                else
                    // 1-word instruction
                    next_state = (movec_fast_mem || parallel_fast_mem ||
                                  l_move_fast_internal || xy_move_fast) ?
                                 S_WB_FETCH :
                                 needs_mem ? S_MEM : S_WB_FETCH;
            end
            S_LATCH_EXT: begin
                if (data_issue_wait)
                    next_state = S_LATCH_EXT;
                else if (dec_is_class1_immediate)
                    // Two-word immediate parallel move: base two plus the
                    // manual's two-clock immediate-address term.
                    next_state = S_DO_WAIT3;
                else if (dec_loop_is_do && do_fast_mem)
                    // Internal short/ordinary sources carry no mv term: the
                    // registered response is consumed at the six-clock WB.
                    next_state = S_DO_WAIT1;
                else if (needs_mem)
                    next_state = S_MEM;
                else if (movec_long_immediate)
                    // Decode/latch/writeback is three clocks; long immediate
                    // carries the two-clock EA term and therefore needs four.
                    next_state = S_DO_WAIT3;
                else if (dec_loop_is_do)
                    // Indexed/predecrement count sources need one more quiet
                    // clock than the ordinary S_MEM + DO completion path.
                    next_state = do_slow_ea ? S_REP_WAIT2 : S_DO_WAIT1;
                else if (dec_is_branch_op || jbit_flow)
                    // Absolute ordinary jumps and register Jbit forms enter
                    // here after decode/latch.  Both need three quiet clocks
                    // before architectural writeback (six clocks total).
                    next_state = S_DO_WAIT1;
                else
                    next_state = S_WB_FETCH;
            end
            S_MEM: begin
                // P memory shares the instruction-fetch port.  A peripheral
                // read followed by a P write therefore needs its own cycle.
                if ((dec_is_l_move && l_move_external) ||
                    (dec_is_xy_move && xy_both_external))
                    next_state = S_L_MEM_Y_REQ;
                else if (parallel_slow_ea ||
                         (dec_is_l_move && l_move_slow_ea))
                    next_state = S_DO_WAIT3;
                else if (dec_loop_is_rep)
                    next_state = S_REP_WAIT1;
                else if (dec_loop_is_do)
                    next_state = do_slow_ea ? S_REP_WAIT2 : S_DO_WAIT1;
                else if (dec_is_movep) begin
                    if (movep_p_ref && !decode_inst[15])
                        // Peripheral-to-P performs its P write first; timing
                        // is completed from S_P_WRITE below.
                        next_state = S_P_WRITE;
                    else if (movep_p_ref && dec_is_two_word)
                        // P absolute: current path is four clocks; add four.
                        next_state = S_REP_WAIT2;
                    else if (movep_p_ref && movep_slow_ea)
                        // P indexed/predecrement: three clocks; add five.
                        next_state = S_REP_WAIT1;
                    else if (movep_p_ref)
                        // P ordinary EA: three clocks; add three.
                        next_state = S_DO_WAIT1;
                    else if (dec_is_two_word)
                        // X/Y absolute: four clocks; add two.
                        next_state = S_DO_WAIT2;
                    else if (movep_slow_ea)
                        // X/Y indexed/predecrement: three clocks; add three.
                        next_state = S_DO_WAIT1;
                    else
                        // Register or ordinary X/Y EA: three clocks; add one.
                        next_state = S_DO_WAIT3;
                end
                else if (dec_is_movem) begin
                    if (dec_is_two_word)
                        // Absolute: current path is four clocks; add four.
                        next_state = S_REP_WAIT2;
                    else if (movem_slow_ea)
                        // Indexed/predecrement: three clocks; add five.
                        next_state = S_REP_WAIT1;
                    else
                        // Short and ordinary indirect: three clocks; add three.
                        next_state = S_DO_WAIT1;
                end
                else if (movec_slow_ea)
                    // Decode/memory/writeback is three clocks.  Indexed and
                    // predecrement MOVEC require one additional quiet clock.
                    next_state = S_DO_WAIT3;
                else if (bit_data_op) begin
                    if (dec_is_two_word)
                        // Absolute: decode/latch/memory/writeback is four;
                        // the manual requires six clocks.
                        next_state = S_DO_WAIT2;
                    else if (bit_slow_ea)
                        // Indexed/predecrement: three clocks plus three quiet.
                        next_state = S_DO_WAIT1;
                    else
                        // Short/ordinary/peripheral: three clocks plus one.
                        next_state = S_DO_WAIT3;
                end
                else if (jbit_flow)
                    // Memory Jbit has already spent decode/latch/read.  Fast
                    // source modes need two quiet clocks; indexed and
                    // predecrement sources need four.
                    next_state = jbit_slow_ea ? S_REP_WAIT1 : S_DO_WAIT2;
                else
                    next_state = S_WB_FETCH;
            end
            S_P_WRITE: begin
                if (p_data_issue_wait)
                    next_state = S_P_WRITE;
                else if (dec_is_two_word)
                    // Peripheral-to-P absolute is five clocks; add three.
                    next_state = S_DO_WAIT1;
                else if (movep_slow_ea)
                    // Peripheral-to-P indexed/predecrement is four; add four.
                    next_state = S_REP_WAIT2;
                else
                    // Peripheral-to-P ordinary EA is four; add two.
                    next_state = S_DO_WAIT2;
            end
            S_IRQ_FETCH:   next_state = p_fetch_wait ? S_IRQ_FETCH :
                                                        S_DECODE_EXEC;
            S_REP_WAIT1:   next_state = dec_is_swi ?
                                        ((swi_wait_steps > 3'd1) ?
                                         S_REP_WAIT1 : S_WB_FETCH) :
                                        S_REP_WAIT2;
            S_REP_WAIT2: begin
                if (jump_slow_ea || jbit_slow_ea)
                    next_state = S_DO_WAIT2;
                else if (dec_loop_is_rep && rep_slow_ea)
                    next_state = S_DO_WAIT3;
                else if (dec_loop_is_do && do_slow_ea)
                    next_state = S_DO_WAIT1;
                else
                    next_state = (dec_is_movep || dec_is_movem) ?
                                 S_DO_WAIT1 : S_WB_FETCH;
            end
            S_DO_WAIT1:    next_state = S_DO_WAIT2;
            S_DO_WAIT2:    next_state = S_DO_WAIT3;
            S_DO_WAIT3:    next_state = S_WB_FETCH;
            S_L_MEM_Y_REQ: next_state = xy_issue_wait ?
                                       S_L_MEM_Y_REQ : S_L_MEM_Y_WB;
            S_L_MEM_Y_WB:  next_state = (dec_is_l_move &&
                                          l_move_slow_ea) ?
                                         S_DO_WAIT3 : S_WB_FETCH;
            S_HALT: begin
                if (trace_flush_active)
                    next_state = trace_flush_complete ? S_IRQ_FETCH : S_HALT;
                else if (stop_active) begin
                    if (!stop_recovering)
                        next_state = S_HALT;
                    else if (stop_recovery_complete)
                        next_state = stop_trace_pending ? S_HALT :
                                     stop_irq_accept ? S_IRQ_FETCH :
                                                       S_WB_FETCH;
                    else
                        next_state = S_HALT;
                end else if (wait_active)
                    next_state = wait_irq_accept ? S_IRQ_FETCH : S_HALT;
                else
                    next_state = halt ? S_HALT : S_WB_FETCH;
            end
            default:       next_state = S_RESET;
        endcase
    end

    // ========================================================================
    // Fetch Address Logic (combinational)
    // ========================================================================
    // In S_WB_FETCH: present next PC with branch/RTS/JCLR redirect
    // In S_DECODE_EXEC (2-word): present PC for extension word fetch
    // In S_DECODE_EXEC (MOVEM): present P-data address
    always @(*) begin
        p_fetch_addr = pc;
        p_fetch_en   = 1'b0;

        if (clock_enable && p_fetch_stalled) begin
            p_fetch_en = 1'b1;
            p_fetch_addr = p_fetch_addr_hold;
        end
        else if (clock_enable && state == S_WB_FETCH) begin
            p_fetch_en = !trace_start;
            if (trace_start)
                p_fetch_addr = pc;
            else if (illegal_accept)
                p_fetch_addr = 16'h003e;
            else if (stack_error_accept)
                p_fetch_addr = 16'h0002;
            else if (swi_accept)
                p_fetch_addr = 16'h0006;
            else if (irq_accept)
                p_fetch_addr = irq_accept_vector;
            else if (irq_vector_first_one_word_fetch)
                // Fetch vector slot two even when slot one changes flow.  In
                // that case S_IRQ_FETCH discards this response and requests
                // the already-computed branch target.
                p_fetch_addr = irq_vector_base + 16'd1;
            else if (at_loop_end && !loop_last && !dec_loop_is_enddo)
                p_fetch_addr = loop_start_addr;
            else if (dec_is_branch_op && branch_taken)
                p_fetch_addr = branch_effective_target;
            else if (taking_jclr || taking_jset)
                p_fetch_addr = dec_branch_target[15:0];
            else if ((dec_is_rts || dec_is_rti) && stack_sp_operable &&
                     stack_sp_base != 6'd0)
                p_fetch_addr = ss[sp[3:0] - 4'd1][39:24];
            else if (rep_active && !rep_final_iteration)
                p_fetch_addr = rep_fetch_addr;
            else
                p_fetch_addr = pc;
        end
        else if (clock_enable && state == S_IRQ_FETCH) begin
            p_fetch_en = 1'b1;
            p_fetch_addr = irq_vector_discard_second ? pc : irq_vector_base;
        end
        else if (clock_enable && state == S_DECODE_EXEC &&
                 !rep_boundary_illegal_accept &&
                 !rep_boundary_trace_start &&
                 !rep_boundary_irq_accept && !fetching_ext &&
                 dec_is_two_word) begin
            // Fetch extension word (pc hasn't been incremented yet at this point)
            p_fetch_en   = 1'b1;
            p_fetch_addr = irq_vector_active ? irq_vector_base + 16'd1 :
                                               pc + 16'd1;
        end
    end

    // ========================================================================
    // Memory Access Logic (combinational)
    // ========================================================================
    // Present X/Y/P-data addresses during S_DECODE_EXEC or S_LATCH_EXT
    wire in_decode = clock_enable && !rep_boundary_illegal_accept &&
                     !rep_boundary_trace_start &&
                     !rep_boundary_irq_accept &&
                     ((state == S_DECODE_EXEC && !fetching_ext &&
                       !dec_is_two_word) ||
                      (state == S_DECODE_EXEC && fetching_ext) ||
                      (state == S_LATCH_EXT));

    // Use the instruction word directly for register reads in data addressing
    // (decode_inst is the instruction being decoded)
    always @(*) begin
        x_addr  = 16'h0;
        x_rd_en = 1'b0;
        x_wr_en = 1'b0;
        x_wdata = 24'h0;
        y_addr  = 16'h0;
        y_rd_en = 1'b0;
        y_wr_en = 1'b0;
        y_wdata = 24'h0;
        p_data_addr  = 16'h0;
        p_data_rd_en = 1'b0;
        p_data_wr_en = 1'b0;
        p_data_wdata = 24'h0;

        if (in_decode) begin
            // ============================================================
            // MOVEP: peripheral I/O
            // ============================================================
            if (dec_is_movep) begin
                if (movep_reg_ref) begin
                    // Register-reference form.  Bit 16 selects the peripheral
                    // space; bits 13:8 are a register code, never an EA.
                    if (decode_inst[15]) begin
                        // Register -> peripheral write
                        if (!decode_inst[16]) begin
                            x_addr = {10'h3FF, decode_inst[5:0]};
                            x_wr_en = 1'b1;
                            x_wdata = movep_reg_write_data;
                        end else begin
                            y_addr = {10'h3FF, decode_inst[5:0]};
                            y_wr_en = 1'b1;
                            y_wdata = movep_reg_write_data;
                        end
                    end else begin
                        // Peripheral -> register read
                        if (!decode_inst[16]) begin
                            x_addr = {10'h3FF, decode_inst[5:0]};
                            x_rd_en = 1'b1;
                        end else begin
                            y_addr = {10'h3FF, decode_inst[5:0]};
                            y_rd_en = 1'b1;
                        end
                    end
                end else if (movep_p_ref) begin
                    // P-memory-reference form.  Bit 16 selects the peripheral
                    // space; bits 13:8 are the P effective address.
                    if (decode_inst[15]) begin
                        p_data_addr = (decode_inst[13:11] == 3'b110) ?
                                      decode_ext[15:0] : agu_eff_addr;
                        p_data_rd_en = 1'b1;
                    end else if (!decode_inst[16]) begin
                        x_addr = {10'h3FF, decode_inst[5:0]};
                        x_rd_en = 1'b1;
                    end else begin
                        y_addr = {10'h3FF, decode_inst[5:0]};
                        y_rd_en = 1'b1;
                    end
                end else begin
                    // X/Y-memory-reference form.  Bit 16 selects the
                    // peripheral space and bit 6 selects the EA memory space.
                    if (movep_xy_immediate) begin
                        // Immediate -> peripheral write
                        if (!decode_inst[16]) begin
                            x_addr = {10'h3FF, decode_inst[5:0]};
                            x_wr_en = 1'b1;
                            x_wdata = decode_ext;
                        end else begin
                            y_addr = {10'h3FF, decode_inst[5:0]};
                            y_wr_en = 1'b1;
                            y_wdata = decode_ext;
                        end
                    end else if (decode_inst[15]) begin
                        // Memory[EA] -> peripheral: read EA first
                        if (!decode_inst[6]) begin
                            x_addr = (decode_inst[13:11] == 3'b110) ?
                                     decode_ext[15:0] : agu_eff_addr;
                            x_rd_en = 1'b1;
                        end else begin
                            y_addr = (decode_inst[13:11] == 3'b110) ?
                                     decode_ext[15:0] : agu_eff_addr;
                            y_rd_en = 1'b1;
                        end
                    end else begin
                        // Peripheral -> memory[EA]: read peripheral
                        if (!decode_inst[16]) begin
                            x_addr = {10'h3FF, decode_inst[5:0]};
                            x_rd_en = 1'b1;
                        end else begin
                            y_addr = {10'h3FF, decode_inst[5:0]};
                            y_rd_en = 1'b1;
                        end
                    end
                end
            end
            // ============================================================
            // MOVEM: P-memory data access
            // ============================================================
            else if (dec_is_movem) begin
                if (!decode_inst[14])
                    p_data_addr = {10'h0, decode_inst[13:8]};  // Short absolute
                else if (decode_inst[13:11] == 3'b110)
                    p_data_addr = decode_ext[15:0];            // Absolute (MMM=110)
                else
                    p_data_addr = agu_eff_addr;               // Includes indexed/predecrement

                if (decode_inst[15]) begin
                    p_data_rd_en = 1'b1;
                end else begin
                    p_data_wr_en = 1'b1;
                    p_data_wdata = read_reg(decode_inst[5:0]);
                end
            end
            // ============================================================
            // MOVEC: X/Y memory to or from a control register
            // ============================================================
            else if (movec_mem_ref) begin
                if (!decode_inst[6]) begin
                    x_addr = movec_mem_addr;
                    if (decode_inst[15])
                        x_rd_en = 1'b1;
                    else begin
                        x_wr_en = 1'b1;
                        x_wdata = read_reg({1'b1, decode_inst[4:0]});
                    end
                end else begin
                    y_addr = movec_mem_addr;
                    if (decode_inst[15])
                        y_rd_en = 1'b1;
                    else begin
                        y_wr_en = 1'b1;
                        y_wdata = read_reg({1'b1, decode_inst[4:0]});
                    end
                end
            end
            // ============================================================
            // Bit operations (BCLR/BSET/BCHG/BTST, JCLR/JSET)
            // ============================================================
            else if (dec_is_bit_op) begin
                if (dec_bit_op_mem) begin
                    if (!dec_bit_op_space) begin
                        case (dec_bit_op_amode)
                            2'b00: x_addr = {10'h0, dec_bit_op_addr};
                            2'b01: x_addr =
                                (decode_inst[13:11] == 3'b110) ?
                                decode_ext[15:0] : agu_eff_addr;
                            2'b10: x_addr = {10'h3FF, dec_bit_op_addr};
                            default: x_addr = 16'h0;
                        endcase
                        x_rd_en = 1'b1;
                    end else begin
                        case (dec_bit_op_amode)
                            2'b00: y_addr = {10'h0, dec_bit_op_addr};
                            2'b01: y_addr =
                                (decode_inst[13:11] == 3'b110) ?
                                decode_ext[15:0] : agu_eff_addr;
                            2'b10: y_addr = {10'h3FF, dec_bit_op_addr};
                            default: y_addr = 16'h0;
                        endcase
                        y_rd_en = 1'b1;
                    end
                end
            end
            // ============================================================
            // REP memory count source
            // ============================================================
            else if (rep_mem_form) begin
                if (!rep_mem_space) begin
                    x_addr = rep_ea_form ? agu_eff_addr :
                                           {10'h0, decode_inst[13:8]};
                    x_rd_en = 1'b1;
                end else begin
                    y_addr = rep_ea_form ? agu_eff_addr :
                                           {10'h0, decode_inst[13:8]};
                    y_rd_en = 1'b1;
                end
            end
            // ============================================================
            // DO memory count source
            // ============================================================
            else if (do_mem_form) begin
                if (!do_mem_space) begin
                    x_addr = do_ea_form ? agu_eff_addr :
                                         {10'h0, decode_inst[13:8]};
                    x_rd_en = 1'b1;
                end else begin
                    y_addr = do_ea_form ? agu_eff_addr :
                                         {10'h0, decode_inst[13:8]};
                    y_rd_en = 1'b1;
                end
            end
            // ============================================================
            // Parallel moves (X/Y memory)
            // ============================================================
            else if (dec_is_l_move) begin
                // Internal X/Y memories are independent and issue together.
                // When both views reach the shared external SRAM, issue X
                // here and Y in the dedicated state below.
                x_addr = l_move_addr;
                if (l_move_load)
                    x_rd_en = 1'b1;
                else begin
                    x_wr_en = 1'b1;
                    x_wdata = l_move_x_source;
                end
                if (!l_move_external) begin
                    y_addr = l_move_addr;
                    if (l_move_load)
                        y_rd_en = 1'b1;
                    else begin
                        y_wr_en = 1'b1;
                        y_wdata = l_move_y_source;
                    end
                end
            end
            else if (dec_is_xy_move) begin
                // Preserve architectural dual-bus issue unless both external
                // views converge on CYAN's one physical SRAM port.  Only that
                // conflicting case defers Y to S_L_MEM_Y_REQ.
                x_addr = xy_x_eff_addr;
                if (decode_inst[22]) begin
                    x_wr_en = 1'b1;
                    x_wdata = read_reg(xy_x_reg);
                end else begin
                    x_rd_en = 1'b1;
                end
                if (!xy_both_external) begin
                    y_addr = xy_y_eff_addr;
                    if (decode_inst[15]) begin
                        y_wr_en = 1'b1;
                        y_wdata = read_reg(xy_y_reg);
                    end else begin
                        y_rd_en = 1'b1;
                    end
                end
            end
            else if (dec_move_src_mem || dec_move_dst_mem) begin
                if (!dec_move_mem_space) begin
                    // Class-I X:R always encodes MMMRRR here; its fixed zero
                    // at bit 14 is not the ordinary X/Y short-address flag.
                    // Its MMMRRR=110000 form takes the extension address.
                    x_addr = (dec_is_xr_ry_class1 || dec_is_xr_ry_class2) ?
                             ((decode_inst[13:8] == 6'b110000) ?
                              decode_ext[15:0] : agu_eff_addr) :
                             !decode_inst[14] ?
                             {10'h0, dec_addr_absolute} :
                             (dec_addr_mode_mmm == 3'b110) ?
                             decode_ext[15:0] : agu_eff_addr;
                    if (dec_move_src_mem)
                        x_rd_en = 1'b1;
                    if (dec_move_dst_mem) begin
                        x_wr_en = 1'b1;
                        x_wdata = read_reg(dec_move_src_reg);
                    end
                end else begin
                    y_addr = dec_is_xr_ry_class2 ? agu_eff_addr :
                             !decode_inst[14] ?
                             {10'h0, dec_addr_absolute} :
                             (dec_addr_mode_mmm == 3'b110) ?
                             decode_ext[15:0] : agu_eff_addr;
                    if (dec_move_src_mem)
                        y_rd_en = 1'b1;
                    if (dec_move_dst_mem) begin
                        y_wr_en = 1'b1;
                        y_wdata = read_reg(dec_move_src_reg);
                    end
                end
            end
        end

        // Second half of an L: or X:Y transfer.  One subsequent architectural
        // state retains the synchronous SRAM response before register
        // writeback.
        if (clock_enable && state == S_L_MEM_Y_REQ) begin
            if (dec_is_l_move && l_move_external) begin
                y_addr = l_move_addr;
                if (l_move_load)
                    y_rd_en = 1'b1;
                else begin
                    y_wr_en = 1'b1;
                    y_wdata = l_move_y_source;
                end
            end else if (dec_is_xy_move && xy_both_external) begin
                y_addr = xy_y_eff_addr;
                if (decode_inst[15]) begin
                    y_wr_en = 1'b1;
                    y_wdata = read_reg(xy_y_reg);
                end else begin
                    y_rd_en = 1'b1;
                end
            end
        end

        // ================================================================
        // Bit op WRITE-BACK in S_WB_FETCH (BSET/BCLR/BCHG: read-modify-write)
        // ================================================================
        if (clock_enable && state == S_WB_FETCH && dec_is_bit_op &&
            dec_bit_op_mem && (dec_bit_op_type != 2'b00)) begin
            if (!dec_bit_op_space) begin
                case (dec_bit_op_amode)
                    2'b00: x_addr = {10'h0, dec_bit_op_addr};
                    2'b01: x_addr =
                        (decode_inst[13:11] == 3'b110) ?
                        decode_ext[15:0] : agu_eff_addr;
                    2'b10: x_addr = {10'h3FF, dec_bit_op_addr};
                    default: x_addr = 16'h0;
                endcase
                x_wr_en = 1'b1;
                case (dec_bit_op_type)
                    2'b11: x_wdata = x_data_hold | (24'd1 << dec_bit_op_bit);
                    2'b10: x_wdata = x_data_hold & ~(24'd1 << dec_bit_op_bit);
                    2'b01: x_wdata = x_data_hold ^ (24'd1 << dec_bit_op_bit);
                    default: x_wdata = x_data_hold;
                endcase
            end else begin
                case (dec_bit_op_amode)
                    2'b00: y_addr = {10'h0, dec_bit_op_addr};
                    2'b01: y_addr =
                        (decode_inst[13:11] == 3'b110) ?
                        decode_ext[15:0] : agu_eff_addr;
                    2'b10: y_addr = {10'h3FF, dec_bit_op_addr};
                    default: y_addr = 16'h0;
                endcase
                y_wr_en = 1'b1;
                case (dec_bit_op_type)
                    2'b11: y_wdata = y_data_hold | (24'd1 << dec_bit_op_bit);
                    2'b10: y_wdata = y_data_hold & ~(24'd1 << dec_bit_op_bit);
                    2'b01: y_wdata = y_data_hold ^ (24'd1 << dec_bit_op_bit);
                    default: y_wdata = y_data_hold;
                endcase
            end
        end

        // ================================================================
        // MOVEP memory-reference write-backs in S_WB_FETCH
        // ================================================================
        if (clock_enable && state == S_WB_FETCH &&
            (movep_xy_ref || movep_p_ref) &&
            decode_inst[15] && !movep_xy_immediate) begin
            if (!decode_inst[16]) begin
                x_addr = {10'h3FF, decode_inst[5:0]};
                x_wr_en = 1'b1;
                x_wdata = movep_p_ref ? p_data_hold :
                          (decode_inst[6] ? y_data_hold : x_data_hold);
            end else begin
                y_addr = {10'h3FF, decode_inst[5:0]};
                y_wr_en = 1'b1;
                y_wdata = movep_p_ref ? p_data_hold :
                          (decode_inst[6] ? y_data_hold : x_data_hold);
            end
        end
        if (clock_enable && state == S_WB_FETCH && movep_xy_ref &&
            !decode_inst[15]) begin
            if (!decode_inst[6]) begin
                x_addr = (decode_inst[13:11] == 3'b110) ? decode_ext[15:0] : agu_eff_addr;
                x_wr_en = 1'b1;
                x_wdata = decode_inst[16] ? y_data_hold : x_data_hold;
            end else begin
                y_addr = (decode_inst[13:11] == 3'b110) ? decode_ext[15:0] : agu_eff_addr;
                y_wr_en = 1'b1;
                y_wdata = decode_inst[16] ? y_data_hold : x_data_hold;
            end
        end

        // P memory and instruction fetch share a port, so complete a
        // peripheral-to-P transfer in the dedicated non-fetching state.
        if (clock_enable && state == S_P_WRITE && movep_p_ref &&
            !decode_inst[15]) begin
            p_data_addr = (decode_inst[13:11] == 3'b110) ?
                          decode_ext[15:0] : agu_eff_addr;
            p_data_wr_en = 1'b1;
            p_data_wdata = decode_inst[16] ? y_data_hold : x_data_hold;
        end

        // A completed response can arrive between architectural core clocks.
        // Do not re-issue that exact half on the clock which consumes the
        // retained acknowledgement; any unresolved companion half remains
        // live and keeps the issuing state stalled.
        if (xy_issue_state && x_req_complete) begin
            x_rd_en = 1'b0;
            x_wr_en = 1'b0;
        end
        if (xy_issue_state && y_req_complete) begin
            y_rd_en = 1'b0;
            y_wr_en = 1'b0;
        end
    end

    // Debug: trace Y-peripheral writes (HPC1 HANDTX = diagnostic result)
`ifndef SYNTHESIS
    always @(posedge clk) begin
        if (y_wr_en && (y_addr == 16'hFFE6) &&
            $test$plusargs("trace_handtx"))
            $display("HANDTX_WR: PC=%04X data=%06X", pc, y_wdata);
    end
`endif

    // ========================================================================
    // MOVEC Register-to-Register: Source Register Read Mux
    // ========================================================================
    wire [5:0] movec_rr_src_reg = decode_inst[15] ? decode_inst[13:8] : decode_inst[5:0];
    wire [5:0] movec_rr_dst_reg = decode_inst[15] ? decode_inst[5:0] : decode_inst[13:8];
    reg [23:0] movec_rr_src_val;

    always @(*) begin
        movec_rr_src_val = 24'h0;
        case (movec_rr_src_reg)
            6'h04: movec_rr_src_val = x0_reg;
            6'h05: movec_rr_src_val = x1_reg;
            6'h06: movec_rr_src_val = y0_reg;
            6'h07: movec_rr_src_val = y1_reg;
            6'h08: movec_rr_src_val = acc_a[23:0];
            6'h09: movec_rr_src_val = acc_b[23:0];
            6'h0A: movec_rr_src_val = {16'h0, acc_a[55:48]};
            6'h0B: movec_rr_src_val = {16'h0, acc_b[55:48]};
            6'h0C: movec_rr_src_val = acc_a[47:24];
            6'h0D: movec_rr_src_val = acc_b[47:24];
            6'h0E: movec_rr_src_val = acc_a_bus_word[23:0];
            6'h0F: movec_rr_src_val = acc_b_bus_word[23:0];
            6'h10: movec_rr_src_val = {8'h0, r_reg[0]};
            6'h11: movec_rr_src_val = {8'h0, r_reg[1]};
            6'h12: movec_rr_src_val = {8'h0, r_reg[2]};
            6'h13: movec_rr_src_val = {8'h0, r_reg[3]};
            6'h14: movec_rr_src_val = {8'h0, r_reg[4]};
            6'h15: movec_rr_src_val = {8'h0, r_reg[5]};
            6'h16: movec_rr_src_val = {8'h0, r_reg[6]};
            6'h17: movec_rr_src_val = {8'h0, r_reg[7]};
            6'h18: movec_rr_src_val = {8'h0, n_reg[0]};
            6'h19: movec_rr_src_val = {8'h0, n_reg[1]};
            6'h1A: movec_rr_src_val = {8'h0, n_reg[2]};
            6'h1B: movec_rr_src_val = {8'h0, n_reg[3]};
            6'h1C: movec_rr_src_val = {8'h0, n_reg[4]};
            6'h1D: movec_rr_src_val = {8'h0, n_reg[5]};
            6'h1E: movec_rr_src_val = {8'h0, n_reg[6]};
            6'h1F: movec_rr_src_val = {8'h0, n_reg[7]};
            6'h20: movec_rr_src_val = {8'h0, m_reg[0]};
            6'h21: movec_rr_src_val = {8'h0, m_reg[1]};
            6'h22: movec_rr_src_val = {8'h0, m_reg[2]};
            6'h23: movec_rr_src_val = {8'h0, m_reg[3]};
            6'h24: movec_rr_src_val = {8'h0, m_reg[4]};
            6'h25: movec_rr_src_val = {8'h0, m_reg[5]};
            6'h26: movec_rr_src_val = {8'h0, m_reg[6]};
            6'h27: movec_rr_src_val = {8'h0, m_reg[7]};
            6'h39: movec_rr_src_val = sr_arch;
            6'h3A: movec_rr_src_val = {16'h0, omr};
            6'h3B: movec_rr_src_val = {18'h0, sp};
            6'h3C: movec_rr_src_val = stack_sp_operable ? {8'h0,
                ss[(sp[3:0] == 4'd0) ? 4'd0 :
                   sp[3:0] - 4'd1][39:24]} : 24'h0;
            6'h3D: movec_rr_src_val = stack_sp_operable ? {8'h0,
                ss[(sp[3:0] == 4'd0) ? 4'd0 :
                   sp[3:0] - 4'd1][15:0]} : 24'h0;
            6'h3E: movec_rr_src_val = {8'h0, la};
            6'h3F: movec_rr_src_val = {8'h0, lc[15:0]};
            default: movec_rr_src_val = 24'h0;
        endcase
    end

    // ========================================================================
    // Move Register Write Logic (combinational)
    // ========================================================================
    always @(*) begin
        move_wr_en_c   = 1'b0;
        move_wr_code_c = 6'h0;
        move_wr_data_c = 24'h0;

        if (dec_is_movec_rr) begin
            move_wr_en_c   = 1'b1;
            move_wr_code_c = movec_rr_dst_reg;
            move_wr_data_c = movec_rr_src_val;
        end
        else if (dec_is_lua) begin
            // LUA writes the computed update to D.  Unlike an ordinary
            // memory effective address, it never commits the source Rn.
            move_wr_en_c   = 1'b1;
            move_wr_code_c = {1'b0, dec_lua_ddddd};
            move_wr_data_c = {8'h0, agu_r_out};
        end
        else if (dec_is_parallel && dec_move_type == 4'd1) begin // MOVE_REG
            move_wr_en_c   = 1'b1;
            move_wr_code_c = dec_move_dst_reg;
            move_wr_data_c = read_reg(dec_move_src_reg);
        end
        else if (dec_is_movec && decode_inst[7:5] == 3'b101) begin
            // MOVE(C) #xx,D1 is 00000101 iiiiiiii 101ddddd.
            // The full middle byte is immediate data; its MS bit is not the
            // W direction bit used by the other MOVEC encodings.
            move_wr_en_c   = 1'b1;
            move_wr_code_c = decode_inst[5:0];
            move_wr_data_c = {16'h0, decode_inst[15:8]};
        end
        else if (movec_long_immediate) begin
            move_wr_en_c   = 1'b1;
            move_wr_code_c = decode_inst[5:0];
            move_wr_data_c = decode_ext;
        end
        else if (movec_mem_ref && decode_inst[15]) begin
            move_wr_en_c   = 1'b1;
            move_wr_code_c = decode_inst[5:0];
            // Fast ordinary forms bypass S_MEM: the synchronous SRAM request
            // was issued in decode and its registered result is live now.
            // Slow/absolute forms retain the ordinary captured response.
            move_wr_data_c = decode_inst[6] ?
                             (movec_fast_mem ? y_rdata : y_data_hold) :
                             (movec_fast_mem ? x_rdata : x_data_hold);
        end
        else if (dec_move_is_imm && dec_is_move_op) begin
            move_wr_en_c   = 1'b1;
            move_wr_code_c = dec_move_dst_reg;
            move_wr_data_c = dec_move_imm_data;
        end
        else if (dec_is_class1_immediate) begin
            move_wr_en_c   = 1'b1;
            move_wr_code_c = dec_move_dst_reg;
            move_wr_data_c = decode_ext;
        end
        else if (dec_is_movem && decode_inst[15]) begin
            move_wr_en_c   = 1'b1;
            move_wr_code_c = decode_inst[5:0];
            move_wr_data_c = p_data_hold;
        end
        else if (movep_reg_ref && !decode_inst[15]) begin
            move_wr_en_c   = 1'b1;
            move_wr_code_c = decode_inst[13:8];
            move_wr_data_c = !decode_inst[16] ? x_data_hold : y_data_hold;
        end
        else if (dec_move_src_mem && dec_is_move_op && !dec_is_l_move) begin
            move_wr_en_c   = 1'b1;
            move_wr_code_c = dec_move_dst_reg;
            move_wr_data_c = !dec_move_mem_space ?
                             (parallel_fast_mem ? x_rdata : x_data_hold) :
                             (parallel_fast_mem ? y_rdata : y_data_hold);
        end
    end

    reg        addr_delay_next_valid;
    reg [1:0]  addr_delay_next_kind;
    reg [2:0]  addr_delay_next_index;
    reg [15:0] addr_delay_next_value;

    // Capture the pre-write value of an explicit move-type R/N/M
    // destination.  Tcc's optional R-register transfer is the only such
    // destination outside the unified move write path.
    always @(*) begin
        addr_delay_next_valid = 1'b0;
        addr_delay_next_kind  = ADDR_DELAY_R;
        addr_delay_next_index = 3'd0;
        addr_delay_next_value = 16'h0;

        if (move_wr_en_c && move_wr_code_c >= 6'h10 &&
            move_wr_code_c <= 6'h27) begin
            addr_delay_next_valid = 1'b1;
            addr_delay_next_index = move_wr_code_c[2:0];
            if (move_wr_code_c < 6'h18) begin
                addr_delay_next_kind  = ADDR_DELAY_R;
                addr_delay_next_value = r_reg[move_wr_code_c[2:0]];
            end else if (move_wr_code_c < 6'h20) begin
                addr_delay_next_kind  = ADDR_DELAY_N;
                addr_delay_next_value = n_reg[move_wr_code_c[2:0]];
            end else begin
                addr_delay_next_kind  = ADDR_DELAY_M;
                addr_delay_next_value = m_reg[move_wr_code_c[2:0]];
            end
        end

        if (dec_is_tcc && cond_result && dec_tcc_dual) begin
            addr_delay_next_valid = 1'b1;
            addr_delay_next_kind  = ADDR_DELAY_R;
            addr_delay_next_index = dec_tcc_r_dest;
            addr_delay_next_value = r_reg[dec_tcc_r_dest];
        end
    end

    // ========================================================================
    // Writeback & Register Update
    // ========================================================================
    integer j;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_a  <= 56'h0;
            acc_b  <= 56'h0;
            x0_reg <= 24'h0;
            x1_reg <= 24'h0;
            y0_reg <= 24'h0;
            y1_reg <= 24'h0;
            for (j = 0; j < 8; j = j + 1) begin
                r_reg[j] <= 16'h0;
                n_reg[j] <= 16'h0;
                m_reg[j] <= 16'hFFFF;
            end
            addr_delay_valid <= 1'b0;
            addr_delay_kind  <= ADDR_DELAY_R;
            addr_delay_index <= 3'd0;
            addr_delay_value <= 16'h0;
            sr     <= 24'h0300;
            omr    <= 8'h0;
            pc     <= 16'h0;
            la     <= 16'h0;
            lc     <= 24'h0;
            sp     <= 6'h0;
            for (j = 0; j < 15; j = j + 1)
                ss[j] <= 48'h0;
            cur_inst <= 24'h0;
            ext_word <= 24'h0;
            p_data_hold <= 24'h0;
            x_data_hold <= 24'h0;
            y_data_hold <= 24'h0;
            fetching_ext <= 1'b0;
            irq_vector_active <= 1'b0;
            irq_vector_second_slot <= 1'b0;
            irq_vector_discard_second <= 1'b0;
            irq_vector_base <= 16'h0;
            irq_vector_level <= 2'b00;
            rep_active <= 1'b0;
            rep_fetch_addr <= 16'h0;
            rep_saved_lc <= 24'h0;
            rep_boundary_pending <= 1'b0;
            rep_illegal_pending <= 1'b0;
            wait_active <= 1'b0;
            wait_pending_steps <= 4'd0;
            stop_active <= 1'b0;
            stop_recovering <= 1'b0;
            stop_recovery_steps <= 17'd0;
            stop_trace_pending <= 1'b0;
            trace_flush_active <= 1'b0;
            trace_flush_steps <= 3'd0;
            swi_wait_steps <= 3'd0;
        end else begin
            // ================================================================
            // Debug register write (override)
            // ================================================================
            if (dbg_reg_wr_en) begin
                case (dbg_reg_wr_addr)
                    6'h04: x0_reg <= dbg_reg_wr_data[23:0];
                    6'h05: x1_reg <= dbg_reg_wr_data[23:0];
                    6'h06: y0_reg <= dbg_reg_wr_data[23:0];
                    6'h07: y1_reg <= dbg_reg_wr_data[23:0];
                    6'h0E: acc_a  <= dbg_reg_wr_data[55:0];
                    6'h0F: acc_b  <= dbg_reg_wr_data[55:0];
                    6'h39: sr     <= dbg_reg_wr_data[23:0] & SR_ARCH_MASK;
                    6'h3A: omr    <= dbg_reg_wr_data[7:0];
                    default: ;
                endcase
            end

            if (clock_enable && !p_fetch_wait && !xy_writeback_wait) begin
            case (state)
                // ============================================================
                // S_WB_FETCH: Present fetch address + writeback all results
                // ============================================================
                S_WB_FETCH: begin
                    addr_delay_valid <= addr_delay_next_valid;
                    addr_delay_kind  <= addr_delay_next_kind;
                    addr_delay_index <= addr_delay_next_index;
                    addr_delay_value <= addr_delay_next_value;

                    // External interrupts are sampled only at an instruction
                    // boundary.  Fetching the two-word vector does not alter
                    // PC, so a vector JSR stacks the normal resume PC.  The
                    // board vector uses that long-interrupt form.
                    if (illegal_accept) begin
                        irq_vector_active <= 1'b1;
                        irq_vector_second_slot <= 1'b0;
                        irq_vector_discard_second <= 1'b0;
                        irq_vector_base <= 16'h003e;
                        irq_vector_level <= 2'b11;
                        pc <= pc - 16'd1;
                    end else if (stack_error_accept) begin
                        irq_vector_active <= 1'b1;
                        irq_vector_second_slot <= 1'b0;
                        irq_vector_discard_second <= 1'b0;
                        irq_vector_base <= 16'h0002;
                        irq_vector_level <= 2'b11;
                    end else if (trace_start) begin
                        trace_flush_active <= 1'b1;
                        trace_flush_steps <= 3'd6;
                    end else if (swi_accept) begin
                        irq_vector_active <= 1'b1;
                        irq_vector_second_slot <= 1'b0;
                        irq_vector_discard_second <= 1'b0;
                        irq_vector_base <= 16'h0006;
                        irq_vector_level <= 2'b11;
                    end else if (wb_irq_accept) begin
                        irq_vector_active <= 1'b1;
                        irq_vector_second_slot <= 1'b0;
                        irq_vector_discard_second <= 1'b0;
                        irq_vector_base <= wb_irq_vector;
                        irq_vector_level <= wb_irq_level;
                    end else if (irq_vector_active) begin
                        // A one-word first slot either advances to the second
                        // fast instruction or, after a taken flow change,
                        // discards the fetched second slot.  A two-word first
                        // slot and every second slot complete the package.
                        if (!irq_vector_second_slot && !dec_is_two_word &&
                            !(dec_is_branch_op && branch_taken)) begin
                            irq_vector_second_slot <= 1'b1;
                        end else begin
                            irq_vector_active <= 1'b0;
                            irq_vector_second_slot <= 1'b0;
                            irq_vector_discard_second <=
                                !irq_vector_second_slot &&
                                !dec_is_two_word && dec_is_branch_op &&
                                branch_taken;
                        end
                        if (dec_branch_push_pc && branch_taken) begin
                            sr[15] <= 1'b0;       // clear loop flag
                            sr[13] <= 1'b0;       // clear trace
                            sr[11:10] <= 2'b00;   // clear scaling mode
                            sr[9:8] <= irq_vector_level;
                        end
                    end

                    // WAIT aborts the successor fetched in this cycle and
                    // suspends core retirement.  If an unmasked interrupt is
                    // already pending, UMA2 A-220/A-221 and UM08 Figure 8-16
                    // require the equivalent of eight NOPs (a minimum of 16
                    // oscillator clocks) before vector fetch.  Thirteen quiet
                    // clocks here, together with decode, this writeback clock,
                    // and vector handoff, establish that exact minimum.
                    if (dec_is_wait && !trace_start) begin
                        wait_active <= 1'b1;
                        wait_pending_steps <= irq_any_serviceable ? 4'd13 :
                                                                    4'd0;
                    end

                    // STOP always enters its own processing state, even when
                    // IRQA is already asserted.  IRQA starts (but does not
                    // bypass) the OMR-selected clock-recovery interval.
                    if (dec_is_stop) begin
                        stop_active <= 1'b1;
                        stop_recovering <= irq_a;
                        stop_recovery_steps <= irq_a ?
                                                   stop_recovery_period :
                                                   17'd0;
                        // T is sampled at instruction entry.  STOP completes
                        // with a trace exception pending, but the exception
                        // survives the stopped-clock interval rather than
                        // preventing entry to the stop processing state.
                        stop_trace_pending <= sr[13];
                    end

                    // ALU result writeback
                    if (dec_is_alu_op && alu_result_valid && !dec_alu_no_writeback) begin
                        if (dec_alu_dest == 1'b0)
                            acc_a <= alu_result;
                        else
                            acc_b <= alu_result;
                    end

                    // CCR update from ALU
                    if (dec_is_alu_op && alu_ccr_valid)
                        sr[7:0] <= {1'b0, alu_ccr_out[6:0]};

                    // Tcc is a move-type operation: when false it is a NOP;
                    // when true both destinations observe their old sources
                    // on this edge and CCR remains unchanged.
                    if (dec_is_tcc && cond_result) begin
                        if (dec_tcc_alu_dest)
                            acc_b <= tcc_alu_source;
                        else
                            acc_a <= tcc_alu_source;
                        if (dec_tcc_dual)
                            r_reg[dec_tcc_r_dest] <= r_reg[dec_tcc_r_src];
                    end

                    // L is sticky when a full accumulator is limited onto
                    // XDB or YDB.  SR[7] is reserved on DSP56000/DSP56001.
                    if (movep_reg_full_acc_source && movep_acc_limit)
                        sr[6] <= 1'b1;
                    if (dec_is_l_move && !l_move_load && l_move_limit)
                        sr[6] <= 1'b1;
                    if (dec_is_xy_move &&
                        ((decode_inst[22] &&
                          ((xy_x_reg == 6'h0e && acc_a_bus_word[24]) ||
                           (xy_x_reg == 6'h0f && acc_b_bus_word[24]))) ||
                         (decode_inst[15] &&
                          ((xy_y_reg == 6'h0e && acc_a_bus_word[24]) ||
                           (xy_y_reg == 6'h0f && acc_b_bus_word[24])))))
                        sr[6] <= 1'b1;
                    // MOVEM uses the same scaled/limited 24-bit data-bus
                    // view for a full A/B source.  The saturated P-memory
                    // payload is already selected by read_reg(); retain the
                    // corresponding sticky limiting indication as well.
                    if (dec_is_movem && !decode_inst[15] &&
                        ((decode_inst[5:0] == 6'h0e && acc_a_bus_word[24]) ||
                         (decode_inst[5:0] == 6'h0f && acc_b_bus_word[24])))
                        sr[6] <= 1'b1;
                    // Register MOVEC likewise scales and limits a full A/B
                    // source.  A non-SR destination latches the limiting
                    // event; an SR destination instead receives every CCR
                    // bit from the (already limited) source value below.
                    if (dec_is_movec_rr && movec_rr_dst_reg != 6'h39 &&
                        ((movec_rr_src_reg == 6'h0e && acc_a_bus_word[24]) ||
                         (movec_rr_src_reg == 6'h0f && acc_b_bus_word[24])))
                        sr[6] <= 1'b1;
                    // An ordinary register MOVE uses the same full-A/B data
                    // bus view as memory and control-register moves.  Its
                    // already-limited payload must report that event in L.
                    if (dec_is_parallel && dec_move_type == 4'd1 &&
                        ((dec_move_src_reg == 6'h0e && acc_a_bus_word[24]) ||
                         (dec_move_src_reg == 6'h0f && acc_b_bus_word[24])))
                        sr[6] <= 1'b1;
                    // The independent Class-I register half transfers full
                    // A/B through the same shifter/limiter onto Y0/Y1 (X:R)
                    // or X0/X1 (R:Y).
                    if (dec_is_xr_ry_class1 &&
                        (decode_inst[14] ?
                         (decode_inst[19] ? acc_b_bus_word[24] :
                                            acc_a_bus_word[24]) :
                         (decode_inst[17] ? acc_b_bus_word[24] :
                                            acc_a_bus_word[24])))
                        sr[6] <= 1'b1;
                    // Ordinary X/Y and Class-I primary memory stores use the
                    // pre-ALU full accumulator through the same limiter as
                    // their data bus.  Merge L after ALU CCR writeback so
                    // parallel moves win.
                    if (dec_is_parallel && dec_move_dst_mem &&
                        (dec_move_type == 4'd3 || dec_move_type == 4'd4 ||
                         dec_move_type == 4'd9 || dec_move_type == 4'd10) &&
                        ((dec_move_src_reg == 6'h0e && acc_a_bus_word[24]) ||
                         (dec_move_src_reg == 6'h0f && acc_b_bus_word[24])))
                        sr[6] <= 1'b1;

                    // L: read destinations.  X memory supplies the first
                    // named half and Y memory the second.  A/B destinations
                    // include the documented sign-extension/zero-fill rules.
                    if (dec_is_l_move && l_move_load) begin
                        case (l_move_lll)
                            3'b000: begin
                                acc_a[47:24] <= l_move_fast_internal ?
                                                x_rdata : x_data_hold;
                                acc_a[23:0] <= l_move_fast_internal ?
                                               y_rdata : y_data_hold;
                            end
                            3'b001: begin
                                acc_b[47:24] <= l_move_fast_internal ?
                                                x_rdata : x_data_hold;
                                acc_b[23:0] <= l_move_fast_internal ?
                                               y_rdata : y_data_hold;
                            end
                            3'b010: begin
                                x1_reg <= l_move_fast_internal ?
                                          x_rdata : x_data_hold;
                                x0_reg <= l_move_fast_internal ?
                                          y_rdata : y_data_hold;
                            end
                            3'b011: begin
                                y1_reg <= l_move_fast_internal ?
                                          x_rdata : x_data_hold;
                                y0_reg <= l_move_fast_internal ?
                                          y_rdata : y_data_hold;
                            end
                            3'b100: acc_a <= l_move_fast_internal ?
                                {{8{x_rdata[23]}}, x_rdata, y_rdata} :
                                {{8{x_data_hold[23]}}, x_data_hold, y_data_hold};
                            3'b101: acc_b <= l_move_fast_internal ?
                                {{8{x_rdata[23]}}, x_rdata, y_rdata} :
                                {{8{x_data_hold[23]}}, x_data_hold, y_data_hold};
                            3'b110: begin
                                acc_a <= l_move_fast_internal ?
                                    {{8{x_rdata[23]}}, x_rdata, 24'h0} :
                                    {{8{x_data_hold[23]}}, x_data_hold, 24'h0};
                                acc_b <= l_move_fast_internal ?
                                    {{8{y_rdata[23]}}, y_rdata, 24'h0} :
                                    {{8{y_data_hold[23]}}, y_data_hold, 24'h0};
                            end
                            3'b111: begin
                                acc_b <= l_move_fast_internal ?
                                    {{8{x_rdata[23]}}, x_rdata, 24'h0} :
                                    {{8{x_data_hold[23]}}, x_data_hold, 24'h0};
                                acc_a <= l_move_fast_internal ?
                                    {{8{y_rdata[23]}}, y_rdata, 24'h0} :
                                    {{8{y_data_hold[23]}}, y_data_hold, 24'h0};
                            end
                        endcase
                    end

                    // X:Y has two independent register destinations.  Both
                    // memory values were captured before this edge; duplicate
                    // destinations with the ALU or each other are forbidden
                    // by the architecture and assembler.
                    if (dec_is_xy_move) begin
                        if (!decode_inst[22]) begin
                            case (xy_x_reg)
                                6'h04: x0_reg <= xy_move_fast ? x_rdata :
                                                                   x_data_hold;
                                6'h05: x1_reg <= xy_move_fast ? x_rdata :
                                                                   x_data_hold;
                                6'h0e: acc_a <= xy_move_fast ?
                                    {{8{x_rdata[23]}}, x_rdata, 24'h0} :
                                    {{8{x_data_hold[23]}}, x_data_hold, 24'h0};
                                6'h0f: acc_b <= xy_move_fast ?
                                    {{8{x_rdata[23]}}, x_rdata, 24'h0} :
                                    {{8{x_data_hold[23]}}, x_data_hold, 24'h0};
                                default: ;
                            endcase
                        end
                        if (!decode_inst[15]) begin
                            case (xy_y_reg)
                                6'h06: y0_reg <= xy_move_fast ? y_rdata :
                                                                   y_data_hold;
                                6'h07: y1_reg <= xy_move_fast ? y_rdata :
                                                                   y_data_hold;
                                6'h0e: acc_a <= xy_move_fast ?
                                    {{8{y_rdata[23]}}, y_rdata, 24'h0} :
                                    {{8{y_data_hold[23]}}, y_data_hold, 24'h0};
                                6'h0f: acc_b <= xy_move_fast ?
                                    {{8{y_rdata[23]}}, y_rdata, 24'h0} :
                                    {{8{y_data_hold[23]}}, y_data_hold, 24'h0};
                                default: ;
                            endcase
                        end
                    end

                    // AGU R register update
                    if (agu_r_wr_en && !dec_is_xy_move &&
                        (needs_xy_mem || needs_p_data || dec_move_addr_update ||
                        dec_branch_use_ea) &&
                        !movep_reg_ref &&
                        (!movec_mem_ref || decode_inst[14]) &&
                        (!dec_is_movem || decode_inst[14]) &&
                        (!(dec_move_src_mem || dec_move_dst_mem) ||
                         decode_inst[14] || dec_is_xr_ry_class1 ||
                         dec_is_xr_ry_class2))
                        r_reg[agu_r_wr_idx] <= agu_r_out;
                    if (dec_is_xy_move) begin
                        if (xy_x_r_wr_en)
                            r_reg[xy_x_r_wr_idx] <= xy_x_r_out;
                        if (xy_y_r_wr_en)
                            r_reg[xy_y_r_wr_idx] <= xy_y_r_out;
                    end

                    // REP saves the architectural LC, loads the 16-bit
                    // source count, and repeats the following one-word
                    // instruction without refetching a different PC.  Zero
                    // naturally wraps through 16'hffff for 65,536 total
                    // executions.  LC is restored after the final iteration.
                    if (dec_loop_is_rep) begin
                        rep_active <= 1'b1;
                        rep_fetch_addr <= pc;
                        rep_saved_lc <= lc;
                        rep_illegal_pending <= 1'b0;
                        lc <= {8'h0, rep_source_count};
                        pc <= pc + 16'd1;
                        if (rep_reg_full_acc_source && movep_acc_limit)
                            sr[6] <= 1'b1;
                    end else if (rep_active) begin
                        if (rep_final_iteration) begin
                            rep_active <= 1'b0;
                            lc <= rep_saved_lc;
                            rep_boundary_pending <= 1'b1;
                            rep_illegal_pending <= dec_is_illegal;
                        end else begin
                            lc <= {8'h0, lc[15:0] - 16'd1};
                        end
                    end

                    // NORM: update R counter
                    if (dec_is_norm && alu_norm_did_shift) begin
                        if (alu_norm_shift_dir)
                            r_reg[dec_norm_rrr] <= r_reg[dec_norm_rrr] + 16'd1;
                        else
                            r_reg[dec_norm_rrr] <= r_reg[dec_norm_rrr] - 16'd1;
                    end

                    // Unified move register write
                    if (move_wr_en_c) begin
                        case (move_wr_code_c)
                            6'h04: x0_reg <= move_wr_data_c;
                            6'h05: x1_reg <= move_wr_data_c;
                            6'h06: y0_reg <= move_wr_data_c;
                            6'h07: y1_reg <= move_wr_data_c;
                            6'h08: acc_a[23:0]  <= move_wr_data_c;
                            6'h09: acc_b[23:0]  <= move_wr_data_c;
                            6'h0A: acc_a[55:48] <= move_wr_data_c[7:0];
                            6'h0B: acc_b[55:48] <= move_wr_data_c[7:0];
                            6'h0C: acc_a[47:24] <= move_wr_data_c;
                            6'h0D: acc_b[47:24] <= move_wr_data_c;
                            6'h0E: acc_a <= {{8{move_wr_data_c[23]}}, move_wr_data_c, 24'h0};
                            6'h0F: acc_b <= {{8{move_wr_data_c[23]}}, move_wr_data_c, 24'h0};
                            6'h10: r_reg[0] <= move_wr_data_c[15:0];
                            6'h11: r_reg[1] <= move_wr_data_c[15:0];
                            6'h12: r_reg[2] <= move_wr_data_c[15:0];
                            6'h13: r_reg[3] <= move_wr_data_c[15:0];
                            6'h14: r_reg[4] <= move_wr_data_c[15:0];
                            6'h15: r_reg[5] <= move_wr_data_c[15:0];
                            6'h16: r_reg[6] <= move_wr_data_c[15:0];
                            6'h17: r_reg[7] <= move_wr_data_c[15:0];
                            6'h18: n_reg[0] <= move_wr_data_c[15:0];
                            6'h19: n_reg[1] <= move_wr_data_c[15:0];
                            6'h1A: n_reg[2] <= move_wr_data_c[15:0];
                            6'h1B: n_reg[3] <= move_wr_data_c[15:0];
                            6'h1C: n_reg[4] <= move_wr_data_c[15:0];
                            6'h1D: n_reg[5] <= move_wr_data_c[15:0];
                            6'h1E: n_reg[6] <= move_wr_data_c[15:0];
                            6'h1F: n_reg[7] <= move_wr_data_c[15:0];
                            6'h20: m_reg[0] <= move_wr_data_c[15:0];
                            6'h21: m_reg[1] <= move_wr_data_c[15:0];
                            6'h22: m_reg[2] <= move_wr_data_c[15:0];
                            6'h23: m_reg[3] <= move_wr_data_c[15:0];
                            6'h24: m_reg[4] <= move_wr_data_c[15:0];
                            6'h25: m_reg[5] <= move_wr_data_c[15:0];
                            6'h26: m_reg[6] <= move_wr_data_c[15:0];
                            6'h27: m_reg[7] <= move_wr_data_c[15:0];
                            6'h39: sr  <= move_wr_data_c & SR_ARCH_MASK;
                            6'h3A: omr <= move_wr_data_c[7:0];
                            6'h3B: sp  <= move_wr_data_c[5:0];
                            // Architectural SSH writes preincrement SP.
                            // The zero-based backing array therefore receives
                            // the new top entry at the old numeric SP.
                            6'h3C: begin
                                if (stack_sp_operable) begin
                                    if (stack_sp_base < 6'd15) begin
                                        ss[sp[3:0]][47:40] <= 8'h0;
                                        ss[sp[3:0]][39:24] <=
                                            move_wr_data_c[15:0];
                                    end
                                    sp <= stack_sp_base + 6'd1;
                                end
                            end
                            // SSL writes the low bank of the current top
                            // entry and do not change SP.
                            6'h3D: begin
                                if (stack_sp_operable) begin
                                    ss[(sp[3:0] == 4'd0) ? 4'd0 :
                                       sp[3:0] - 4'd1][23:16] <= 8'h0;
                                    ss[(sp[3:0] == 4'd0) ? 4'd0 :
                                       sp[3:0] - 4'd1][15:0] <=
                                        move_wr_data_c[15:0];
                                end
                            end
                            6'h3E: la  <= move_wr_data_c[15:0];
                            6'h3F: lc  <= {8'h0, move_wr_data_c[15:0]};
                            default: ;
                        endcase
                    end

                    // Class-I performs its independent accumulator-to-input-
                    // register transfer in the same instruction as the X:R or
                    // R:Y primary memory move.  Both sources are sampled before
                    // any parallel ALU writeback.
                    if (dec_is_xr_ry_class1) begin
                        if (!decode_inst[14]) begin
                            if (decode_inst[16])
                                y1_reg <= decode_inst[17] ?
                                          acc_b_bus_word[23:0] :
                                          acc_a_bus_word[23:0];
                            else
                                y0_reg <= decode_inst[17] ?
                                          acc_b_bus_word[23:0] :
                                          acc_a_bus_word[23:0];
                        end else begin
                            if (decode_inst[18])
                                x1_reg <= decode_inst[19] ?
                                          acc_b_bus_word[23:0] :
                                          acc_a_bus_word[23:0];
                            else
                                x0_reg <= decode_inst[19] ?
                                          acc_b_bus_word[23:0] :
                                          acc_a_bus_word[23:0];
                        end
                    end

                    // Class-II X:R/R:Y samples both parallel sources before
                    // this edge: the selected A/B value has already driven the
                    // memory write, while X0/Y0 now replaces that accumulator's
                    // signed high/middle word and clears its low word.  A legal
                    // parallel ALU opcode cannot name the same destination.
                    if (dec_is_xr_ry_class2) begin
                        if (decode_inst[16])
                            acc_b <= {{8{(decode_inst[15] ? y0_reg[23] :
                                         x0_reg[23])}},
                                      (decode_inst[15] ? y0_reg : x0_reg),
                                      24'h0};
                        else
                            acc_a <= {{8{(decode_inst[15] ? y0_reg[23] :
                                         x0_reg[23])}},
                                      (decode_inst[15] ? y0_reg : x0_reg),
                                      24'h0};
                    end

                    // Every architectural SSH source read postdecrements SP.
                    // Normalize a user-forced UF from the low pointer bits;
                    // an already-set SE remains frozen.
                    if (stack_ssh_pop && stack_sp_operable)
                        sp <= stack_sp_base - 6'd1;

                    // Branch execution
                    if (dec_is_branch_op && branch_taken) begin
                        if (dec_branch_push_pc) begin
                            if (stack_sp_operable) begin
                                if (stack_sp_base < 6'd15)
                                    ss[sp[3:0]] <= {8'h0, pc, sr_arch};
                                sp <= stack_sp_base + 6'd1;
                            end
                        end
                        pc <= branch_effective_target;
                    end

                    // Bit-conditioned jumps preserve all condition codes.
                    // JSCLR/JSSET push the post-extension PC/SR only on the
                    // taken path, then share the exact JCLR/JSET redirect.
                    if (dec_is_bit_op) begin
                        if (dec_bit_op_mem &&
                            !dec_is_jclr && !dec_is_jset)
                            sr[0] <= jclr_bit_val;
                        if (!dec_bit_op_mem &&
                            !dec_is_jclr && !dec_is_jset) begin
                            // Register B operations copy the old selected bit
                            // to C before any modifying writeback.
                            // SR is self-referential: its modifying forms
                            // change only the selected SR bit, so C is not a
                            // second destination unless bit zero is selected.
                            if (!(dec_bit_op_addr == 6'h29 &&
                                  dec_bit_op_type != 2'b00))
                                sr[0] <= jclr_bit_val;
                            if (dec_bit_op_type != 2'b00) begin
                                case (dec_bit_op_addr)
                                    6'h04: x0_reg <= bit_reg_result;
                                    6'h05: x1_reg <= bit_reg_result;
                                    6'h06: y0_reg <= bit_reg_result;
                                    6'h07: y1_reg <= bit_reg_result;
                                    6'h08: acc_a[23:0] <= bit_reg_result;
                                    6'h09: acc_b[23:0] <= bit_reg_result;
                                    6'h0a: acc_a[55:48] <=
                                                bit_reg_result[7:0];
                                    6'h0b: acc_b[55:48] <=
                                                bit_reg_result[7:0];
                                    6'h0c: acc_a[47:24] <= bit_reg_result;
                                    6'h0d: acc_b[47:24] <= bit_reg_result;
                                    6'h0e: acc_a[23:0] <= bit_reg_result;
                                    6'h0f: acc_b[23:0] <= bit_reg_result;
                                    6'h29: sr <= bit_reg_result & SR_ARCH_MASK;
                                    6'h2a: omr <= bit_reg_result[7:0];
                                    6'h2b: begin
                                        // UF is latched while SE is set;
                                        // user writes may clear it only from
                                        // the forced UF=1,SE=0 state.
                                        if (!(dec_bit_op_bit == 5'd5 && sp[4]))
                                            sp <= bit_reg_result[5:0];
                                    end
                                    6'h2c: begin
                                        if (stack_sp_operable)
                                            ss[(sp[3:0] == 4'd0) ? 4'd0 :
                                               sp[3:0] - 4'd1][39:24] <=
                                                bit_reg_result[15:0];
                                    end
                                    6'h2d: begin
                                        if (stack_sp_operable)
                                            ss[(sp[3:0] == 4'd0) ? 4'd0 :
                                               sp[3:0] - 4'd1][15:0] <=
                                                bit_reg_result[15:0];
                                    end
                                    6'h2e: la <= bit_reg_result[15:0];
                                    6'h2f: lc <= {8'h0,
                                                  bit_reg_result[15:0]};
                                    default: begin
                                        if (dec_bit_op_addr >= 6'h10 &&
                                            dec_bit_op_addr <= 6'h17)
                                            r_reg[dec_bit_op_addr[2:0]] <=
                                                bit_reg_result[15:0];
                                        else if (dec_bit_op_addr >= 6'h18 &&
                                                 dec_bit_op_addr <= 6'h1f)
                                            n_reg[dec_bit_op_addr[2:0]] <=
                                                bit_reg_result[15:0];
                                        else if (dec_bit_op_addr >= 6'h20 &&
                                                 dec_bit_op_addr <= 6'h27)
                                            m_reg[dec_bit_op_addr[2:0]] <=
                                                bit_reg_result[15:0];
                                    end
                                endcase
                            end
                        end
                        if (taking_js_bit && stack_sp_operable) begin
                            if (stack_sp_base < 6'd15)
                                ss[sp[3:0]] <= {8'h0, pc, sr_arch};
                            sp <= stack_sp_base + 6'd1;
                        end
                        if (taking_jclr || taking_jset)
                            pc <= dec_branch_target[15:0];
                    end

                    // RTS
                    if (dec_is_rts) begin
                        if (stack_sp_operable) begin
                            if (stack_sp_base != 6'd0)
                                pc <= ss[sp[3:0] - 4'd1][39:24];
                            sp <= stack_sp_base - 6'd1;
                        end
                    end

                    // RTI
                    if (dec_is_rti) begin
                        if (stack_sp_operable) begin
                            if (stack_sp_base != 6'd0) begin
                                pc <= ss[sp[3:0] - 4'd1][39:24];
                                sr <= ss[sp[3:0] - 4'd1][23:0] &
                                      SR_ARCH_MASK;
                            end
                            sp <= stack_sp_base - 6'd1;
                        end
                    end

                    // ANDI/ORI
                    if (dec_is_andi_ori) begin
                        case (dec_andi_ori_target)
                            2'b00: begin
                                if (dec_andi_ori_is_and)
                                    sr[15:8] <= (sr[15:8] &
                                                  dec_andi_ori_imm) & 8'haf;
                                else
                                    sr[15:8] <= (sr[15:8] |
                                                  dec_andi_ori_imm) & 8'haf;
                            end
                            2'b01: begin
                                if (dec_andi_ori_is_and)
                                    sr[7:0] <= {1'b0,
                                        sr[6:0] & dec_andi_ori_imm[6:0]};
                                else
                                    sr[7:0] <= {1'b0,
                                        sr[6:0] | dec_andi_ori_imm[6:0]};
                            end
                            2'b10: begin
                                if (dec_andi_ori_is_and)
                                    omr <= omr & dec_andi_ori_imm;
                                else
                                    omr <= omr | dec_andi_ori_imm;
                            end
                            default: ;
                        endcase
                    end

                    // DO loop initialization
                    if (dec_loop_is_do) begin
                        if (stack_sp_operable) begin
                            if (stack_sp_base <= 6'd13) begin
                                // First entry: prior LA/LC.  Second entry:
                                // loop start PC and prior SR, including LF.
                                ss[sp[3:0]] <= {8'h0, la, lc};
                                ss[sp[3:0] + 4'd1] <= {8'h0, pc, sr_arch};
                                la <= decode_ext[15:0];
                                lc <= {8'h0, do_source_count};
                                sr[15] <= 1'b1;
                                if (do_reg_full_acc_source && movep_acc_limit)
                                    sr[6] <= 1'b1;
                            end
                            sp <= stack_sp_base + 6'd2;
                        end
                    end

                    // ENDDO
                    if (dec_loop_is_enddo) begin
                        if (stack_sp_operable) begin
                            if (stack_sp_base >= 6'd2) begin
                                // Restore LF only from the loop PC/SR entry,
                                // then LA/LC from the entry beneath it.
                                sr[15] <= ss[sp[3:0] - 4'd1][15];
                                la <= ss[sp[3:0] - 4'd2][39:24];
                                lc <= ss[sp[3:0] - 4'd2][23:0];
                            end
                            sp <= stack_sp_base - 6'd2;
                        end
                    end

                    // Loop end detection
                    if (!rep_active && !dec_loop_is_rep &&
                        at_loop_end && !loop_last && !dec_loop_is_enddo) begin
                        lc <= {8'h0, lc[15:0] - 16'd1};
                        pc <= loop_start_addr;
                    end
                    if (!rep_active && !dec_loop_is_rep &&
                        loop_last && !dec_loop_is_enddo) begin
                        if (stack_sp_operable) begin
                            if (stack_sp_base >= 6'd2) begin
                                sr[15] <= ss[sp[3:0] - 4'd1][15];
                                la <= ss[sp[3:0] - 4'd2][39:24];
                                lc <= ss[sp[3:0] - 4'd2][23:0];
                            end
                            sp <= stack_sp_base - 6'd2;
                        end
                    end
                end

                // ============================================================
                // S_DECODE_EXEC: Latch instruction, decode, present data addrs
                // ============================================================
                S_DECODE_EXEC: begin
                    if (data_issue_wait) begin
                        // Preserve the exact instruction and PC until every
                        // asserted X/Y or shared-P request has been accepted.
                    end else if (rep_boundary_illegal_accept) begin
                        irq_vector_active <= 1'b1;
                        irq_vector_second_slot <= 1'b0;
                        irq_vector_discard_second <= 1'b0;
                        irq_vector_base <= 16'h003e;
                        irq_vector_level <= 2'b11;
                        rep_boundary_pending <= 1'b0;
                        rep_illegal_pending <= 1'b0;
                    end else if (rep_boundary_trace_start) begin
                        trace_flush_active <= 1'b1;
                        trace_flush_steps <= 3'd6;
                        rep_boundary_pending <= 1'b0;
                        rep_illegal_pending <= 1'b0;
                    end else if (rep_boundary_irq_accept) begin
                        irq_vector_active <= 1'b1;
                        irq_vector_second_slot <= 1'b0;
                        irq_vector_discard_second <= 1'b0;
                        irq_vector_base <= irq_accept_vector;
                        irq_vector_level <= irq_accept_level;
                        rep_boundary_pending <= 1'b0;
                        rep_illegal_pending <= 1'b0;
                    end else if (fetching_ext) begin
                        // Latching extension word (arrived from P-memory)
                        ext_word <= p_fetch_data;
                        fetching_ext <= 1'b0;
                        pc <= pc + 16'd1;
                    end else begin
                        // Latching first instruction word
                        cur_inst <= p_fetch_data;
                        if (!irq_vector_active && !rep_active)
                            pc <= pc + 16'd1;
                        if (dec_is_two_word) begin
                            // Need extension word - will fetch in next clock
                            // PC already incremented, points to ext word
                        end
                        // Decoding a consecutive REP transfers the exclusion
                        // to that new package.  Its own setup WB is blocked by
                        // dec_loop_is_rep, its followers by rep_active, and its
                        // final follower will establish the next boundary.
                        if (rep_boundary_pending)
                            rep_boundary_pending <= 1'b0;
                        if (rep_boundary_pending)
                            rep_illegal_pending <= 1'b0;
                        if (dec_is_swi)
                            swi_wait_steps <= 3'd6;
                    end
                end

                S_REP_WAIT1: begin
                    if (dec_is_swi && swi_wait_steps != 3'd0)
                        swi_wait_steps <= swi_wait_steps - 3'd1;
                end

                // ============================================================
                // S_LATCH_EXT: Latch extension word from P-memory
                // ============================================================
                S_LATCH_EXT: begin
                    if (!data_issue_wait) begin
                        ext_word <= p_fetch_data;
                        if (!irq_vector_active)
                            pc <= pc + 16'd1;
                    end
                end

                // ============================================================
                // S_MEM: Wait for memory response (block RAM latency)
                // ============================================================
                S_MEM: begin
                    // Capture all memory responses before S_WB_FETCH changes
                    // the decoder selection or reclaims P for instruction
                    // fetch.  Internal/peripheral values are harmlessly held
                    // by the same rule; only the selected source is consumed.
                    if (needs_p_data)
                        p_data_hold <= p_data_rdata;
                    // MOVEP peripheral-to-P is classified as a P-memory
                    // instruction but still sources X/Y peripheral data.
                    if (needs_mem) begin
                        x_data_hold <= x_rdata;
                        if (!(dec_is_l_move && l_move_external) &&
                            (!dec_is_xy_move || !xy_both_external))
                            y_data_hold <= y_rdata;
                    end
                end

                S_L_MEM_Y_REQ: begin
                    // Fast shared-port L:/X:Y paths arrive directly from
                    // decode; retain the registered X response while the Y
                    // request is issued on this same clock.
                    if ((dec_is_l_move && l_move_external) ||
                        (dec_is_xy_move && xy_both_external)) begin
                        // Once the serialized Y response is accepted the
                        // shared SRAM read-data bus contains Y, not X.  Keep
                        // the X half captured on the original Y request.
                        if (!y_response_complete)
                            x_data_hold <= x_rdata;
                    end
                end

                S_L_MEM_Y_WB: begin
                    if ((dec_is_l_move && l_move_load) ||
                        (dec_is_xy_move && xy_both_external &&
                         !decode_inst[15]))
                        y_data_hold <= y_rdata;
                end

                S_IRQ_FETCH: begin
                    // When slot one changed program flow, the response being
                    // discarded in this state is vector slot two.  The fetch
                    // issued here supplies the branch target to the ordinary
                    // decode state on the following cycle.
                    if (irq_vector_discard_second)
                        irq_vector_discard_second <= 1'b0;
                end

                S_HALT: begin
                    if (trace_flush_active) begin
                        if (trace_flush_steps > 3'd1)
                            trace_flush_steps <= trace_flush_steps - 3'd1;
                        else begin
                            trace_flush_active <= 1'b0;
                            trace_flush_steps <= 3'd0;
                            irq_vector_active <= 1'b1;
                            irq_vector_second_slot <= 1'b0;
                            irq_vector_discard_second <= 1'b0;
                            irq_vector_base <= 16'h0004;
                            irq_vector_level <= 2'b11;
                        end
                    end else if (stop_active) begin
                        if (!stop_recovering && irq_a) begin
                            stop_recovering <= 1'b1;
                            stop_recovery_steps <= stop_recovery_period;
                        end else if (stop_recovering) begin
                            if (stop_recovery_steps > 17'd1) begin
                                stop_recovery_steps <=
                                    stop_recovery_steps - 17'd1;
                            end else begin
                                stop_active <= 1'b0;
                                stop_recovering <= 1'b0;
                                stop_recovery_steps <= 17'd0;
                                if (stop_trace_pending) begin
                                    // Trace is nonmaskable and outranks the
                                    // IRQA level that restarted the clock.
                                    // Preserve the ordinary three-instruction
                                    // trace pipeline-flush interval.
                                    stop_trace_pending <= 1'b0;
                                    trace_flush_active <= 1'b1;
                                    trace_flush_steps <= 3'd6;
                                end else if (stop_irq_accept) begin
                                    irq_vector_active <= 1'b1;
                                    irq_vector_second_slot <= 1'b0;
                                    irq_vector_discard_second <= 1'b0;
                                    irq_vector_base <= stop_accept_vector;
                                    irq_vector_level <= stop_accept_level;
                                end
                            end
                        end
                    // Peripherals and edge capture keep running in WAIT.
                    // Once an unmasked request is serviceable, begin its
                    // vector package before refetching the successor.
                    end else if (wait_active) begin
                        if (wait_pending_steps != 4'd0) begin
                            wait_pending_steps <= wait_pending_steps - 4'd1;
                        end else if (wait_irq_accept) begin
                            wait_active <= 1'b0;
                            irq_vector_active <= 1'b1;
                            irq_vector_second_slot <= 1'b0;
                            irq_vector_discard_second <= 1'b0;
                            irq_vector_base <= irq_accept_vector;
                            irq_vector_level <= irq_accept_level;
                        end
                    end
                end

                default: ;
            endcase
            end
        end
    end

endmodule
