// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

//////////////////////////////////////////////////////////////////////////////
// DSP56001 X-Peripheral Space Decoder
//
// Standard DSP56001 peripherals at X:$FFC0-$FFFF:
//   $FFE0: PBC    - Port B Control
//   $FFE1: PCC    - Port C Control (SSI)
//   $FFE2: PBDDR  - Port B Data Direction
//   $FFE3: PCDDR  - Port C Data Direction
//   $FFE4: PBD    - Port B Data
//   $FFE5: PCD    - Port C Data
//   $FFEC: CRA    - SSI Control Register A
//   $FFED: CRB    - SSI Control Register B
//   $FFEE: SSISR  - SSI Status Register
//   $FFEF: TX/RX  - SSI Transmit/Receive Data
//   $FFF0: SCR    - SCI Control Register
//   $FFF1: SSR    - SCI Status Register
//   $FFF2: SCCR   - SCI Clock Control Register
//   $FFF3: STXA   - SCI Transmit Address Data
//   $FFF4-$FFF6: SRX/STX byte aliases
//   $FFFE: BCR    - Bus Control Register (wait states)
//   $FFFF: IPR    - Interrupt Priority Register
//
// The SSI boundary is word-oriented because the IP20 audio glue supplies the
// external serial clock and frame sync.  ssi_tx_ready represents one complete
// externally clocked transmit time slot; ssi_tx_en marks a data word accepted
// for that slot.  This preserves the DSP56001 TX/TDE/interrupt lifecycle while
// leaving physical serialization to the board audio block.
//////////////////////////////////////////////////////////////////////////////

module dsp56001_periph (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clock_enable,      // DSP oscillator-clock step
    input  wire        soft_reset,
    input  wire        stop_reset,

    // ========================================================================
    // Peripheral Bus Interface (from memory subsystem)
    // ========================================================================
    input  wire [5:0]  addr,              // 6-bit address ($FFC0 base = addr 0)
    input  wire [23:0] wdata,
    output reg  [23:0] rdata,
    input  wire        rd_en,
    input  wire        wr_en,

    // ========================================================================
    // SSI Interface (directly exposed for audio)
    // ========================================================================
    output reg         ssi_tx_en,         // SSI transmit word accepted
    output wire [23:0] ssi_tx_data,       // SSI transmit data
    input  wire        ssi_tx_ready,      // External glue requests one word
    input  wire        ssi_rx_valid,      // SSI receive data valid
    input  wire [23:0] ssi_rx_data,       // SSI receive data
    output wire        ssi_sclk_out,      // SSI serial clock
    output wire        ssi_fss_out,       // SSI frame sync

    // ========================================================================
    // SCI Character Interface (to the physical serial shifter)
    // ========================================================================
    output reg         sci_tx_valid,      // STX handed to an idle shifter
    output wire [7:0]  sci_tx_data,
    output wire        sci_tx_address,    // Multidrop address/data bit
    input  wire        sci_tx_ready,      // Shifter can accept one character
    input  wire        sci_tx_done,       // Accepted character fully shifted
    input  wire        sci_rx_valid,      // One complete received character
    input  wire [7:0]  sci_rx_data,
    input  wire        sci_rx_r8,
    input  wire        sci_rx_framing_error,
    input  wire        sci_rx_parity_error,
    input  wire        sci_rx_idle,       // Qualified 10/11-one idle detection

    // ========================================================================
    // Interrupt Outputs
    // ========================================================================
    output wire        irq_ssi_tx,        // SSI TX interrupt
    output wire        irq_ssi_rx,        // SSI RX interrupt
    output wire        irq_ssi_tx_exception,
    output wire        irq_ssi_rx_exception,
    output wire        irq_sci_tx,        // SCI TX interrupt
    output wire        irq_sci_rx,        // SCI RX interrupt
    output wire        irq_sci_rx_exception,
    output wire        irq_sci_idle,
    output wire        irq_sci_timer,     // SCI periodic timer interrupt
    input  wire        sci_idle_ack,      // P:$001a vector-package complete
    input  wire        sci_timer_ack,     // P:$001c vector-package complete

    // Full DSP56001 IPR.  External IRQA/IRQB occupy bits 2:0/5:3; SSI IPL is
    // bits 13:12 and SCI IPL is bits 15:14.
    output wire [15:0] irq_priority,
    output wire [15:0] bus_wait_states
);

    // ========================================================================
    // Register Storage
    // ========================================================================
    // Address offsets from $FFC0 base
    localparam A_PBC   = 6'h20;  // $FFE0 - $FFC0 = 0x20
    localparam A_PCC   = 6'h21;  // $FFE1
    localparam A_PBDDR = 6'h22;  // $FFE2
    localparam A_PCDDR = 6'h23;  // $FFE3
    localparam A_PBD   = 6'h24;  // $FFE4
    localparam A_PCD   = 6'h25;  // $FFE5
    localparam A_CRA   = 6'h2C;  // $FFEC
    localparam A_CRB   = 6'h2D;  // $FFED
    localparam A_SSISR = 6'h2E;  // $FFEE
    localparam A_TXRX  = 6'h2F;  // $FFEF
    localparam A_SCR   = 6'h30;  // $FFF0
    localparam A_SSR   = 6'h31;  // $FFF1
    localparam A_SCCR  = 6'h32;  // $FFF2
    localparam A_STXA  = 6'h33;  // $FFF3
    localparam A_STXL  = 6'h34;  // $FFF4
    localparam A_STXM  = 6'h35;  // $FFF5
    localparam A_STXH  = 6'h36;  // $FFF6
    localparam A_BCR   = 6'h3E;  // $FFFE
    localparam A_IPR   = 6'h3F;  // $FFFF

    reg [23:0] pbc;
    reg [23:0] pcc;
    reg [23:0] pbddr;
    reg [23:0] pcddr;
    reg [23:0] pbd;
    reg [23:0] pcd;
    reg [23:0] cra;
    reg [23:0] crb;
    reg [23:0] ssisr;
    reg [23:0] tx_reg;
    reg [23:0] rx_reg;
    reg [23:0] scr;
    reg [23:0] sccr;
    reg [7:0]  sci_ssr;
    reg [7:0]  sci_stx;
    reg [7:0]  sci_srx;
    reg        sci_stx_address;
    reg [23:0] bcr;
    reg [23:0] ipr;
    reg        tx_timeslot_disabled;
    reg        tue_clear_armed;
    reg        roe_clear_armed;
    reg        sci_icyc_phase;
    reg [11:0] sci_cd_counter;
    reg [2:0]  sci_prescale_counter;
    reg [4:0]  sci_timer_divider;
    reg        sci_timer_pending;
    reg        sci_tx_busy;
    reg        sci_error_clear_armed;
    reg        sci_idle_armed;
    reg        sci_idle_pending;

    wire ssi_pins_enabled = |pcc[8:3];
    wire ssi_tx_enabled = ssi_pins_enabled && crb[12];
    wire ssi_rx_enabled = ssi_pins_enabled && crb[13];
    wire ssi_tx_underrun_event = ssi_tx_ready && ssi_tx_enabled &&
                                  !tx_timeslot_disabled && ssisr[6];
    wire ssi_rx_word_event = ssi_rx_valid && ssi_rx_enabled && !ssisr[7];
    wire ssi_rx_overrun_event = ssi_rx_valid && ssi_rx_enabled && ssisr[7];
    wire sci_pins_enabled = |pcc[2:0];
    wire sci_asynchronous_mode = scr[2:0] != 3'b000;
    wire sci_parity_mode = scr[2:1] == 2'b10;
    wire sci_receiver_sleeping = scr[6] && sci_asynchronous_mode;
    wire sci_address_wakeup = (scr[2:0] == 3'b110) ? sci_rx_r8 :
                                                          sci_rx_data[7];
    wire sci_rx_transfer_enabled = sci_rx_valid && scr[8] &&
                                   (!sci_receiver_sleeping ||
                                    (scr[5] && sci_address_wakeup)) &&
                                   !sci_ssr[6];
    wire sci_rx_word_event = sci_rx_transfer_enabled && !sci_ssr[2];
    wire sci_rx_overrun_event = sci_rx_transfer_enabled && sci_ssr[2];

    // ========================================================================
    // Register Write
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pbc   <= 24'h0;
            pcc   <= 24'h0;
            pbddr <= 24'h0;
            pcddr <= 24'h0;
            pbd   <= 24'h0;
            pcd   <= 24'h0;
            cra   <= 24'h0;
            crb   <= 24'h0;
            ssisr <= 24'h000040;  // TDE=1 (transmit data empty) on reset
            tx_reg <= 24'h0;
            rx_reg <= 24'h0;
            scr   <= 24'h0;
            sccr  <= 24'h0;
            sci_ssr <= 8'h03; // TRNE=1, TDRE=1
            sci_stx <= 8'h0;
            sci_srx <= 8'h0;
            sci_stx_address <= 1'b0;
            bcr   <= 24'hFFFF;   // Maximum wait states on reset
            ipr   <= 24'h0;
            ssi_tx_en <= 1'b0;
            sci_tx_valid <= 1'b0;
            tx_timeslot_disabled <= 1'b0;
            tue_clear_armed <= 1'b0;
            roe_clear_armed <= 1'b0;
            sci_icyc_phase <= 1'b0;
            sci_cd_counter <= 12'h0;
            sci_prescale_counter <= 3'h0;
            sci_timer_divider <= 5'h0;
            sci_timer_pending <= 1'b0;
            sci_tx_busy <= 1'b0;
            sci_error_clear_armed <= 1'b0;
            sci_idle_armed <= 1'b0;
            sci_idle_pending <= 1'b0;
        end else if (soft_reset) begin
            // RESET instruction: reset on-chip peripheral control/status and
            // IPR without resetting the processor or external HPC1 host.
            // Motorola specifies that SSI TX/RX data registers are unchanged.
            pbc   <= 24'h0;
            pcc   <= 24'h0;
            pbddr <= 24'h0;
            pcddr <= 24'h0;
            pbd   <= 24'h0;
            pcd   <= 24'h0;
            cra   <= 24'h0;
            crb   <= 24'h0;
            ssisr <= 24'h000040;
            scr   <= 24'h0;
            sccr  <= 24'h0;
            sci_ssr <= 8'h03;
            bcr   <= 24'hFFFF;
            ipr   <= 24'h0;
            ssi_tx_en <= 1'b0;
            sci_tx_valid <= 1'b0;
            tx_timeslot_disabled <= 1'b0;
            tue_clear_armed <= 1'b0;
            roe_clear_armed <= 1'b0;
            sci_icyc_phase <= 1'b0;
            sci_cd_counter <= 12'h0;
            sci_prescale_counter <= 3'h0;
            sci_timer_divider <= 5'h0;
            sci_timer_pending <= 1'b0;
            sci_tx_busy <= 1'b0;
            sci_error_clear_armed <= 1'b0;
            sci_idle_armed <= 1'b0;
            sci_idle_pending <= 1'b0;
        end else if (stop_reset) begin
            // STOP holds SSI/SCI/HI in individual reset.  Their control
            // registers, IPR, BCR, and TX/RX data are preserved; implemented
            // status returns to the documented empty/not-full state.
            ssisr <= 24'h000040;
            sci_ssr <= 8'h03;
            ssi_tx_en <= 1'b0;
            sci_tx_valid <= 1'b0;
            tx_timeslot_disabled <= 1'b0;
            tue_clear_armed <= 1'b0;
            roe_clear_armed <= 1'b0;
            sci_icyc_phase <= 1'b0;
            sci_cd_counter <= sccr[11:0];
            sci_prescale_counter <= 3'h0;
            sci_timer_divider <= 5'h0;
            sci_timer_pending <= 1'b0;
            sci_tx_busy <= 1'b0;
            sci_error_clear_armed <= 1'b0;
            sci_idle_armed <= 1'b0;
            sci_idle_pending <= 1'b0;
        end else begin
            ssi_tx_en <= 1'b0;
            sci_tx_valid <= 1'b0;

            // The SCI periodic timer starts with fosc/2 (Icyc), then divides
            // by CD+1, optionally by eight, by two and by sixteen.  One
            // clock_enable is one fosc period, so CD=0/SCP=0 expires every
            // 64 enabled clocks.  The timer remains available while all SCI
            // pins are GPIO, as required by section 11.2.9.
            if (sci_timer_ack || !scr[13])
                sci_timer_pending <= 1'b0;
            if (clock_enable) begin
                sci_icyc_phase <= ~sci_icyc_phase;
                if (sci_icyc_phase) begin
                    if (sci_cd_counter != 12'h0)
                        sci_cd_counter <= sci_cd_counter - 12'h1;
                    else begin
                        sci_cd_counter <= sccr[11:0];
                        if (sccr[13] &&
                            (sci_prescale_counter != 3'h7))
                            sci_prescale_counter <=
                                sci_prescale_counter + 3'h1;
                        else begin
                            sci_prescale_counter <= 3'h0;
                            if (sci_timer_divider != 5'h1f)
                                sci_timer_divider <=
                                    sci_timer_divider + 5'h1;
                            else begin
                                sci_timer_divider <= 5'h0;
                                if (scr[13])
                                    sci_timer_pending <= 1'b1;
                            end
                        end
                    end
                end
            end

            // Configuring all six Port C SSI pins as GPIO holds the SSI in
            // individual reset.  Control and data registers are retained;
            // only status and in-flight serial state reset.
            if (!ssi_pins_enabled) begin
                ssisr <= 24'h000040;
                tx_timeslot_disabled <= 1'b0;
                tue_clear_armed <= 1'b0;
                roe_clear_armed <= 1'b0;
            end else begin
                // The external IP20 audio glue supplies clock/frame timing.
                // One ready pulse completes a whole SSI time slot.
                if (ssi_tx_ready && ssi_tx_enabled) begin
                    if (tx_timeslot_disabled) begin
                        tx_timeslot_disabled <= 1'b0;
                        ssisr[6] <= 1'b1;
                    end else if (!ssisr[6]) begin
                        ssi_tx_en <= 1'b1;
                        ssisr[6] <= 1'b1;
                        ssisr[2] <= 1'b1; // TFS: sole normal-mode slot
                    end else begin
                        // Empty TX retransmits its previous word and records
                        // the architectural underrun exception status.
                        ssi_tx_en <= 1'b1;
                        ssisr[4] <= 1'b1;
                        ssisr[2] <= 1'b1;
                    end
                end

                // Completed receive words transfer only when the receiver is
                // enabled.  An overrun preserves the unread RX word.
                if (ssi_rx_valid && ssi_rx_enabled) begin
                    if (ssisr[7]) begin
                        ssisr[5] <= 1'b1;
                    end else begin
                        rx_reg <= ssi_rx_data;
                        ssisr[7] <= 1'b1;
                        ssisr[3] <= 1'b1; // RFS: sole normal-mode slot
                    end
                end
            end

            // PCC[2:0]=000 holds the SCI in individual reset.  Motorola
            // preserves control/data registers but restores the status flags
            // to transmitter-empty/not-full for individual and STOP resets.
            if (!sci_pins_enabled) begin
                sci_ssr <= 8'h03;
                sci_tx_busy <= 1'b0;
                sci_error_clear_armed <= 1'b0;
                sci_idle_armed <= 1'b0;
                sci_idle_pending <= 1'b0;
            end else begin
                // STX and the physical shift register are independently
                // buffered.  Handoff makes STX empty (TDRE) while TRNE stays
                // low until the accepted character has completely shifted.
                if (!sci_tx_busy && !sci_ssr[1] && sci_tx_ready && scr[9]) begin
                    sci_tx_valid <= 1'b1;
                    sci_tx_busy <= 1'b1;
                    sci_ssr[1] <= 1'b1;
                end
                if (sci_tx_done && sci_tx_busy) begin
                    sci_tx_busy <= 1'b0;
                    if (sci_ssr[1])
                        sci_ssr[0] <= 1'b1;
                end

                // A valid start/character clears the live IDLE indication
                // and arms exactly one later idle request.  A receive overrun
                // preserves the unread SRX word and overrides FE/PE.
                if (sci_rx_valid && scr[8]) begin
                    sci_ssr[3] <= 1'b0;
                    sci_idle_pending <= 1'b0;
                    // In asynchronous wakeup mode, receiver data and receive
                    // interrupts are suppressed while RWU is set.  Address
                    // wakeup accepts the qualifying address character itself:
                    // R8 is the address bit in multidrop mode, while D7 is
                    // used by the other asynchronous formats.
                    if (!sci_receiver_sleeping ||
                        (scr[5] && sci_address_wakeup)) begin
                        if (sci_receiver_sleeping)
                            scr[6] <= 1'b0;
                        sci_idle_armed <= 1'b1;
                        if (!sci_ssr[6]) begin
                            if (sci_ssr[2]) begin
                                sci_ssr[4] <= 1'b1;
                                sci_ssr[6:5] <= 2'b00;
                            end else begin
                                sci_srx <= sci_rx_data;
                                sci_ssr[7] <= sci_rx_r8;
                                // FE is defined only for asynchronous
                                // formats.  PE is defined only for the even/
                                // odd parity formats (WDS=4/5); synchronous,
                                // 10-bit and multidrop operation must leave
                                // it clear regardless of the raw shifter
                                // indication presented at this boundary.
                                sci_ssr[6] <= sci_asynchronous_mode &&
                                              sci_rx_framing_error;
                                sci_ssr[5] <= sci_ssr[5] |
                                              (sci_parity_mode &&
                                               sci_rx_parity_error);
                                sci_ssr[2] <= 1'b1;
                            end
                        end
                    end
                end
                if (sci_idle_ack)
                    sci_idle_pending <= 1'b0;
                if (sci_rx_idle) begin
                    if (sci_receiver_sleeping && !scr[5]) begin
                        // Idle-line wakeup clears RWU, but the interrupt that
                        // would otherwise describe this same idle interval is
                        // suppressed while the receiver is asleep.
                        scr[6] <= 1'b0;
                        sci_ssr[3] <= 1'b1;
                        sci_idle_pending <= 1'b0;
                        sci_idle_armed <= 1'b0;
                    end else if (sci_idle_armed) begin
                        sci_ssr[3] <= 1'b1;
                        sci_idle_pending <= 1'b1;
                        sci_idle_armed <= 1'b0;
                    end
                end
            end

            if (wr_en) begin
                case (addr)
                    // Port B has one control bit and fifteen GPIO pins;
                    // absent/reserved X-data-bus bits read zero.
                    A_PBC:   pbc   <= wdata & 24'h000001;
                    // Port C has nine physical pins.  PCC, PCDDR and PCD
                    // therefore expose only bits 8:0 on the 24-bit X bus.
                    A_PCC:   pcc   <= wdata & 24'h0001FF;
                    A_PBDDR: pbddr <= wdata & 24'h007FFF;
                    A_PCDDR: pcddr <= wdata & 24'h0001FF;
                    A_PBD:   pbd   <= wdata & 24'h007FFF;
                    A_PCD:   pcd   <= wdata & 24'h0001FF;
                    // CRA and CRB are architectural 16-bit registers on the
                    // 24-bit X data bus; absent high-order bits read zero.
                    A_CRA:   cra   <= wdata & 24'h00FFFF;
                    A_CRB:   crb   <= wdata & 24'h00FFFF;
                    A_TXRX: begin
                        tx_reg <= wdata;
                        // TDR itself is retained and remains writable across
                        // SSI individual reset, but every SSISR bit must stay
                        // at its reset value while all six SSI pins are GPIO.
                        if (ssi_pins_enabled) begin
                            ssisr[6] <= 1'b0;  // Clear TDE on write
                            tx_timeslot_disabled <= 1'b0;
                            // A fresh same-clock underrun is newer than this
                            // armed two-access clear and must remain pending.
                            if (tue_clear_armed && !ssi_tx_underrun_event)
                                ssisr[4] <= 1'b0;
                            tue_clear_armed <= 1'b0;
                        end
                    end
                    // SSISR and TSR share the address: reads return status;
                    // writes suppress data for the following network slot.
                    A_SSISR: if (ssi_pins_enabled) begin
                        ssisr[6] <= 1'b0;
                        tx_timeslot_disabled <= 1'b1;
                        if (tue_clear_armed && !ssi_tx_underrun_event)
                            ssisr[4] <= 1'b0;
                        tue_clear_armed <= 1'b0;
                    end
                    // SCR is a 16-bit register with bit 14 reserved.  SCCR is
                    // a fully defined 16-bit register; upper data-bus bits do
                    // not exist in either architectural register.
                    A_SCR:   scr   <= wdata & 24'h00BFFF;
                    A_SCCR:  sccr  <= wdata & 24'h00FFFF;
                    // STX is one byte decoded at three packing aliases.  STXA
                    // uses the low byte and marks a multidrop address word.
                    // This programming-model slice intentionally does not yet
                    // claim serial shift timing; every accepted write still
                    // performs the architectural TDRE transition.
                    A_STXA: if (sci_pins_enabled) begin
                        sci_stx <= wdata[7:0];
                        sci_stx_address <= 1'b1;
                        sci_ssr[1] <= 1'b0;
                        sci_ssr[0] <= 1'b0;
                    end
                    A_STXL: if (sci_pins_enabled) begin
                        sci_stx <= wdata[7:0];
                        sci_stx_address <= 1'b0;
                        sci_ssr[1] <= 1'b0;
                        sci_ssr[0] <= 1'b0;
                    end
                    A_STXM: if (sci_pins_enabled) begin
                        sci_stx <= wdata[15:8];
                        sci_stx_address <= 1'b0;
                        sci_ssr[1] <= 1'b0;
                        sci_ssr[0] <= 1'b0;
                    end
                    A_STXH: if (sci_pins_enabled) begin
                        sci_stx <= wdata[23:16];
                        sci_stx_address <= 1'b0;
                        sci_ssr[1] <= 1'b0;
                        sci_ssr[0] <= 1'b0;
                    end
                    // BCR is four 4-bit wait-state fields; bits 23:16 do not exist.
                    A_BCR:   bcr   <= wdata & 24'h00FFFF;
                    // IPR bits 9:6 are reserved and read as zero (DSP56001 UM).
                    A_IPR:   ipr   <= wdata & 24'h00FC3F;
                    default: ;
                endcase
            end

            // Exception flags require the Motorola two-access clear sequence:
            // read SSISR, then write TX/TSR for TUE or read RX for ROE.
            if (rd_en && addr == A_SSISR) begin
                if (ssisr[4])
                    tue_clear_armed <= 1'b1;
                if (ssisr[5])
                    roe_clear_armed <= 1'b1;
            end
            if (rd_en && addr == A_TXRX) begin
                // Reads consume the prior RX word.  An arriving word on an
                // empty register is a new RDF event and survives the read;
                // an overrun against a full register likewise survives an
                // armed ROE clear.
                if (!ssi_rx_word_event)
                    ssisr[7] <= 1'b0;
                if (roe_clear_armed && !ssi_rx_overrun_event)
                    ssisr[5] <= 1'b0;
                roe_clear_armed <= 1'b0;
            end

            // SCI receive errors use the same documented two-access form:
            // read SSR, then read any SRX byte alias.  SRX read always clears
            // RDRF; error flags clear only when the status read came first.
            if (rd_en && addr == A_SSR && |sci_ssr[6:4])
                sci_error_clear_armed <= 1'b1;
            if (rd_en && (addr == A_STXL || addr == A_STXM ||
                          addr == A_STXH)) begin
                // A completed character on this edge is newer than the SRX
                // read and must remain advertised.  Likewise, a fresh
                // overrun must survive the older SSR-then-SRX clear sequence.
                if (!sci_rx_word_event)
                    sci_ssr[2] <= 1'b0;
                if (sci_error_clear_armed) begin
                    // The SSR read arms a clear of the previously observed
                    // FE/PE state.  A newly accepted character on this SRX
                    // read edge is newer state: replace the old flags with
                    // that character's format-qualified errors instead of
                    // erasing them with the armed clear.
                    if (sci_rx_word_event) begin
                        sci_ssr[6] <= sci_asynchronous_mode &&
                                      sci_rx_framing_error;
                        sci_ssr[5] <= sci_parity_mode &&
                                      sci_rx_parity_error;
                    end else begin
                        sci_ssr[6:5] <= 2'b00;
                    end
                    if (!sci_rx_overrun_event)
                        sci_ssr[4] <= 1'b0;
                end
                sci_error_clear_armed <= 1'b0;
            end
        end
    end

    // ========================================================================
    // Register Read
    // ========================================================================
    always @(*) begin
        rdata = 24'h0;
        case (addr)
            A_PBC:   rdata = pbc;
            A_PCC:   rdata = pcc;
            A_PBDDR: rdata = pbddr;
            A_PCDDR: rdata = pcddr;
            A_PBD:   rdata = pbd;
            // The KCU105 product boundary does not expose Port C GPIO input
            // pins.  Those inputs are therefore low; output-configured bits
            // return their output-data latches as required by section 11.1.
            A_PCD:   rdata = pcd & pcddr;
            A_CRA:   rdata = cra;
            A_CRB:   rdata = crb;
            A_SSISR: rdata = ssisr;
            A_TXRX:  rdata = rx_reg;  // Read returns RX data
            A_SCR:   rdata = scr;
            A_SSR:   rdata = {16'h0, sci_ssr};
            A_SCCR:  rdata = sccr;
            A_STXL:  rdata = {16'h0, sci_srx};
            A_STXM:  rdata = {8'h0, sci_srx, 8'h0};
            A_STXH:  rdata = {sci_srx, 16'h0};
            A_BCR:   rdata = bcr;
            A_IPR:   rdata = ipr;
            default: rdata = 24'h0;
        endcase
    end

    // ========================================================================
    // SSI Output
    // ========================================================================
    assign ssi_tx_data = tx_reg;
    // The board integration uses external clock and frame sync
    // (SCKD=0, SCD2=0).  Internal serial-clock generation is not used at this
    // board boundary; time-slot timing enters through ssi_tx_ready.
    assign ssi_sclk_out = 1'b0;
    assign ssi_fss_out  = 1'b0;
    assign sci_tx_data = sci_stx;
    assign sci_tx_address = sci_stx_address;

    // ========================================================================
    // Interrupt Outputs (active when enabled in CRB/SCR)
    // ========================================================================
    assign irq_ssi_tx = crb[14] && ssisr[6];   // TIE && TDE
    assign irq_ssi_rx = crb[15] && ssisr[7];   // RIE && RDF
    assign irq_ssi_tx_exception = irq_ssi_tx && ssisr[4];
    assign irq_ssi_rx_exception = irq_ssi_rx && ssisr[5];
    assign irq_sci_tx = scr[12] && sci_ssr[1]; // TIE && TDRE
    assign irq_sci_rx = !sci_receiver_sleeping && scr[11] &&
                        sci_ssr[2] && !(|sci_ssr[6:4]);
    assign irq_sci_rx_exception = !sci_receiver_sleeping && scr[11] &&
                                  sci_ssr[2] && (|sci_ssr[6:4]);
    assign irq_sci_idle = !sci_receiver_sleeping && scr[10] &&
                          sci_idle_pending;
    assign irq_sci_timer = scr[13] && sci_timer_pending;
    assign irq_priority = ipr[15:0];
    assign bus_wait_states = bcr[15:0];

endmodule
