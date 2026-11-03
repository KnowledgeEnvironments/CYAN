// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - Z8530/Z85130 SCC Serial Controller
// Zilog SCC/ESCC dual-channel serial controller
//
// Implementation supports:
// - Channel A (keyboard) and B (console serial)
// - Asynchronous mode with configurable baud rate
// - TX shift register with serial output (Channels A and B)
// - Optional Z85130 enhanced personality with four-byte TX and eight-byte RX
//   FIFOs, WR7' extended reads, and selectable FIFO interrupt thresholds
// - TX/RX interrupt pending with Master Interrupt Enable
// - Status registers (RR0, RR1, RR3)
// - WR0 command decoding (Reset TX Int, Reset Ext/Status)
//
// Read data is combinatorial (available same cycle as valid)
// Side effects (pointer reset, rx_ready clear) are registered

`default_nettype none

module scc_z8530 #(
    parameter INST_NAME = "SCC",
    parameter integer CLK_HZ = 50000000,
    parameter integer BRG_CLOCK_HZ = 3672000,
    parameter integer FIXED_TX_BAUD = 0,
    parameter integer TX_BAUD = 9600,
    // Select the complete async Z85130 FIFO/register personality.  The name is
    // retained for compatibility with historical simulation invocations.
    parameter integer ADVERTISE_TX_FIFO = 0
) (
    input  wire        clk,
    input  wire        rst_n,

    // CPU bus interface
    input  wire [3:0]  addr,
    input  wire [63:0] wdata,
    output wire [63:0] rdata,       // Combinatorial read data
    input  wire [7:0]  wmask,
    input  wire        valid,
    input  wire        write,
    output wire        ready,

    // Serial outputs
    output wire        txd_a,
    output wire        txd_b,
    input  wire        rxd_a,
    input  wire        rxd_b,

    // RX injection (for simulation) - Channel B
    input  wire [7:0]  rx_inject_data,
    input  wire        rx_inject_valid,

    // RX injection - Channel A (keyboard)
    input  wire [7:0]  rx_inject_data_a,
    input  wire        rx_inject_valid_a,

    // TX monitor - Channel A (keyboard detection)
    output reg  [7:0]  tx_byte_out_a,
    output reg         tx_byte_valid_a,
    output reg  [7:0]  tx_byte_out_b,
    output reg         tx_byte_valid_b,

    // One-cycle description of each accepted CPU access.  This is strictly
    // observational: downstream trace backpressure must never reach ready.
    output wire [31:0] access_event_data,
    output wire        access_event_valid,

    output wire [127:0] debug_probe,

    // Interrupt output (directly active, active-high)
    output wire        scc_irq
);

    // =========================================================================
    // Write register pointer
    // =========================================================================

    reg [3:0] reg_ptr_a;
    reg [3:0] reg_ptr_b;
    // WR9 reset commands remain asserted internally for four additional
    // PCLKs. Accesses to the affected channel during that interval are
    // acknowledged by the bus wrapper but ignored by the SCC.
    reg [2:0] reset_recovery_a;
    reg [2:0] reset_recovery_b;

    // =========================================================================
    // Write Registers (WR0-WR15) - one set per channel
    // =========================================================================

    // Channel A
    reg [7:0] wr1_a, wr2_a, wr3_a, wr4_a, wr5_a, wr6_a, wr7_a;
    reg [7:0] wr9_a, wr10_a, wr11_a, wr12_a, wr13_a, wr14_a, wr15_a;
    reg [7:0] wr7p_a;  // WR7' (Z85130 enhanced feature register)

    // Channel B (console)
    reg [7:0] wr1_b, wr2_b, wr3_b, wr4_b, wr5_b, wr6_b, wr7_b;
    reg [7:0] wr9_b, wr10_b, wr11_b, wr12_b, wr13_b, wr14_b, wr15_b;
    reg [7:0] wr7p_b;  // WR7' (Z85130 enhanced feature register)

    // =========================================================================
    // TX shift registers and state machines
    // =========================================================================

    // Channel A
    reg [11:0] tx_shift_a;    // Start + data + parity + up to two stop bits
    reg [4:0]  tx_half_remaining_a;
    reg        tx_half_phase_a;
    reg        tx_active_a;
    reg [7:0]  tx_data_a;
    reg [7:0]  tx_fifo_1_a;
    reg [7:0]  tx_fifo_2_a;
    reg [7:0]  tx_fifo_3_a;
    reg [2:0]  tx_count_a;
    reg        tx_written_a;
    reg        tx_break_a;
    reg        tx_underrun_eom_a;

    // Channel B (console)
    reg [11:0] tx_shift_b;    // Start + data + parity + up to two stop bits
    reg [4:0]  tx_half_remaining_b;
    reg        tx_half_phase_b;
    reg        tx_active_b;
    reg [7:0]  tx_data_b;
    reg [7:0]  tx_fifo_1_b;
    reg [7:0]  tx_fifo_2_b;
    reg [7:0]  tx_fifo_3_b;
    reg [2:0]  tx_count_b;
    reg        tx_written_b;
    reg        tx_break_b;
    reg        tx_underrun_eom_b;

    wire tx_pending_a = (tx_count_a != 3'd0);
    wire tx_pending_b = (tx_count_b != 3'd0);

    // The Z85130 enhanced personality has an eight-byte receive FIFO.  Keep
    // rx_data_*/rx_ready_* as the externally useful head/occupied names used
    // by existing debug probes; entries 1..7 hold the remaining queue.
    reg [7:0]  rx_data_a;
    reg [7:0]  rx_fifo_a [1:7];
    reg [3:0]  rx_count_a;
    reg        rx_ready_a;
    reg        rx_overrun_a;
    reg        rx_framing_a;
    reg        rx_parity_error_a;
    reg [2:0]  rx_error_fifo_a [1:7]; // {framing, overrun, parity}
    reg        rx_break_a;
    reg        rx_current_parity_error_a;
    reg        rx_parity_a;
    reg        rx_first_armed_a;
    reg [2:0]  rxd_a_sync;
    reg [2:0]  rx_state_a;
    reg [15:0] rx_clk_count_a;
    reg [2:0]  rx_bit_index_a;
    reg [7:0]  rx_shift_a;

    // Channel B receive FIFO.
    reg [7:0]  rx_data_b;
    reg [7:0]  rx_fifo_b [1:7];
    reg [3:0]  rx_count_b;
    reg        rx_ready_b;     // Character available for reading
    reg        rx_overrun_b;
    reg        rx_framing_b;
    reg        rx_parity_error_b;
    reg [2:0]  rx_error_fifo_b [1:7]; // {framing, overrun, parity}
    reg        rx_break_b;
    reg        rx_current_parity_error_b;
    reg        rx_parity_b;
    reg        rx_first_armed_b;
    reg [2:0]  rxd_b_sync;
    reg [2:0]  rx_state_b;
    reg [15:0] rx_clk_count_b;
    reg [2:0]  rx_bit_index_b;
    reg [7:0]  rx_shift_b;

    integer fifo_index;

    // Baud rate generators
    reg [16:0] baud_cnt_a;
    wire [15:0] baud_tc_a = {wr13_a, wr12_a};
    reg [6:0] tx_clk_div_a;
    reg       brg_half_a;
    reg       brg_clock_en_a;
    reg       tx_program_half_phase_a;
    reg       tx_program_half_clk_en_a;
    reg       tx_program_clk_en_a;
    reg [31:0] fixed_baud_cnt_a;
    reg        tx_fixed_half_phase_a;
    reg        tx_fixed_half_clk_en_a;
    reg        tx_fixed_clk_en_a;

    reg [16:0] baud_cnt_b;
    wire [15:0] baud_tc_b = {wr13_b, wr12_b};

    reg [6:0] tx_clk_div_b;
    reg       brg_half_b;
    reg       brg_clock_en_b;
    reg       tx_program_half_phase_b;
    reg       tx_program_half_clk_en_b;
    reg       tx_program_clk_en_b;
    reg [31:0] fixed_baud_cnt_b;
    reg        tx_fixed_half_phase_b;
    reg        tx_fixed_half_clk_en_b;
    reg        tx_fixed_clk_en_b;
    reg [31:0] brg_ref_phase;
    reg        brg_ref_tick;
    localparam integer FIXED_CLKS_PER_HALF_BIT_RAW = CLK_HZ / (2 * TX_BAUD);
    localparam integer FIXED_CLKS_PER_HALF_BIT =
        (FIXED_CLKS_PER_HALF_BIT_RAW > 0) ? FIXED_CLKS_PER_HALF_BIT_RAW : 1;
    localparam [31:0] FIXED_CLKS_PER_HALF_BIT_LAST =
        (FIXED_CLKS_PER_HALF_BIT > 1) ? (FIXED_CLKS_PER_HALF_BIT - 1) : 32'd0;
    localparam integer RX_CLKS_PER_BIT = CLK_HZ / TX_BAUD;
    localparam integer RX_HALF_BIT = RX_CLKS_PER_BIT / 2;
    localparam [15:0] RX_HALF_BIT_16 = RX_HALF_BIT[15:0];
    localparam [15:0] RX_LAST_BIT_16 = RX_CLKS_PER_BIT[15:0] - 16'd1;
    localparam [2:0] RX_IDLE  = 3'd0;
    localparam [2:0] RX_START = 3'd1;
    localparam [2:0] RX_DATA  = 3'd2;
    localparam [2:0] RX_STOP  = 3'd3;
    localparam [2:0] RX_PARITY = 3'd4;
    wire [6:0] clock_divisor_a = (wr4_a[7:6] == 2'b00) ? 7'd1  :
                                 (wr4_a[7:6] == 2'b01) ? 7'd16 :
                                 (wr4_a[7:6] == 2'b10) ? 7'd32 : 7'd64;
    wire [6:0] clock_divisor_b = (wr4_b[7:6] == 2'b00) ? 7'd1  :
                                 (wr4_b[7:6] == 2'b01) ? 7'd16 :
                                 (wr4_b[7:6] == 2'b10) ? 7'd32 : 7'd64;

    // WR5 selects 5-or-less, 7, 6, or 8 transmitted data bits.  In the
    // 5-or-less mode the upper data bits encode the actual length, exactly as
    // on the NMOS/CMOS SCC: 1111000D sends one bit through 000DDDDD sending
    // five bits.  Data always leaves least-significant bit first.
    function automatic [3:0] tx_data_bits;
        input [7:0] data;
        input [7:0] wr5;
        begin
            case (wr5[6:5])
                2'b01: tx_data_bits = 4'd7;
                2'b10: tx_data_bits = 4'd6;
                2'b11: tx_data_bits = 4'd8;
                default: begin
                    casez (data)
                        8'b1111000?: tx_data_bits = 4'd1;
                        8'b111000??: tx_data_bits = 4'd2;
                        8'b11000???: tx_data_bits = 4'd3;
                        8'b1000????: tx_data_bits = 4'd4;
                        default:     tx_data_bits = 4'd5;
                    endcase
                end
            endcase
        end
    endfunction

    function automatic [11:0] tx_async_frame;
        input [7:0] data;
        input [7:0] wr4;
        input [7:0] wr5;
        integer i;
        reg [3:0] data_count;
        reg parity_xor;
        begin
            tx_async_frame = 12'hfff;
            tx_async_frame[0] = 1'b0;
            data_count = tx_data_bits(data, wr5);
            parity_xor = 1'b0;
            for (i = 0; i < 8; i = i + 1) begin
                if (i < data_count) begin
                    tx_async_frame[i + 1] = data[i];
                    parity_xor = parity_xor ^ data[i];
                end
            end
            if (wr4[0])
                // WR4[1]=1 selects even parity; zero selects odd parity.
                tx_async_frame[data_count + 1] = wr4[1]
                    ? parity_xor : ~parity_xor;
        end
    endfunction

    function automatic [4:0] tx_frame_half_bits;
        input [7:0] data;
        input [7:0] wr4;
        input [7:0] wr5;
        reg [2:0] stop_halves;
        begin
            case (wr4[3:2])
                2'b10: stop_halves = 3'd3; // 1.5 stop bits
                2'b11: stop_halves = 3'd4; // 2 stop bits
                default: stop_halves = 3'd2; // async one-stop selection
            endcase
            tx_frame_half_bits = (tx_data_bits(data, wr5) + 5'd1 +
                                  (wr4[0] ? 5'd1 : 5'd0)) * 5'd2 +
                                 {2'b00, stop_halves};
        end
    endfunction

    function automatic [3:0] rx_data_bits;
        input [7:0] wr3;
        begin
            case (wr3[7:6])
                2'b00: rx_data_bits = 4'd5;
                2'b01: rx_data_bits = 4'd7;
                2'b10: rx_data_bits = 4'd6;
                default: rx_data_bits = 4'd8;
            endcase
        end
    endfunction
    wire [15:0] programmed_rx_half_a = (clock_divisor_a > 1)
        ? {9'd0, clock_divisor_a[6:1]} - 16'd1 : 16'd0;
    wire [15:0] programmed_rx_half_b = (clock_divisor_b > 1)
        ? {9'd0, clock_divisor_b[6:1]} - 16'd1 : 16'd0;
    wire [15:0] programmed_rx_last_a = {9'd0, clock_divisor_a} - 16'd1;
    wire [15:0] programmed_rx_last_b = {9'd0, clock_divisor_b} - 16'd1;
    wire [15:0] rx_half_count_a = (FIXED_TX_BAUD != 0)
        ? RX_HALF_BIT_16 : programmed_rx_half_a;
    wire [15:0] rx_half_count_b = (FIXED_TX_BAUD != 0)
        ? RX_HALF_BIT_16 : programmed_rx_half_b;
    wire [15:0] rx_last_count_a = (FIXED_TX_BAUD != 0)
        ? RX_LAST_BIT_16 : programmed_rx_last_a;
    wire [15:0] rx_last_count_b = (FIXED_TX_BAUD != 0)
        ? RX_LAST_BIT_16 : programmed_rx_last_b;
    wire       rx_clock_en_a = (FIXED_TX_BAUD != 0) ? 1'b1 : brg_clock_en_a;
    wire       rx_clock_en_b = (FIXED_TX_BAUD != 0) ? 1'b1 : brg_clock_en_b;
    wire       tx_clk_en_a = (FIXED_TX_BAUD != 0)
        ? tx_fixed_clk_en_a : tx_program_clk_en_a;
    wire       tx_clk_en_b = (FIXED_TX_BAUD != 0)
        ? tx_fixed_clk_en_b : tx_program_clk_en_b;
    wire       tx_half_clk_en_a = (FIXED_TX_BAUD != 0)
        ? tx_fixed_half_clk_en_a : tx_program_half_clk_en_a;
    wire       tx_half_clk_en_b = (FIXED_TX_BAUD != 0)
        ? tx_fixed_half_clk_en_b : tx_program_half_clk_en_b;
    wire       rxd_a_i = rxd_a_sync[2];
    wire       rxd_b_i = rxd_b_sync[2];
    wire       rx_enabled_a = wr3_a[0];
    wire       rx_enabled_b = wr3_b[0];
    wire       reset_access_blocked =
        (reset_recovery_a != 3'd0 &&
         (addr[3:0] == 4'hB || addr[3:0] == 4'hF)) ||
        (reset_recovery_b != 3'd0 &&
         (addr[3:0] == 4'h3 || addr[3:0] == 4'h7));
    wire       rx_data_read_a = valid && ready && !reset_access_blocked &&
                                !write &&
                                (addr[3:0] == 4'hF);
    wire       rx_data_read_b = valid && ready && !reset_access_blocked &&
                                !write &&
                                (addr[3:0] == 4'h7);
    wire       rx_serial_char_valid_a = rx_enabled_a &&
                                        (rx_state_a == RX_STOP) &&
                                        rx_clock_en_a &&
                                        (rx_clk_count_a == rx_last_count_a);
    wire       rx_serial_char_valid_b = rx_enabled_b &&
                                        (rx_state_b == RX_STOP) &&
                                        rx_clock_en_b &&
                                        (rx_clk_count_b == rx_last_count_b);
    wire       rx_serial_break_start_a = rx_serial_char_valid_a &&
                                         !rx_break_a &&
                                         (rx_shift_a == 8'h00) && !rxd_a_i;
    wire       rx_serial_break_start_b = rx_serial_char_valid_b &&
                                         !rx_break_b &&
                                         (rx_shift_b == 8'h00) && !rxd_b_i;
    wire       rx_break_release_a = rx_break_a && rxd_a_i;
    wire       rx_break_release_b = rx_break_b && rxd_b_i;
    wire       rx_fifo_push_a = rx_inject_valid_a ||
                                (rx_serial_char_valid_a && !rx_break_a &&
                                 !rx_serial_break_start_a) ||
                                (rx_break_release_a && !rx_inject_valid_a);
    wire       rx_fifo_push_b = rx_inject_valid ||
                                (rx_serial_char_valid_b && !rx_break_b &&
                                 !rx_serial_break_start_b) ||
                                (rx_break_release_b && !rx_inject_valid);

    // Serial output.  The shift registers fill with ones as a frame drains,
    // so their LSBs are already the correct idle level.  Do not mux them with
    // tx_active here: tx_active and tx_shift change on the same clock edge at
    // frame boundaries, and a combinational mux can produce a narrow hardware
    // glitch that a UART mistakes for the start of an extra character.
    assign txd_a = tx_break_a ? 1'b0 : tx_shift_a[0];
    assign txd_b = tx_break_b ? 1'b0 : tx_shift_b[0];

    // =========================================================================
    // Interrupt Pending registers
    // =========================================================================

    reg tx_ip_a, tx_ip_b;     // TX Interrupt Pending
    reg rx_ip_a, rx_ip_b;     // RX Interrupt Pending
    reg ext_ip_a, ext_ip_b;   // External/Status Interrupt Pending

    // External/Status latch: previous RR0 value, set by Reset Ext/Status cmd
    reg [7:0] ext_latch_a, ext_latch_b;
    // WR15 mostly lines up with RR0, except D2: on the Z85C30 it enables the
    // SDLC frame-status FIFO and is not an interrupt enable for RR0 D2
    // (Transmit Buffer Empty).  Comparing all of WR15 against RR0 therefore
    // creates a spurious External/Status interrupt whenever a byte enters or
    // leaves the transmit holding register.
    localparam [7:0] WR15_EXT_STATUS_MASK = 8'hFA;
    wire enhanced_read_a = (ADVERTISE_TX_FIFO != 0) && wr7p_a[6];
    wire enhanced_read_b = (ADVERTISE_TX_FIFO != 0) && wr7p_b[6];
    // WR15 D0 identifies the enhanced device and selects WR7'.  The active
    // one-byte Z8530 personality must read it as zero.  D2 is the unmodeled
    // SDLC frame-status FIFO and must remain zero in either personality.
    wire [7:0] rr15_a = wr15_a &
        ((ADVERTISE_TX_FIFO != 0) ? 8'hFB : WR15_EXT_STATUS_MASK);
    wire [7:0] rr15_b = wr15_b &
        ((ADVERTISE_TX_FIFO != 0) ? 8'hFB : WR15_EXT_STATUS_MASK);

    // Master Interrupt Enable (WR9 is shared, written from either channel)
    wire mie = wr9_b[3];

    // RR3 (Master Interrupt Pending - only valid on Channel A per Z8530 spec)
    wire [7:0] rr3_a = {
        2'b0,
        rx_ip_a,    // Bit 5: Ch A RX IP
        tx_ip_a,    // Bit 4: Ch A TX IP
        ext_ip_a,   // Bit 3: Ch A Ext/Status IP
        rx_ip_b,    // Bit 2: Ch B RX IP
        tx_ip_b,    // Bit 1: Ch B TX IP
        ext_ip_b    // Bit 0: Ch B Ext/Status IP
    };

    reg [2:0] rr2_status_code;
    always @(*) begin
        // Z8530 priority: RxA > TxA > ExtA > RxB > TxB > ExtB.
        // RR2B reports this source in the vector status field.
        if (rx_ip_a)
            rr2_status_code = (rx_overrun_a || rx_framing_a ||
                               (rx_parity_error_a && wr1_a[2]))
                ? 3'b111 : 3'b110;
        else if (tx_ip_a)
            rr2_status_code = 3'b100;
        else if (ext_ip_a)
            rr2_status_code = 3'b101;
        else if (rx_ip_b)
            rr2_status_code = (rx_overrun_b || rx_framing_b ||
                               (rx_parity_error_b && wr1_b[2]))
                ? 3'b011 : 3'b010;
        else if (tx_ip_b)
            rr2_status_code = 3'b000;
        else if (ext_ip_b)
            rr2_status_code = 3'b001;
        else
            rr2_status_code = 3'b000;
    end

    wire [7:0] rr2_b = wr9_b[4]
        ? ((wr2_b & 8'h8F) | {1'b0, rr2_status_code, 4'b0000})
        : ((wr2_b & 8'hF1) | {4'b0000, rr2_status_code, 1'b0});

    // Interrupt output: any pending source with MIE enabled
    assign scc_irq = mie && (tx_ip_a | tx_ip_b | rx_ip_a | rx_ip_b | ext_ip_a | ext_ip_b);

`ifndef SYNTHESIS
    reg scc_debug;
    initial begin
        scc_debug = 1'b0;
        if ($test$plusargs("scc_debug") || $test$plusargs("trace_scc"))
            scc_debug = 1'b1;
    end
`endif

    // Debug: monitor SCC1 IRQ transitions (enabled for boot debug)
    reg scc_irq_prev;
    reg tx_ip_b_prev, ext_ip_b_prev;
    always @(posedge clk) begin
        scc_irq_prev <= scc_irq;
        tx_ip_b_prev <= tx_ip_b;
        ext_ip_b_prev <= ext_ip_b;
`ifndef SYNTHESIS
        if (scc_debug && scc_irq && !scc_irq_prev)
            $display("%s_IRQ: ASSERT mie=%b tx_ip=%b%b rx_ip=%b%b ext_ip=%b%b wr1_b=%02x", INST_NAME, mie, tx_ip_a, tx_ip_b, rx_ip_a, rx_ip_b, ext_ip_a, ext_ip_b, wr1_b);
`endif
        //if (!scc_irq && scc_irq_prev)
        //    $display("%s_IRQ: DEASSERT", INST_NAME);
        // if (tx_ip_b && !tx_ip_b_prev)
        //     $display("%s_TX_IP_B: SET (wr1_b=%02x tx_pending=%b tx_was_pending=%b)", INST_NAME, wr1_b, tx_pending_b, tx_was_pending_b);
        // if (!tx_ip_b && tx_ip_b_prev)
        //     $display("%s_TX_IP_B: CLEARED", INST_NAME);
        // if (ext_ip_b && !ext_ip_b_prev)
        //     $display("%s_EXT_IP_B: SET (wr15=%02x rr0=%02x latch=%02x)", INST_NAME, wr15_b, rr0_b, ext_latch_b);
    end

    // =========================================================================
    // Status bits
    // =========================================================================

    wire [7:0] rr0_b = {
        rx_break_b,     // Bit 7: Break/Abort
        tx_underrun_eom_b, // Bit 6: TX underrun/EOM
        1'b0,           // Bit 5: CTS asserted (active-low Z85C30/IP20 input)
        1'b0,           // Bit 4: Sync/Hunt
        1'b0,           // Bit 3: DCD asserted (active-low Z85C30/IP20 input)
        (ADVERTISE_TX_FIFO != 0) ? (tx_count_b < 3'd4) : !tx_pending_b,
                         // Bit 2: top TX FIFO location available
        1'b0,           // Bit 1: Zero count
        rx_ready_b      // Bit 0: RX char available
    };

    wire [7:0] rr1_b = {
        1'b0,                          // Bit 7: End of frame (SDLC)
        rx_framing_b,                  // Bit 6: CRC/Framing error
        rx_overrun_b,                  // Bit 5: RX overrun error
        rx_parity_error_b,             // Bit 4: Parity error
        1'b0,                          // Bit 3: Residue code 2
        1'b1,                          // Bit 2: Residue code 1
        1'b1,                          // Bit 1: Residue code 0
        !tx_active_b && !tx_pending_b  // Bit 0: All Sent
    };

    wire [7:0] rr0_a = {
        rx_break_a,     // Bit 7: Break/Abort
        tx_underrun_eom_a, // Bit 6: TX underrun/EOM
        1'b0,           // Bit 5: CTS asserted (active-low Z85C30/IP20 input)
        1'b0,           // Bit 4: Sync/Hunt
        1'b0,           // Bit 3: DCD asserted (active-low Z85C30/IP20 input)
        (ADVERTISE_TX_FIFO != 0) ? (tx_count_a < 3'd4) : !tx_pending_a,
                         // Bit 2: top TX FIFO location available
        1'b0,           // Bit 1: Zero count
        rx_ready_a      // Bit 0: RX char available
    };

    wire [7:0] rr1_a = {
        1'b0,                          // Bit 7: End of frame (SDLC)
        rx_framing_a,                  // Bit 6: CRC/Framing error
        rx_overrun_a,                  // Bit 5: RX overrun error
        rx_parity_error_a,             // Bit 4: Parity error
        1'b0,                          // Bit 3: Residue code 2
        1'b1,                          // Bit 2: Residue code 1
        1'b1,                          // Bit 1: Residue code 0
        !tx_active_a && !tx_pending_a  // Bit 0: All Sent
    };

    assign debug_probe = {
        32'h5343_435A, // "SCCZ"
        wr1_b, wr3_b, wr5_b, wr9_b,
        wr12_b, wr13_b, wr14_b, wr15_b,
        rr0_b,
        rr1_b,
        {rx_ip_a, tx_ip_a, ext_ip_a, rx_ip_b, tx_ip_b, ext_ip_b, mie, scc_irq},
        {rx_ready_b, rx_overrun_b, rx_framing_b, tx_pending_b,
         tx_active_b, txd_b, rxd_b_i, rx_enabled_b}
    };

    // Receive interrupt modes in WR1[4:3]:
    //   00 disabled, 01 first character/special, 10 every character/special,
    //   11 special condition only.
    task receive_char_a;
        input [7:0] value;
        input [2:0] error_status;
        input       special;
        begin
            // A CPU read and receiver completion may land on the same FPGA
            // clock.  Treat that as one atomic FIFO pop+push; independent
            // nonblocking assignments lose the arriving byte (or replay stale
            // stale tail data) when software polls a busy serial line.
            if (rx_data_read_a && rx_count_a != 4'd0) begin
                if (rx_count_a == 4'd1) begin
                    rx_data_a <= value;
                    {rx_framing_a, rx_overrun_a,
                     rx_parity_error_a} <= error_status;
                end else begin
                    rx_data_a <= rx_fifo_a[1];
                    {rx_framing_a, rx_overrun_a,
                     rx_parity_error_a} <= rx_error_fifo_a[1];
                    for (fifo_index = 1; fifo_index < 7; fifo_index = fifo_index + 1) begin
                        if (fifo_index < {28'd0, rx_count_a} - 32'd1) begin
                            rx_fifo_a[fifo_index] <= rx_fifo_a[fifo_index + 1];
                            rx_error_fifo_a[fifo_index] <=
                                rx_error_fifo_a[fifo_index + 1];
                        end
                    end
                    rx_fifo_a[rx_count_a - 1'b1] <= value;
                    rx_error_fifo_a[rx_count_a - 1'b1] <= error_status;
                end
                rx_ready_a <= 1'b1;
                case (wr1_a[4:3])
                    2'b01: begin
                        rx_ip_a <= rx_first_armed_a || special;
                        if (rx_first_armed_a)
                            rx_first_armed_a <= 1'b0;
                    end
                    2'b10: rx_ip_a <= special ||
                        (ADVERTISE_TX_FIFO == 0) || !wr7p_a[3] ||
                        (rx_count_a >= 4'd4);
                    2'b11: rx_ip_a <= special;
                    default: rx_ip_a <= 1'b0;
                endcase
            end else if (rx_count_a ==
                    ((ADVERTISE_TX_FIFO != 0) ? 4'd8 : 4'd3)) begin
                rx_overrun_a <= 1'b1;
                if (wr1_a[4:3] != 2'b00)
                    rx_ip_a <= 1'b1;
            end else begin
                if (rx_count_a == 4'd0) begin
                    rx_data_a <= value;
                    {rx_framing_a, rx_overrun_a,
                     rx_parity_error_a} <= error_status;
                end else begin
                    rx_fifo_a[rx_count_a] <= value;
                    rx_error_fifo_a[rx_count_a] <= error_status;
                end
                rx_count_a <= rx_count_a + 4'd1;
                rx_ready_a <= 1'b1;
                case (wr1_a[4:3])
                    2'b01: begin
                        if (rx_first_armed_a || special)
                            rx_ip_a <= 1'b1;
                        if (rx_first_armed_a)
                            rx_first_armed_a <= 1'b0;
                    end
                    2'b10: if (special || (ADVERTISE_TX_FIFO == 0) ||
                            !wr7p_a[3] || (rx_count_a >= 4'd3))
                        rx_ip_a <= 1'b1;
                    2'b11: if (special) rx_ip_a <= 1'b1;
                    default: ;
                endcase
            end
        end
    endtask

    task receive_char_b;
        input [7:0] value;
        input [2:0] error_status;
        input       special;
        begin
            if (rx_data_read_b && rx_count_b != 4'd0) begin
                if (rx_count_b == 4'd1) begin
                    rx_data_b <= value;
                    {rx_framing_b, rx_overrun_b,
                     rx_parity_error_b} <= error_status;
                end else begin
                    rx_data_b <= rx_fifo_b[1];
                    {rx_framing_b, rx_overrun_b,
                     rx_parity_error_b} <= rx_error_fifo_b[1];
                    for (fifo_index = 1; fifo_index < 7; fifo_index = fifo_index + 1) begin
                        if (fifo_index < {28'd0, rx_count_b} - 32'd1) begin
                            rx_fifo_b[fifo_index] <= rx_fifo_b[fifo_index + 1];
                            rx_error_fifo_b[fifo_index] <=
                                rx_error_fifo_b[fifo_index + 1];
                        end
                    end
                    rx_fifo_b[rx_count_b - 1'b1] <= value;
                    rx_error_fifo_b[rx_count_b - 1'b1] <= error_status;
                end
                rx_ready_b <= 1'b1;
                case (wr1_b[4:3])
                    2'b01: begin
                        rx_ip_b <= rx_first_armed_b || special;
                        if (rx_first_armed_b)
                            rx_first_armed_b <= 1'b0;
                    end
                    2'b10: rx_ip_b <= special ||
                        (ADVERTISE_TX_FIFO == 0) || !wr7p_b[3] ||
                        (rx_count_b >= 4'd4);
                    2'b11: rx_ip_b <= special;
                    default: rx_ip_b <= 1'b0;
                endcase
            end else if (rx_count_b ==
                    ((ADVERTISE_TX_FIFO != 0) ? 4'd8 : 4'd3)) begin
                rx_overrun_b <= 1'b1;
                if (wr1_b[4:3] != 2'b00)
                    rx_ip_b <= 1'b1;
            end else begin
                if (rx_count_b == 4'd0) begin
                    rx_data_b <= value;
                    {rx_framing_b, rx_overrun_b,
                     rx_parity_error_b} <= error_status;
                end else begin
                    rx_fifo_b[rx_count_b] <= value;
                    rx_error_fifo_b[rx_count_b] <= error_status;
                end
                rx_count_b <= rx_count_b + 4'd1;
                rx_ready_b <= 1'b1;
                case (wr1_b[4:3])
                    2'b01: begin
                        if (rx_first_armed_b || special)
                            rx_ip_b <= 1'b1;
                        if (rx_first_armed_b)
                            rx_first_armed_b <= 1'b0;
                    end
                    2'b10: if (special || (ADVERTISE_TX_FIFO == 0) ||
                            !wr7p_b[3] || (rx_count_b >= 4'd3))
                        rx_ip_b <= 1'b1;
                    2'b11: if (special) rx_ip_b <= 1'b1;
                    default: ;
                endcase
            end
        end
    endtask

    task reset_channel_a;
        begin
            reset_recovery_a <= 3'd4;
            tx_shift_a <= 12'hFFF;
            tx_half_remaining_a <= 5'd0;
            tx_half_phase_a <= 1'b0;
            tx_active_a <= 1'b0;
            tx_data_a <= 8'h0;
            tx_fifo_1_a <= 8'h0;
            tx_fifo_2_a <= 8'h0;
            tx_fifo_3_a <= 8'h0;
            tx_count_a <= 3'd0;
            tx_written_a <= 1'b0;
            tx_break_a <= 1'b0;
            tx_underrun_eom_a <= 1'b1;
            rx_data_a <= 8'h0;
            rx_count_a <= 4'd0;
            rx_ready_a <= 1'b0;
            rx_overrun_a <= 1'b0;
            rx_framing_a <= 1'b0;
            rx_parity_error_a <= 1'b0;
            for (fifo_index = 1; fifo_index < 8; fifo_index = fifo_index + 1) begin
                rx_fifo_a[fifo_index] <= 8'h0;
                rx_error_fifo_a[fifo_index] <= 3'b000;
            end
            rx_break_a <= 1'b0;
            rx_current_parity_error_a <= 1'b0;
            rx_parity_a <= 1'b0;
            rx_first_armed_a <= 1'b1;
            rxd_a_sync <= 3'b111;
            rx_state_a <= RX_IDLE;
            rx_clk_count_a <= 16'd0;
            rx_bit_index_a <= 3'd0;
            rx_shift_a <= 8'h0;
            tx_byte_out_a <= 8'h0;
            tx_byte_valid_a <= 1'b0;
            reg_ptr_a <= 4'd0;
            wr1_a <= 8'h0;
            wr3_a <= 8'h0;
            wr4_a <= 8'h04;
            wr5_a <= 8'h0;
            wr6_a <= 8'h0;
            wr7_a <= 8'h0;
            wr10_a <= 8'h0;
            wr11_a <= 8'h0;
            wr12_a <= 8'h0;
            wr13_a <= 8'h0;
            wr14_a <= 8'h20;
            wr15_a <= 8'hF8;
            wr7p_a <= 8'h20;
            tx_ip_a <= 1'b0;
            rx_ip_a <= 1'b0;
            ext_ip_a <= 1'b0;
            ext_latch_a <= 8'h44;
        end
    endtask

    task reset_channel_b;
        begin
            reset_recovery_b <= 3'd4;
            tx_shift_b <= 12'hFFF;
            tx_half_remaining_b <= 5'd0;
            tx_half_phase_b <= 1'b0;
            tx_active_b <= 1'b0;
            tx_data_b <= 8'h0;
            tx_fifo_1_b <= 8'h0;
            tx_fifo_2_b <= 8'h0;
            tx_fifo_3_b <= 8'h0;
            tx_count_b <= 3'd0;
            tx_written_b <= 1'b0;
            tx_break_b <= 1'b0;
            tx_underrun_eom_b <= 1'b1;
            rx_data_b <= 8'h0;
            rx_count_b <= 4'd0;
            rx_ready_b <= 1'b0;
            rx_overrun_b <= 1'b0;
            rx_framing_b <= 1'b0;
            rx_parity_error_b <= 1'b0;
            for (fifo_index = 1; fifo_index < 8; fifo_index = fifo_index + 1) begin
                rx_fifo_b[fifo_index] <= 8'h0;
                rx_error_fifo_b[fifo_index] <= 3'b000;
            end
            rx_break_b <= 1'b0;
            rx_current_parity_error_b <= 1'b0;
            rx_parity_b <= 1'b0;
            rx_first_armed_b <= 1'b1;
            rxd_b_sync <= 3'b111;
            rx_state_b <= RX_IDLE;
            rx_clk_count_b <= 16'd0;
            rx_bit_index_b <= 3'd0;
            rx_shift_b <= 8'h0;
            tx_byte_out_b <= 8'h0;
            tx_byte_valid_b <= 1'b0;
            reg_ptr_b <= 4'd0;
            wr1_b <= 8'h0;
            wr3_b <= 8'h0;
            wr4_b <= 8'h04;
            wr5_b <= 8'h0;
            wr6_b <= 8'h0;
            wr7_b <= 8'h0;
            wr10_b <= 8'h0;
            wr11_b <= 8'h0;
            wr12_b <= 8'h0;
            wr13_b <= 8'h0;
            wr14_b <= 8'h20;
            wr15_b <= 8'hF8;
            wr7p_b <= 8'h20;
            tx_ip_b <= 1'b0;
            rx_ip_b <= 1'b0;
            ext_ip_b <= 1'b0;
            ext_latch_b <= 8'h44;
        end
    endtask

    task apply_wr9;
        input [7:0] value;
        begin
`ifndef SYNTHESIS
            if (scc_debug)
                $display(
                    "%s_WR9: value=%02x reset_a=%b reset_b=%b tx_a(active=%b pending=%b half_remaining=%0d data=%02x) tx_b(active=%b pending=%b half_remaining=%0d data=%02x)",
                    INST_NAME, value, value[7], value[6],
                    tx_active_a, tx_pending_a, tx_half_remaining_a, tx_data_a,
                    tx_active_b, tx_pending_b, tx_half_remaining_b, tx_data_b);
`endif
            if (value[7])
                reset_channel_a();
            if (value[6])
                reset_channel_b();
            if (value[7:6] == 2'b11) begin
                wr2_a <= 8'h0;
                wr2_b <= 8'h0;
            end
            wr9_a <= {2'b00, value[5:0]};
            wr9_b <= {2'b00, value[5:0]};
        end
    endtask

    // =========================================================================
    // Address decode (big-endian byte addresses from IP20.h)
    // =========================================================================

    wire sel_ctrl_b = (addr[3:0] == 4'h3);  // Channel B Control (console)
    wire sel_data_b = (addr[3:0] == 4'h7);  // Channel B Data (console)
    wire sel_ctrl_a = (addr[3:0] == 4'hB);  // Channel A Control
    wire sel_data_a = (addr[3:0] == 4'hF);  // Channel A Data
    wire tx_write_a = write && (sel_data_a || (sel_ctrl_a && reg_ptr_a == 4'd8));
    wire tx_write_b = write && (sel_data_b || (sel_ctrl_b && reg_ptr_b == 4'd8));
    wire tx_fifo_full_a = (ADVERTISE_TX_FIFO != 0)
        ? (tx_count_a == 3'd4) : tx_pending_a;
    wire tx_fifo_full_b = (ADVERTISE_TX_FIFO != 0)
        ? (tx_count_b == 3'd4) : tx_pending_b;
    wire tx_write_blocked = (tx_write_a && tx_fifo_full_a) ||
                            (tx_write_b && tx_fifo_full_b);
    assign ready = !tx_write_blocked;

    wire tx_fifo_write_a = valid && ready && !reset_access_blocked &&
                           write && tx_write_a;
    wire tx_fifo_write_b = valid && ready && !reset_access_blocked &&
                           write && tx_write_b;

    // =========================================================================
    // Combinatorial read data output
    // =========================================================================

    wire [7:0] wdata_byte =
        wmask[7] ? wdata[63:56] :
        wmask[6] ? wdata[55:48] :
        wmask[5] ? wdata[47:40] :
        wmask[4] ? wdata[39:32] :
        wmask[3] ? wdata[31:24] :
        wmask[2] ? wdata[23:16] :
        wmask[1] ? wdata[15:8]  :
                   wdata[7:0];

    // Read data computed combinatorially from current registered state
    reg [63:0] rdata_comb;
    assign rdata = rdata_comb;

    always @(*) begin
        rdata_comb = 64'h0;
        // The original SCC aliases RR4..RR7 to RR0..RR3.  Enhanced write
        // register mirrors exist only when WR7' D6 enables extended reads;
        // returning WR contents unconditionally falsely identifies an ESCC.
        if (sel_ctrl_b) begin
            case (reg_ptr_b)
                4'd0:    rdata_comb = {8{rr0_b}};
                4'd1:    rdata_comb = {8{rr1_b}};
                4'd2:    rdata_comb = {8{rr2_b}};     // RR2: modified interrupt vector
                4'd3:    rdata_comb = 64'h0;          // RR3 is valid only on Channel A
                4'd4:    rdata_comb = enhanced_read_b ? {8{wr4_b}} : {8{rr0_b}};
                4'd5:    rdata_comb = enhanced_read_b ? {8{wr5_b}} : {8{rr1_b}};
                4'd6:    rdata_comb = {8{rr2_b}};     // RR6 aliases RR2 without SDLC FIFO
                4'd7:    rdata_comb = 64'h0;          // RR7 aliases Channel B RR3
                4'd9:    rdata_comb = enhanced_read_b ? {8{wr3_b}} : 64'h0;
                4'd10:   rdata_comb = 64'h0;          // RR10 loop/clock status
                4'd11:   rdata_comb = enhanced_read_b ? {8{wr10_b}} : 64'h0;
                4'd12:   rdata_comb = {8{wr12_b}};    // RR12: BRG low byte
                4'd13:   rdata_comb = {8{wr13_b}};    // RR13: BRG high byte
                // Do not advertise Z85130 WR7' FIFO features until this model
                // implements the 4-byte transmit FIFO those features imply.
                4'd14:   rdata_comb = enhanced_read_b ? {8{wr7p_b}} : 64'h0;
                // This model has neither the Z85130 enhanced selector in D0
                // nor the Z85C30 SDLC frame-status FIFO in D2.
                4'd15:   rdata_comb = {8{rr15_b}};
                default: rdata_comb = {8{rr0_b}};
            endcase
        end
        else if (sel_data_b)
            rdata_comb = rx_ready_b ? {8{rx_data_b}} : 64'h0;
        else if (sel_ctrl_a) begin
            case (reg_ptr_a)
                4'd0:    rdata_comb = {8{rr0_a}};
                4'd1:    rdata_comb = {8{rr1_a}};
                4'd2:    rdata_comb = {8{wr2_a}};     // RR2: interrupt vector (unmodified for Ch A)
                4'd3:    rdata_comb = {8{rr3_a}};     // RR3: Master IP (Ch A only)
                4'd4:    rdata_comb = enhanced_read_a ? {8{wr4_a}} : {8{rr0_a}};
                4'd5:    rdata_comb = enhanced_read_a ? {8{wr5_a}} : {8{rr1_a}};
                4'd6:    rdata_comb = {8{wr2_a}};     // RR6 aliases RR2 without SDLC FIFO
                4'd7:    rdata_comb = {8{rr3_a}};     // RR7 aliases RR3 without SDLC FIFO
                4'd9:    rdata_comb = enhanced_read_a ? {8{wr3_a}} : 64'h0;
                4'd10:   rdata_comb = 64'h0;          // RR10 loop/clock status
                4'd11:   rdata_comb = enhanced_read_a ? {8{wr10_a}} : 64'h0;
                4'd12:   rdata_comb = {8{wr12_a}};    // RR12: BRG low byte
                4'd13:   rdata_comb = {8{wr13_a}};    // RR13: BRG high byte
                4'd14:   rdata_comb = enhanced_read_a ? {8{wr7p_a}} : 64'h0;
                4'd15:   rdata_comb = {8{rr15_a}};
                default: rdata_comb = {8{rr0_a}};
            endcase
        end
        else if (sel_data_a)
            rdata_comb = rx_ready_a ? {8{rx_data_a}} : 64'h0;
    end

    wire access_channel_a = sel_ctrl_a || sel_data_a;
    wire access_control = sel_ctrl_a || sel_ctrl_b;
    wire [3:0] access_reg_ptr = access_channel_a ? reg_ptr_a : reg_ptr_b;
    wire access_rx_ready = access_channel_a ? rx_ready_a : rx_ready_b;
    wire access_tx_pending = access_channel_a ? tx_pending_a : tx_pending_b;
    wire [7:0] access_rr0 = access_channel_a ? rr0_a : rr0_b;
    wire [7:0] access_byte = write ? wdata_byte : rdata_comb[7:0];
    assign access_event_valid = valid && ready;
    assign access_event_data = {
        write, access_channel_a, access_control, reset_access_blocked,
        scc_irq, mie, access_rx_ready, access_tx_pending,
        addr, access_reg_ptr, access_byte, access_rr0
    };

    // =========================================================================
    // Baud rate generator and TX clock
    // =========================================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            brg_ref_phase <= 32'd0;
            brg_ref_tick <= 1'b0;
            baud_cnt_a <= 17'd0;
            tx_clk_div_a <= 7'd0;
            brg_half_a <= 1'b0;
            brg_clock_en_a <= 1'b0;
            tx_program_half_phase_a <= 1'b0;
            tx_program_half_clk_en_a <= 1'b0;
            tx_program_clk_en_a <= 1'b0;
            fixed_baud_cnt_a <= 32'd0;
            tx_fixed_half_phase_a <= 1'b0;
            tx_fixed_half_clk_en_a <= 1'b0;
            tx_fixed_clk_en_a <= 1'b0;
            baud_cnt_b <= 17'd0;
            tx_clk_div_b <= 7'd0;
            brg_half_b <= 1'b0;
            brg_clock_en_b <= 1'b0;
            tx_program_half_phase_b <= 1'b0;
            tx_program_half_clk_en_b <= 1'b0;
            tx_program_clk_en_b <= 1'b0;
            fixed_baud_cnt_b <= 32'd0;
            tx_fixed_half_phase_b <= 1'b0;
            tx_fixed_half_clk_en_b <= 1'b0;
            tx_fixed_clk_en_b <= 1'b0;
        end else begin
            brg_ref_tick <= 1'b0;
            brg_clock_en_a <= 1'b0;
            brg_clock_en_b <= 1'b0;
            tx_program_half_clk_en_a <= 1'b0;
            tx_program_half_clk_en_b <= 1'b0;
            tx_program_clk_en_a <= 1'b0;
            tx_program_clk_en_b <= 1'b0;
            tx_fixed_half_clk_en_a <= 1'b0;
            tx_fixed_half_clk_en_b <= 1'b0;
            tx_fixed_clk_en_a <= 1'b0;
            tx_fixed_clk_en_b <= 1'b0;

            // Fractional reference-clock enable.  This models the Indigo's
            // 3.672 MHz SCC PCLK while the RTL remains synchronous to the
            // board clock, without introducing a fabric-generated clock.
            if (BRG_CLOCK_HZ >= CLK_HZ) begin
                brg_ref_phase <= 32'd0;
                brg_ref_tick <= 1'b1;
            end else if (brg_ref_phase + BRG_CLOCK_HZ >= CLK_HZ) begin
                brg_ref_phase <= brg_ref_phase + BRG_CLOCK_HZ - CLK_HZ;
                brg_ref_tick <= 1'b1;
            end else begin
                brg_ref_phase <= brg_ref_phase + BRG_CLOCK_HZ;
            end

            if (FIXED_TX_BAUD != 0) begin
                baud_cnt_a <= 17'd0;
                tx_clk_div_a <= 7'd0;
                brg_half_a <= 1'b0;
                tx_program_half_phase_a <= 1'b0;
                baud_cnt_b <= 17'd0;
                tx_clk_div_b <= 7'd0;
                brg_half_b <= 1'b0;
                tx_program_half_phase_b <= 1'b0;

                if (FIXED_CLKS_PER_HALF_BIT <= 1) begin
                    fixed_baud_cnt_a <= 32'd0;
                    tx_fixed_half_clk_en_a <= 1'b1;
                    tx_fixed_half_phase_a <= ~tx_fixed_half_phase_a;
                    if (tx_fixed_half_phase_a)
                        tx_fixed_clk_en_a <= 1'b1;
                end else if (fixed_baud_cnt_a >= FIXED_CLKS_PER_HALF_BIT_LAST) begin
                    fixed_baud_cnt_a <= 32'd0;
                    tx_fixed_half_clk_en_a <= 1'b1;
                    tx_fixed_half_phase_a <= ~tx_fixed_half_phase_a;
                    if (tx_fixed_half_phase_a)
                        tx_fixed_clk_en_a <= 1'b1;
                end else begin
                    fixed_baud_cnt_a <= fixed_baud_cnt_a + 32'd1;
                end

                if (FIXED_CLKS_PER_HALF_BIT <= 1) begin
                    fixed_baud_cnt_b <= 32'd0;
                    tx_fixed_half_clk_en_b <= 1'b1;
                    tx_fixed_half_phase_b <= ~tx_fixed_half_phase_b;
                    if (tx_fixed_half_phase_b)
                        tx_fixed_clk_en_b <= 1'b1;
                end else if (fixed_baud_cnt_b >= FIXED_CLKS_PER_HALF_BIT_LAST) begin
                    fixed_baud_cnt_b <= 32'd0;
                    tx_fixed_half_clk_en_b <= 1'b1;
                    tx_fixed_half_phase_b <= ~tx_fixed_half_phase_b;
                    if (tx_fixed_half_phase_b)
                        tx_fixed_clk_en_b <= 1'b1;
                end else begin
                    fixed_baud_cnt_b <= fixed_baud_cnt_b + 32'd1;
                end
            end else begin
                fixed_baud_cnt_a <= 32'd0;
                fixed_baud_cnt_b <= 32'd0;

                if (!wr14_a[0]) begin
                    baud_cnt_a <= 17'd0;
                    tx_clk_div_a <= 7'd0;
                    brg_half_a <= 1'b0;
                    tx_program_half_phase_a <= 1'b0;
                end else if (brg_ref_tick) begin
                    if (baud_cnt_a >= {1'b0, baud_tc_a} + 17'd1) begin
                        baud_cnt_a <= 17'd0;
                        brg_half_a <= ~brg_half_a;
                        if (brg_half_a)
                            brg_clock_en_a <= 1'b1;

                        // A BRG terminal count is one half of the raw BRG
                        // clock.  Dividing those events by WR4's x1/x16/x32/
                        // x64 selection produces a true half-data-bit enable;
                        // every second one is the legacy full-bit enable.
                        if (tx_clk_div_a >= clock_divisor_a - 7'd1) begin
                            tx_clk_div_a <= 7'd0;
                            tx_program_half_clk_en_a <= 1'b1;
                            tx_program_half_phase_a <= ~tx_program_half_phase_a;
                            if (tx_program_half_phase_a)
                                tx_program_clk_en_a <= 1'b1;
                        end else begin
                            tx_clk_div_a <= tx_clk_div_a + 7'd1;
                        end
                    end else begin
                        baud_cnt_a <= baud_cnt_a + 17'd1;
                    end
                end

                if (!wr14_b[0]) begin
                    baud_cnt_b <= 17'd0;
                    tx_clk_div_b <= 7'd0;
                    brg_half_b <= 1'b0;
                    tx_program_half_phase_b <= 1'b0;
                end else if (brg_ref_tick) begin
                    if (baud_cnt_b >= {1'b0, baud_tc_b} + 17'd1) begin
                        baud_cnt_b <= 17'd0;
                        brg_half_b <= ~brg_half_b;
                        if (brg_half_b)
                            brg_clock_en_b <= 1'b1;

                        if (tx_clk_div_b >= clock_divisor_b - 7'd1) begin
                            tx_clk_div_b <= 7'd0;
                            tx_program_half_clk_en_b <= 1'b1;
                            tx_program_half_phase_b <= ~tx_program_half_phase_b;
                            if (tx_program_half_phase_b)
                                tx_program_clk_en_b <= 1'b1;
                        end else begin
                            tx_clk_div_b <= tx_clk_div_b + 7'd1;
                        end
                    end else begin
                        baud_cnt_b <= baud_cnt_b + 17'd1;
                    end
                end
            end
        end
    end

    // =========================================================================
    // Main registered logic: TX, RX, interrupts, register access
    // =========================================================================

    wire tx_fifo_pop_a = tx_pending_a && wr5_a[3] &&
        ((!tx_active_a && tx_clk_en_a) ||
         (tx_active_a && tx_half_clk_en_a &&
          tx_half_remaining_a == 5'd1));
    wire tx_fifo_pop_b = tx_pending_b && wr5_b[3] &&
        ((!tx_active_b && tx_clk_en_b) ||
         (tx_active_b && tx_half_clk_en_b &&
          tx_half_remaining_b == 5'd1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_shift_a <= 12'hFFF;
            tx_half_remaining_a <= 5'd0;
            tx_half_phase_a <= 1'b0;
            tx_active_a <= 1'b0;
            tx_data_a <= 8'h0;
            tx_fifo_1_a <= 8'h0;
            tx_fifo_2_a <= 8'h0;
            tx_fifo_3_a <= 8'h0;
            tx_count_a <= 3'd0;
            tx_written_a <= 1'b0;
            tx_break_a <= 1'b0;
            tx_underrun_eom_a <= 1'b1;
            tx_shift_b <= 12'hFFF;
            tx_half_remaining_b <= 5'd0;
            tx_half_phase_b <= 1'b0;
            tx_active_b <= 1'b0;
            tx_data_b <= 8'h0;
            tx_fifo_1_b <= 8'h0;
            tx_fifo_2_b <= 8'h0;
            tx_fifo_3_b <= 8'h0;
            tx_count_b <= 3'd0;
            tx_written_b <= 1'b0;
            tx_break_b <= 1'b0;
            tx_underrun_eom_b <= 1'b1;
            rx_data_a <= 8'h0;
            rx_count_a <= 4'd0;
            rx_ready_a <= 1'b0;
            rx_overrun_a <= 1'b0;
            rx_framing_a <= 1'b0;
            rx_parity_error_a <= 1'b0;
            rx_break_a <= 1'b0;
            rx_current_parity_error_a <= 1'b0;
            rx_parity_a <= 1'b0;
            rx_first_armed_a <= 1'b1;
            rxd_a_sync <= 3'b111;
            rx_state_a <= RX_IDLE;
            rx_clk_count_a <= 16'd0;
            rx_bit_index_a <= 3'd0;
            rx_shift_a <= 8'h0;
            rx_data_b <= 8'h0;
            rx_count_b <= 4'd0;
            rx_ready_b <= 1'b0;
            rx_overrun_b <= 1'b0;
            rx_framing_b <= 1'b0;
            rx_parity_error_b <= 1'b0;
            rx_break_b <= 1'b0;
            rx_current_parity_error_b <= 1'b0;
            rx_parity_b <= 1'b0;
            rx_first_armed_b <= 1'b1;
            rxd_b_sync <= 3'b111;
            rx_state_b <= RX_IDLE;
            rx_clk_count_b <= 16'd0;
            rx_bit_index_b <= 3'd0;
            rx_shift_b <= 8'h0;
            for (fifo_index = 1; fifo_index < 8; fifo_index = fifo_index + 1) begin
                rx_fifo_a[fifo_index] <= 8'h0;
                rx_error_fifo_a[fifo_index] <= 3'b000;
                rx_fifo_b[fifo_index] <= 8'h0;
                rx_error_fifo_b[fifo_index] <= 3'b000;
            end
            tx_byte_out_a <= 8'h0;
            tx_byte_valid_a <= 1'b0;
            tx_byte_out_b <= 8'h0;
            tx_byte_valid_b <= 1'b0;
            reg_ptr_a <= 4'd0;
            reg_ptr_b <= 4'd0;
            reset_recovery_a <= 3'd4;
            reset_recovery_b <= 3'd4;
            wr1_a <= 8'h0; wr2_a <= 8'h0; wr3_a <= 8'h0; wr4_a <= 8'h04; wr5_a <= 8'h0; wr6_a <= 8'h0; wr7_a <= 8'h0;
            wr9_a <= 8'h0; wr10_a <= 8'h0; wr11_a <= 8'h0; wr12_a <= 8'h0; wr13_a <= 8'h0; wr14_a <= 8'h20; wr15_a <= 8'hF8;
            wr7p_a <= 8'h20;
            wr1_b <= 8'h0; wr2_b <= 8'h0; wr3_b <= 8'h0; wr4_b <= 8'h04; wr5_b <= 8'h0; wr6_b <= 8'h0; wr7_b <= 8'h0;
            wr9_b <= 8'h0; wr10_b <= 8'h0; wr11_b <= 8'h0; wr12_b <= 8'h0; wr13_b <= 8'h0; wr14_b <= 8'h20; wr15_b <= 8'hF8;
            wr7p_b <= 8'h20;
            tx_ip_a <= 1'b0; tx_ip_b <= 1'b0;
            rx_ip_a <= 1'b0; rx_ip_b <= 1'b0;
            ext_ip_a <= 1'b0; ext_ip_b <= 1'b0;
            ext_latch_a <= 8'h44; ext_latch_b <= 8'h44;
        end else begin
            if (reset_recovery_a != 3'd0)
                reset_recovery_a <= reset_recovery_a - 3'd1;
            if (reset_recovery_b != 3'd0)
                reset_recovery_b <= reset_recovery_b - 3'd1;
            rxd_a_sync <= {rxd_a_sync[1:0], rxd_a};
            rxd_b_sync <= {rxd_b_sync[1:0], rxd_b};

            if (!rx_enabled_a || rx_break_a) begin
                rx_state_a <= RX_IDLE;
                rx_clk_count_a <= 16'd0;
                rx_bit_index_a <= 3'd0;
            end else begin
                case (rx_state_a)
                    RX_IDLE: begin
                        rx_clk_count_a <= 16'd0;
                        rx_bit_index_a <= 3'd0;
                        if (!rxd_a_i)
                            rx_state_a <= RX_START;
                    end

                    RX_START: begin
                        if (rx_clock_en_a) begin
                            if (rx_clk_count_a == rx_half_count_a) begin
                                rx_clk_count_a <= 16'd0;
                                if (rxd_a_i) begin
                                    rx_state_a <= RX_IDLE;
                                end else begin
                                    rx_shift_a <= 8'h00;
                                    rx_parity_a <= 1'b0;
                                    rx_current_parity_error_a <= 1'b0;
                                    rx_state_a <= RX_DATA;
                                end
                            end else begin
                                rx_clk_count_a <= rx_clk_count_a + 16'd1;
                            end
                        end
                    end

                    RX_DATA: begin
                        if (rx_clock_en_a) begin
                            if (rx_clk_count_a == rx_last_count_a) begin
                                rx_clk_count_a <= 16'd0;
                                rx_shift_a[rx_bit_index_a] <= rxd_a_i;
                                rx_parity_a <= rx_parity_a ^ rxd_a_i;
                                if ({1'b0, rx_bit_index_a} ==
                                        rx_data_bits(wr3_a) - 1'b1)
                                    rx_state_a <= wr4_a[0] ? RX_PARITY : RX_STOP;
                                else begin
                                    rx_bit_index_a <= rx_bit_index_a + 3'd1;
                                end
                            end else begin
                                rx_clk_count_a <= rx_clk_count_a + 16'd1;
                            end
                        end
                    end

                    RX_PARITY: begin
                        if (rx_clock_en_a) begin
                            if (rx_clk_count_a == rx_last_count_a) begin
                                rx_clk_count_a <= 16'd0;
                                if (rxd_a_i != (wr4_a[1]
                                        ? rx_parity_a : ~rx_parity_a)) begin
                                    rx_current_parity_error_a <= 1'b1;
                                end
                                rx_state_a <= RX_STOP;
                            end else begin
                                rx_clk_count_a <= rx_clk_count_a + 16'd1;
                            end
                        end
                    end

                    RX_STOP: begin
                        if (rx_clock_en_a) begin
                            if (rx_clk_count_a == rx_last_count_a) begin
                                rx_clk_count_a <= 16'd0;
                                rx_state_a <= RX_IDLE;
                            end else begin
                                rx_clk_count_a <= rx_clk_count_a + 16'd1;
                            end
                        end
                    end

                    default: rx_state_a <= RX_IDLE;
                endcase
            end

            if (!rx_enabled_b || rx_break_b) begin
                rx_state_b <= RX_IDLE;
                rx_clk_count_b <= 16'd0;
                rx_bit_index_b <= 3'd0;
            end else begin
                case (rx_state_b)
                    RX_IDLE: begin
                        rx_clk_count_b <= 16'd0;
                        rx_bit_index_b <= 3'd0;
                        if (!rxd_b_i)
                            rx_state_b <= RX_START;
                    end

                    RX_START: begin
                        if (rx_clock_en_b) begin
                            if (rx_clk_count_b == rx_half_count_b) begin
                                rx_clk_count_b <= 16'd0;
                                if (rxd_b_i) begin
                                    rx_state_b <= RX_IDLE;
                                end else begin
                                    rx_shift_b <= 8'h00;
                                    rx_parity_b <= 1'b0;
                                    rx_current_parity_error_b <= 1'b0;
                                    rx_state_b <= RX_DATA;
                                end
                            end else begin
                                rx_clk_count_b <= rx_clk_count_b + 16'd1;
                            end
                        end
                    end

                    RX_DATA: begin
                        if (rx_clock_en_b) begin
                            if (rx_clk_count_b == rx_last_count_b) begin
                                rx_clk_count_b <= 16'd0;
                                rx_shift_b[rx_bit_index_b] <= rxd_b_i;
                                rx_parity_b <= rx_parity_b ^ rxd_b_i;
                                if ({1'b0, rx_bit_index_b} ==
                                        rx_data_bits(wr3_b) - 1'b1)
                                    rx_state_b <= wr4_b[0] ? RX_PARITY : RX_STOP;
                                else begin
                                    rx_bit_index_b <= rx_bit_index_b + 3'd1;
                                end
                            end else begin
                                rx_clk_count_b <= rx_clk_count_b + 16'd1;
                            end
                        end
                    end

                    RX_PARITY: begin
                        if (rx_clock_en_b) begin
                            if (rx_clk_count_b == rx_last_count_b) begin
                                rx_clk_count_b <= 16'd0;
                                if (rxd_b_i != (wr4_b[1]
                                        ? rx_parity_b : ~rx_parity_b)) begin
                                    rx_current_parity_error_b <= 1'b1;
                                end
                                rx_state_b <= RX_STOP;
                            end else begin
                                rx_clk_count_b <= rx_clk_count_b + 16'd1;
                            end
                        end
                    end

                    RX_STOP: begin
                        if (rx_clock_en_b) begin
                            if (rx_clk_count_b == rx_last_count_b) begin
                                rx_clk_count_b <= 16'd0;
                                rx_state_b <= RX_IDLE;
                            end else begin
                                rx_clk_count_b <= rx_clk_count_b + 16'd1;
                            end
                        end
                    end

                    default: rx_state_b <= RX_IDLE;
                endcase
            end

            // One receive event per channel per clock.  Injection takes
            // priority only for deterministic simulation/debug behavior.
            if (rx_inject_valid)
                receive_char_b(rx_inject_data, 3'b000, 1'b0);
            else if (rx_serial_char_valid_b && !rx_break_b) begin
                if (rx_serial_break_start_b) begin
                    // A null character plus framing error begins BREAK. The
                    // SCC suppresses repeated nulls until marking returns.
                    rx_break_b <= 1'b1;
                end else begin
                    receive_char_b(rx_shift_b,
                                   {!rxd_b_i, 1'b0,
                                    rx_current_parity_error_b},
                                   !rxd_b_i ||
                                   (rx_current_parity_error_b && wr1_b[2]));
                end
            end

            if (rx_inject_valid_a)
                receive_char_a(rx_inject_data_a, 3'b000, 1'b0);
            else if (rx_serial_char_valid_a && !rx_break_a) begin
                if (rx_serial_break_start_a) begin
                    rx_break_a <= 1'b1;
                end else begin
                    receive_char_a(rx_shift_a,
                                   {!rxd_a_i, 1'b0,
                                    rx_current_parity_error_a},
                                   !rxd_a_i ||
                                   (rx_current_parity_error_a && wr1_a[2]));
                end
            end

            // BREAK terminates on the first marking level. The real SCC
            // leaves exactly one null in the receive FIFO, without framing
            // error; odd parity still makes that null a parity error.
            if (rx_break_release_b && !rx_inject_valid) begin
                rx_break_b <= 1'b0;
                rx_state_b <= RX_IDLE;
                receive_char_b(8'h00, {2'b00, wr4_b[0] && !wr4_b[1]},
                               wr4_b[0] && !wr4_b[1] && wr1_b[2]);
            end
            if (rx_break_release_a && !rx_inject_valid_a) begin
                rx_break_a <= 1'b0;
                rx_state_a <= RX_IDLE;
                receive_char_a(8'h00, {2'b00, wr4_a[0] && !wr4_a[1]},
                               wr4_a[0] && !wr4_a[1] && wr1_a[2]);
            end

            // Channel A/B TX: clear byte-valid after 1 cycle (pulse)
            tx_byte_valid_a <= 1'b0;
            tx_byte_valid_b <= 1'b0;

            // Four-entry Z85130 TX FIFOs.  Pop and CPU push are one atomic
            // occupancy transition so a byte accepted at a frame boundary is
            // neither lost nor duplicated.  The legacy personality constrains
            // ready to one queued byte but uses the same data path.
            if (tx_fifo_write_a)
                tx_written_a <= 1'b1;
            if (tx_fifo_write_b)
                tx_written_b <= 1'b1;

            case ({tx_fifo_pop_a, tx_fifo_write_a})
                2'b01: begin
                    case (tx_count_a)
                        3'd0: tx_data_a <= wdata_byte;
                        3'd1: tx_fifo_1_a <= wdata_byte;
                        3'd2: tx_fifo_2_a <= wdata_byte;
                        default: tx_fifo_3_a <= wdata_byte;
                    endcase
                    tx_count_a <= tx_count_a + 3'd1;
                end
                2'b10: begin
                    tx_data_a <= tx_fifo_1_a;
                    tx_fifo_1_a <= tx_fifo_2_a;
                    tx_fifo_2_a <= tx_fifo_3_a;
                    tx_count_a <= tx_count_a - 3'd1;
                    if (wr1_a[1] &&
                        (((ADVERTISE_TX_FIFO != 0) && !wr7p_a[5]) ||
                         (tx_count_a == 3'd1)))
                        tx_ip_a <= 1'b1;
                end
                2'b11: begin
                    case (tx_count_a)
                        3'd1: tx_data_a <= wdata_byte;
                        3'd2: begin
                            tx_data_a <= tx_fifo_1_a;
                            tx_fifo_1_a <= wdata_byte;
                        end
                        3'd3: begin
                            tx_data_a <= tx_fifo_1_a;
                            tx_fifo_1_a <= tx_fifo_2_a;
                            tx_fifo_2_a <= wdata_byte;
                        end
                        default: begin
                            tx_data_a <= tx_fifo_1_a;
                            tx_fifo_1_a <= tx_fifo_2_a;
                            tx_fifo_2_a <= tx_fifo_3_a;
                            tx_fifo_3_a <= wdata_byte;
                        end
                    endcase
                end
                default: ;
            endcase

            case ({tx_fifo_pop_b, tx_fifo_write_b})
                2'b01: begin
                    case (tx_count_b)
                        3'd0: tx_data_b <= wdata_byte;
                        3'd1: tx_fifo_1_b <= wdata_byte;
                        3'd2: tx_fifo_2_b <= wdata_byte;
                        default: tx_fifo_3_b <= wdata_byte;
                    endcase
                    tx_count_b <= tx_count_b + 3'd1;
                end
                2'b10: begin
                    tx_data_b <= tx_fifo_1_b;
                    tx_fifo_1_b <= tx_fifo_2_b;
                    tx_fifo_2_b <= tx_fifo_3_b;
                    tx_count_b <= tx_count_b - 3'd1;
                    if (wr1_b[1] &&
                        (((ADVERTISE_TX_FIFO != 0) && !wr7p_b[5]) ||
                         (tx_count_b == 3'd1)))
                        tx_ip_b <= 1'b1;
                end
                2'b11: begin
                    case (tx_count_b)
                        3'd1: tx_data_b <= wdata_byte;
                        3'd2: begin
                            tx_data_b <= tx_fifo_1_b;
                            tx_fifo_1_b <= wdata_byte;
                        end
                        3'd3: begin
                            tx_data_b <= tx_fifo_1_b;
                            tx_fifo_1_b <= tx_fifo_2_b;
                            tx_fifo_2_b <= wdata_byte;
                        end
                        default: begin
                            tx_data_b <= tx_fifo_1_b;
                            tx_fifo_1_b <= tx_fifo_2_b;
                            tx_fifo_2_b <= tx_fifo_3_b;
                            tx_fifo_3_b <= wdata_byte;
                        end
                    endcase
                end
                default: ;
            endcase

            // External/Status Interrupt: fire when RR0 status differs from latch
            // and corresponding WR15 IE bits are enabled, and WR1[0] (Ext Int Enable)
            // Since DCD/CTS are hardwired, this mainly fires after Reset Ext/Status
            // when the latch is updated (then cleared by the next Reset Ext/Status).
            // WR15 D0 is not an External/Status enable and D2 is the Z85C30
            // SDLC frame-status FIFO enable, not an enable for RR0 D2.
            // Channel A
            if (wr1_a[0]) begin
                if ((rr0_a & wr15_a & WR15_EXT_STATUS_MASK) !=
                    (ext_latch_a & wr15_a & WR15_EXT_STATUS_MASK))
                    ext_ip_a <= 1'b1;
            end
            // Channel B
            if (wr1_b[0]) begin
                if ((rr0_b & wr15_b & WR15_EXT_STATUS_MASK) !=
                    (ext_latch_b & wr15_b & WR15_EXT_STATUS_MASK))
                    ext_ip_b <= 1'b1;
            end

            // Send Break changes at a transmit-bit boundary, independently
            // of character boundaries.  Keep it registered so WR5 writes
            // cannot create a sub-clock glitch on TxD.
            if (tx_clk_en_a)
                tx_break_a <= wr5_a[4];
            if (tx_clk_en_b)
                tx_break_b <= wr5_b[4];

            // TX: Load the shifter when data is pending and the transmitter
            // is enabled.  The start bit is aligned to a complete data-bit
            // boundary so its first half cannot be shortened by clock phase.
            if (tx_pending_a && !tx_active_a && wr5_a[3] &&
                tx_clk_en_a) begin
                tx_shift_a <= tx_async_frame(tx_data_a, wr4_a, wr5_a);
                tx_half_remaining_a <= tx_frame_half_bits(tx_data_a, wr4_a, wr5_a);
                tx_half_phase_a <= 1'b0;
                tx_active_a <= 1'b1;
`ifndef SYNTHESIS
                if (scc_debug)
                    $display("%s_TX_A: Starting transmission of 0x%02x '%c'", INST_NAME,
                        tx_data_a, (tx_data_a >= 8'h20 && tx_data_a < 8'h7F) ? tx_data_a : 8'h2E);
`endif
            end

            if (tx_pending_b && !tx_active_b && wr5_b[3] &&
                tx_clk_en_b) begin
                tx_shift_b <= tx_async_frame(tx_data_b, wr4_b, wr5_b);
                tx_half_remaining_b <= tx_frame_half_bits(tx_data_b, wr4_b, wr5_b);
                tx_half_phase_b <= 1'b0;
                tx_active_b <= 1'b1;
`ifndef SYNTHESIS
                if (scc_debug)
                    $display("%s_TX: Starting transmission of 0x%02x '%c'", INST_NAME,
                        tx_data_b, (tx_data_b >= 8'h20 && tx_data_b < 8'h7F) ? tx_data_b : 8'h2E);
`endif
            end

            // Shift at half-data-bit resolution.  This gives the SCC's 1.5
            // stop selection an exact endpoint while ordinary bits still
            // shift only on every second enable.  WR8 is a one-byte holding
            // register, so an enabled queued character transfers at the exact
            // stop boundary with no inserted idle cell.  Clearing WR5 Tx
            // Enable lets the current character finish and leaves the queued
            // byte in WR8 until software enables the transmitter again.
            if (tx_active_a && tx_half_clk_en_a) begin
                if (tx_half_remaining_a == 5'd1) begin
                    if (tx_pending_a && wr5_a[3]) begin
                        tx_shift_a <= tx_async_frame(tx_data_a, wr4_a, wr5_a);
                        tx_half_remaining_a <= tx_frame_half_bits(tx_data_a, wr4_a, wr5_a);
                        tx_half_phase_a <= 1'b0;
                    end else begin
                        tx_shift_a <= 12'hfff;
                        tx_half_remaining_a <= 5'd0;
                        tx_half_phase_a <= 1'b0;
                        tx_active_a <= 1'b0;
                    end
                end else begin
                    tx_half_remaining_a <= tx_half_remaining_a - 1'b1;
                    tx_half_phase_a <= ~tx_half_phase_a;
                    if (tx_half_phase_a)
                        tx_shift_a <= {1'b1, tx_shift_a[11:1]};
                end
            end

            if (tx_active_b && tx_half_clk_en_b) begin
                if (tx_half_remaining_b == 5'd1) begin
                    if (tx_pending_b && wr5_b[3]) begin
                        tx_shift_b <= tx_async_frame(tx_data_b, wr4_b, wr5_b);
                        tx_half_remaining_b <= tx_frame_half_bits(tx_data_b, wr4_b, wr5_b);
                        tx_half_phase_b <= 1'b0;
                    end else begin
                        tx_shift_b <= 12'hfff;
                        tx_half_remaining_b <= 5'd0;
                        tx_half_phase_b <= 1'b0;
                        tx_active_b <= 1'b0;
                    end
                end else begin
                    tx_half_remaining_b <= tx_half_remaining_b - 1'b1;
                    tx_half_phase_b <= ~tx_half_phase_b;
                    if (tx_half_phase_b)
                        tx_shift_b <= {1'b1, tx_shift_b[11:1]};
                end
            end

            // Bus access
            if (valid && ready && !reset_access_blocked) begin
                if (write) begin
                    // === Write operations ===
                    if (sel_ctrl_a) begin
                        if (reg_ptr_a == 4'd0) begin
                            // WR0: bits[2:0] = register pointer, bits[5:3] = command
                            // "Point High" command (cmd=1) sets bit 3 of register pointer
                            reg_ptr_a <= (wdata_byte[5:3] == 3'd1)
                                ? {1'b1, wdata_byte[2:0]}
                                : {1'b0, wdata_byte[2:0]};
                            if (wdata_byte[7:6] == 2'b11)
                                tx_underrun_eom_a <= 1'b0;
                            // Decode WR0 command bits [5:3]
                            case (wdata_byte[5:3])
                                3'd2: begin  // Reset Ext/Status Interrupts
                                    ext_ip_a <= 1'b0;
                                    ext_latch_a <= rr0_a;  // Latch current status
                                end
                                3'd3: tx_underrun_eom_a <= 1'b1; // Send Abort
                                3'd4: rx_first_armed_a <= 1'b1; // Interrupt on next Rx char
                                3'd5: tx_ip_a <= 1'b0;  // Reset TX Int Pending
                                3'd6: begin  // Error Reset
                                    rx_overrun_a <= 1'b0;
                                    rx_framing_a <= 1'b0;
                                    rx_parity_error_a <= 1'b0;
                                end
                                // Reset Highest IUS only changes an in-service
                                // latch after interrupt acknowledge.  This
                                // model has no interrupt-acknowledge cycle, so
                                // it must not discard a pending source.
                                3'd7: ;
                                default: ;
                            endcase
                        end else begin
                            case (reg_ptr_a)
                                4'd1: begin
                                    // If enabling TX Int (bit 1: 0→1) while buffer empty, set TX IP
                                    // Legacy Z8530 enables against its empty
                                    // holding register.  ESCC entry-empty mode
                                    // cannot interrupt until the first byte
                                    // has been written since channel reset.
                                    if (wdata_byte[1] && !wr1_a[1] &&
                                        (((ADVERTISE_TX_FIFO == 0) && !tx_pending_a) ||
                                         ((ADVERTISE_TX_FIFO != 0) && tx_written_a &&
                                          !wr7p_a[5] &&
                                          (tx_count_a < 3'd4))))
                                        tx_ip_a <= 1'b1;
                                    if (wdata_byte[4:3] == 2'b00)
                                        rx_ip_a <= 1'b0;
                                    else if (wdata_byte[4:3] == 2'b01)
                                        rx_first_armed_a <= 1'b1;
                                    else if (wdata_byte[4:3] == 2'b10 &&
                                             rx_ready_a &&
                                             ((ADVERTISE_TX_FIFO == 0) ||
                                              !wr7p_a[3] ||
                                              (rx_count_a >= 4'd4)))
                                        rx_ip_a <= 1'b1;
                                    wr1_a  <= wdata_byte;
`ifndef SYNTHESIS
                                    if (scc_debug)
                                        $display("SCC_WR1_A: %02x (RxInt=%b%b TxInt=%b ExtInt=%b)", wdata_byte, wdata_byte[4], wdata_byte[3], wdata_byte[1], wdata_byte[0]);
`endif
                                end
                                4'd2: begin
                                    wr2_a <= wdata_byte;
                                    wr2_b <= wdata_byte;
                                end
                                4'd3:  wr3_a  <= wdata_byte;
                                4'd4:  wr4_a  <= wdata_byte;
                                4'd5:  wr5_a  <= wdata_byte;
                                4'd6:  wr6_a  <= wdata_byte;
                                4'd7: begin
                                    // Z85130: WR15 bit 0 redirects WR7 to WR7'
                                    if ((ADVERTISE_TX_FIFO != 0) && wr15_a[0])
                                        wr7p_a <= wdata_byte;
                                    else
                                        wr7_a  <= wdata_byte;
                                end
                                4'd8: begin  // WR8 = TX buffer
                                    tx_byte_out_a <= wdata_byte;
                                    tx_byte_valid_a <= 1'b1;
                                    tx_ip_a <= 1'b0;  // Loading char clears TX IP
                                    // Z85130: WR14 bit 4 = local loopback
                                    if (wr14_a[4])
                                        receive_char_a(wdata_byte, 3'b000, 1'b0);
                                end
                                4'd9: apply_wr9(wdata_byte);
                                4'd10: wr10_a <= wdata_byte;
                                4'd11: wr11_a <= wdata_byte;
                                4'd12: wr12_a <= wdata_byte;
                                4'd13: wr13_a <= wdata_byte;
                                4'd14: wr14_a <= wdata_byte;
                                4'd15: wr15_a <= wdata_byte;
                                default: ;
                            endcase
                            reg_ptr_a <= 4'd0;
                        end
                    end
                    else if (sel_ctrl_b) begin
                        if (reg_ptr_b == 4'd0) begin
                            // WR0: bits[2:0] = register pointer, bits[5:3] = command
                            // "Point High" command (cmd=1) sets bit 3 of register pointer
                            reg_ptr_b <= (wdata_byte[5:3] == 3'd1)
                                ? {1'b1, wdata_byte[2:0]}
                                : {1'b0, wdata_byte[2:0]};
                            if (wdata_byte[7:6] == 2'b11)
                                tx_underrun_eom_b <= 1'b0;
                            // Decode WR0 command bits [5:3]
                            case (wdata_byte[5:3])
                                3'd2: begin  // Reset Ext/Status Interrupts
                                    ext_ip_b <= 1'b0;
                                    ext_latch_b <= rr0_b;  // Latch current status
                                end
                                3'd3: tx_underrun_eom_b <= 1'b1; // Send Abort
                                3'd4: rx_first_armed_b <= 1'b1; // Interrupt on next Rx char
                                3'd5: tx_ip_b <= 1'b0;  // Reset TX Int Pending
                                3'd6: begin  // Error Reset
                                    rx_overrun_b <= 1'b0;
                                    rx_framing_b <= 1'b0;
                                    rx_parity_error_b <= 1'b0;
                                end
                                3'd7: ; // Reset Highest IUS; no modeled IUS latch.
                                default: ;
                            endcase
                        end else begin
                            case (reg_ptr_b)
                                4'd1: begin
                                    // Preserve the legacy empty-holding-register
                                    // behavior; gate enhanced entry-empty mode
                                    // until the first post-reset write.
                                    if (wdata_byte[1] && !wr1_b[1] &&
                                        (((ADVERTISE_TX_FIFO == 0) && !tx_pending_b) ||
                                         ((ADVERTISE_TX_FIFO != 0) && tx_written_b &&
                                          !wr7p_b[5] &&
                                          (tx_count_b < 3'd4))))
                                        tx_ip_b <= 1'b1;
                                    if (wdata_byte[4:3] == 2'b00)
                                        rx_ip_b <= 1'b0;
                                    else if (wdata_byte[4:3] == 2'b01)
                                        rx_first_armed_b <= 1'b1;
                                    else if (wdata_byte[4:3] == 2'b10 &&
                                             rx_ready_b &&
                                             ((ADVERTISE_TX_FIFO == 0) ||
                                              !wr7p_b[3] ||
                                              (rx_count_b >= 4'd4)))
                                        rx_ip_b <= 1'b1;
                                    wr1_b  <= wdata_byte;
`ifndef SYNTHESIS
                                    if (scc_debug)
                                        $display("SCC_WR1_B: %02x (RxInt=%b%b TxInt=%b ExtInt=%b)", wdata_byte, wdata_byte[4], wdata_byte[3], wdata_byte[1], wdata_byte[0]);
`endif
                                end
                                4'd2: begin
                                    wr2_a <= wdata_byte;
                                    wr2_b <= wdata_byte;
                                end
                                4'd3:  wr3_b  <= wdata_byte;
                                4'd4:  wr4_b  <= wdata_byte;
                                4'd5:  wr5_b  <= wdata_byte;
                                4'd6:  wr6_b  <= wdata_byte;
                                4'd7: begin
                                    if ((ADVERTISE_TX_FIFO != 0) && wr15_b[0])
                                        wr7p_b <= wdata_byte;
                                    else
                                        wr7_b  <= wdata_byte;
                                end
                                4'd8: begin
                                    tx_ip_b <= 1'b0;  // Loading char clears TX IP
                                    tx_byte_out_b <= wdata_byte;
                                    tx_byte_valid_b <= 1'b1;
                                    // Z85130: WR14 bit 4 = local loopback
                                    if (wr14_b[4])
                                        receive_char_b(wdata_byte, 3'b000, 1'b0);
                                end
                                4'd9: apply_wr9(wdata_byte);
                                4'd10: wr10_b <= wdata_byte;
                                4'd11: wr11_b <= wdata_byte;
                                4'd12: wr12_b <= wdata_byte;
                                4'd13: wr13_b <= wdata_byte;
                                4'd14: wr14_b <= wdata_byte;
                                4'd15: wr15_b <= wdata_byte;
                                default: ;
                            endcase
                            reg_ptr_b <= 4'd0;
                        end
                    end
                    else if (sel_data_b) begin
                        tx_ip_b <= 1'b0;  // Loading char clears TX IP
                        tx_byte_out_b <= wdata_byte;
                        tx_byte_valid_b <= 1'b1;
                        // Z85130: WR14 bit 4 = local loopback (data port path)
                        if (wr14_b[4])
                            receive_char_b(wdata_byte, 3'b000, 1'b0);
                    end
                    else if (sel_data_a) begin
                        // Channel A data write (keyboard TX)
                        tx_byte_out_a <= wdata_byte;
                        tx_byte_valid_a <= 1'b1;
                        tx_ip_a <= 1'b0;  // Loading char clears TX IP
                        // Z85130: WR14 bit 4 = local loopback (data port path)
                        if (wr14_a[4])
                            receive_char_a(wdata_byte, 3'b000, 1'b0);
                    end
                end else if (!write) begin
                    // === Read side effects ===
                    if (sel_ctrl_b) begin
                        // if (reg_ptr_b == 4'd0 && rr0_b[0])
                        //     $display("SCC1: RR0 read = %02x (RX AVAIL)", rr0_b);
                        // if (reg_ptr_b == 4'd1)
                        //     $display("SCC1: RR1 read = %02x", rr1_b);
                        reg_ptr_b <= 4'd0;
                    end
                    else if (sel_data_b) begin
                        // if (rx_ready_b)
                        //     $display("SCC_RX_READ: kernel read data=0x%02x '%c'", rx_data_b, rx_data_b);
                        if (rx_count_b != 4'd0 && !rx_fifo_push_b) begin
                            if (rx_count_b > 4'd1) begin
                                rx_data_b <= rx_fifo_b[1];
                                {rx_framing_b, rx_overrun_b,
                                 rx_parity_error_b} <= rx_error_fifo_b[1];
                                for (fifo_index = 1; fifo_index < 7;
                                     fifo_index = fifo_index + 1) begin
                                    if (fifo_index < {28'd0, rx_count_b} - 32'd1) begin
                                        rx_fifo_b[fifo_index] <=
                                            rx_fifo_b[fifo_index + 1];
                                        rx_error_fifo_b[fifo_index] <=
                                            rx_error_fifo_b[fifo_index + 1];
                                    end
                                end
                            end else begin
                                {rx_framing_b, rx_overrun_b,
                                 rx_parity_error_b} <= 3'b000;
                            end
                            rx_count_b <= rx_count_b - 4'd1;
                            rx_ready_b <= (rx_count_b > 4'd1);
                            // Every-character mode remains pending while the
                            // FIFO remains above the selected ESCC threshold.
                            rx_ip_b <= (wr1_b[4:3] == 2'b10) &&
                                (((ADVERTISE_TX_FIFO == 0) || !wr7p_b[3])
                                    ? (rx_count_b > 4'd1)
                                    : (rx_count_b > 4'd4));
                        end
                    end
                    else if (sel_ctrl_a) begin
                        reg_ptr_a <= 4'd0;
                    end
                    else if (sel_data_a) begin
`ifndef SYNTHESIS
                        if ($test$plusargs("trace_input_inject"))
                            $display("SCC0 KBD_POP: data=0x%02x count_before=%0d wr1=0x%02x rx_ip=%0d irq=%0d",
                                     rx_data_a, rx_count_a, wr1_a,
                                     rx_ip_a, scc_irq);
`endif
                        if (rx_count_a != 4'd0 && !rx_fifo_push_a) begin
                            if (rx_count_a > 4'd1) begin
                                rx_data_a <= rx_fifo_a[1];
                                {rx_framing_a, rx_overrun_a,
                                 rx_parity_error_a} <= rx_error_fifo_a[1];
                                for (fifo_index = 1; fifo_index < 7;
                                     fifo_index = fifo_index + 1) begin
                                    if (fifo_index < {28'd0, rx_count_a} - 32'd1) begin
                                        rx_fifo_a[fifo_index] <=
                                            rx_fifo_a[fifo_index + 1];
                                        rx_error_fifo_a[fifo_index] <=
                                            rx_error_fifo_a[fifo_index + 1];
                                    end
                                end
                            end else begin
                                {rx_framing_a, rx_overrun_a,
                                 rx_parity_error_a} <= 3'b000;
                            end
                            rx_count_a <= rx_count_a - 4'd1;
                            rx_ready_a <= (rx_count_a > 4'd1);
                            rx_ip_a <= (wr1_a[4:3] == 2'b10) &&
                                (((ADVERTISE_TX_FIFO == 0) || !wr7p_a[3])
                                    ? (rx_count_a > 4'd1)
                                    : (rx_count_a > 4'd4));
                        end
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
