// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - HPC1 Ethernet + SEEQ 80C03 model
//
// Synthesizable IP20 Ethernet block:
//   - HPC1 Ethernet DMA/status registers
//   - SEEQ 80C03 external register bank
//   - TX descriptor DMA to an external packet byte stream
//   - RX packet byte stream to descriptor DMA
//
// The packet side is deliberately a simple ready/valid byte stream so the same
// RTL can be driven by the Verilator harness or by FPGA MAC/PHY glue.

`default_nettype none

module hpc1_enet #(
    parameter integer CLK_HZ = 50000000
) (
    input  wire        clk,
    input  wire        rst_n,

    // PIO register interface from hpc1.v. Reads are combinational; read side
    // effects and writes happen when pio_valid is asserted for one clock.
    input  wire [11:0] pio_addr,
    input  wire [63:0] pio_wdata,
    output reg  [63:0] pio_rdata,
    input  wire [ 7:0] pio_wmask,
    input  wire        pio_valid,
    input  wire        pio_write,
    input  wire        pio_sel_enet,
    input  wire        pio_sel_hpc_endian,
    input  wire        pio_sel_seeq,

    // Bus master interface to system memory.
    output reg  [31:0] dma_addr,
    output reg  [63:0] dma_wdata,
    input  wire [63:0] dma_rdata,
    output reg  [ 7:0] dma_wmask,
    output reg         dma_valid,
    output reg         dma_write,
    input  wire        dma_ready,
    output wire        dma_active,

    // Packet stream to/from the external harness or MAC.
    output wire [7:0]  tx_data,
    output wire        tx_valid,
    output wire        tx_sof,
    output wire        tx_eof,
    input  wire        tx_ready,

    input  wire [7:0]  rx_data,
    input  wire        rx_valid,
    input  wire        rx_sof,
    input  wire        rx_eof,
    output wire        rx_ready,

    output wire        enet_irq,
    output wire [255:0] debug_probe,
    output wire [767:0] debug_snapshot
);

    localparam [31:0] HPC_STTRDMA     = 32'h0040_0000;
    localparam [31:0] HPC_STRCVDMA    = 32'h0000_4000;
    localparam [31:0] HPC_RBO         = 32'h0000_0008;
    localparam [31:0] HPC_MODNORM     = 32'h0000_0004;
    localparam [31:0] HPC_INTPEND     = 32'h0000_0002;
    localparam [31:0] HPC_ERST        = 32'h0000_0001;

    localparam [7:0]  SEQ_RS_OLD      = 8'h80;
    localparam [7:0]  SEQ_RS_GOOD     = 8'h20;
    localparam [7:0]  SEQ_RS_END      = 8'h10;
    localparam [7:0]  SEQ_RC_RNONE    = 8'h00;
    localparam [7:0]  SEQ_RC_RSB      = 8'h80;
    localparam [7:0]  SEQ_RC_RALL     = 8'h40;
    localparam [7:0]  SEQ_RC_RSMB     = 8'hc0;
    localparam [7:0]  SEQ_RC_INTGOOD  = 8'h20;
    localparam [7:0]  SEQ_RC_INTEND   = 8'h10;
    localparam [7:0]  SEQ_XS_OLD      = 8'h80;
    localparam [7:0]  SEQ_XS_SUCCESS  = 8'h08;
    localparam [7:0]  SEQ_XC_BANK_MASK= 8'h60;
    localparam [7:0]  SEQ_XC_BANK0    = 8'h00;
    localparam [7:0]  SEQ_XC_BANK1    = 8'h20;
    localparam [7:0]  SEQ_XC_BANK2    = 8'h40;
    localparam [7:0]  SEQ_XC_INTGOOD  = 8'h08;
    localparam [7:0]  SEQ_CTL_NOCARR  = 8'h20;
    localparam [7:0]  SEQ_CTL_SHORT   = 8'h10;
    localparam [7:0]  SEQ_CTL_MULTI   = 8'h08;
    localparam [7:0]  SEQ_CTL_SQE     = 8'h04;
    localparam [7:0]  SEQ_CTL_ACOLL   = 8'h02;
    localparam [7:0]  SEQ_CTL_XCOLL   = 8'h01;
    localparam [7:0]  SEQ_FLAGS_NO_SQE= 8'h01;

    localparam [7:0]  RX_STATUS_GOOD  = SEQ_RS_GOOD | SEQ_RS_END;
    localparam [12:0] MAX_RPKT        = 13'd1572;
    localparam [11:0] MAX_FRAME       = 12'd2047;

    localparam [4:0]
        TX_IDLE        = 5'd0,
        TX_FETCH_W0    = 5'd1,
        TX_FETCH_W0_W  = 5'd2,
        TX_FETCH_W1    = 5'd3,
        TX_FETCH_W1_W  = 5'd4,
        TX_FETCH_W2    = 5'd5,
        TX_FETCH_W2_W  = 5'd6,
        TX_READ        = 5'd7,
        TX_READ_W      = 5'd8,
        TX_SEND        = 5'd9,
        TX_DONE        = 5'd10;

    localparam [4:0]
        RX_IDLE        = 5'd0,
        RX_FETCH_W0    = 5'd1,
        RX_FETCH_W0_W  = 5'd2,
        RX_FETCH_W1    = 5'd3,
        RX_FETCH_W1_W  = 5'd4,
        RX_FETCH_W2    = 5'd5,
        RX_FETCH_W2_W  = 5'd6,
        RX_WRITE       = 5'd7,
        RX_WRITE_W     = 5'd8,
        RX_DESC_WRITE  = 5'd9,
        RX_DESC_WRITE_W= 5'd10,
        RX_DONE        = 5'd11,
        RX_READ        = 5'd12,
        RX_PACK_INIT   = 5'd13,
        RX_PACK_PREP   = 5'd14,
        RX_PACK_BYTE   = 5'd15;

    reg [31:0] enet_regs [0:23];
    reg [19:0] enet_intdelay_count;
    reg        enet_intdelay_expired;
    reg [63:0] enet_intdelay_phase;
    reg [31:0] hpc_endian;
    reg [31:0] enet_xmit_fifo [0:15];
    reg [ 7:0] enet_xmit_flags [0:15];
    reg [ 7:0] enet_xmit_flag_latch;
    reg [31:0] enet_recv_fifo [0:15];

    reg [7:0] station_addr [0:5];
    reg [7:0] mcast_lsb    [0:5];
    reg [7:0] mcast_msb    [0:1];
    reg [7:0] seeq_pktgap;
    reg [7:0] seeq_ctl;
    reg [7:0] rx_cmd;
    reg [7:0] rx_stat;
    reg [7:0] tx_cmd;
    reg [7:0] tx_stat;
    reg       intpend;

    reg [11:0] rx_len;
    reg [11:0] rx_wr_pos;
    reg        rx_collecting;
    reg        rx_frame_pending;
    reg [47:0] rx_dst_addr;

    reg [4:0]  tx_state;
    reg [31:0] tx_desc_addr;
    reg [31:0] tx_desc_first;
    reg [31:0] tx_w0;
    reg [31:0] tx_w1;
    reg [31:0] tx_w2;
    reg [12:0] tx_bcnt;
    reg [27:0] tx_bufptr;
    reg [27:0] tx_next;
    reg        tx_eoxp;
    reg        tx_eox;
    reg        tx_xie;
    reg [63:0] tx_rd_buf;
    reg [31:0] tx_rd_addr;
    reg [7:0]  tx_data_r;
    reg        tx_valid_r;
    reg        tx_sof_r;
    reg        tx_eof_r;
    reg        tx_first_byte;
    reg        tx_new_packet_pending;

    reg [4:0]  rx_state;
    reg [31:0] rx_desc_addr;
    reg [31:0] rx_w0;
    reg [31:0] rx_w1;
    reg [31:0] rx_w2;
    reg [31:0] tx_next_word_tmp;
    reg [31:0] rx_next_word_tmp;
    reg [12:0] rx_desc_bcnt;
    reg [27:0] rx_bufptr;
    reg [27:0] rx_next;
    reg        rx_eor;
    reg [11:0] rx_dma_pos;
    reg [12:0] rx_total;
    reg [31:0] rx_pack_base;
    reg [63:0] rx_pack_data;
    reg [ 7:0] rx_pack_wmask;
    reg [ 3:0] rx_pack_limit;
    reg [ 3:0] rx_pack_count;
    reg [11:0] rx_pack_pos;
    reg [10:0] rx_frame_raddr;
    reg [15:0] dbg_rx_stream_frames;
    reg [15:0] dbg_rx_pending_frames;
    reg [15:0] dbg_rx_dma_done;
    reg [15:0] dbg_rx_desc_fail;
`ifdef KCU105_CYAN_SOC_DEBUG
    reg [15:0] dbg_tx_stream_frames;
    reg [ 5:0] dbg_tx_sample_count;
    reg [191:0] dbg_tx_first24;
    reg        dbg_tx_seen_frame;
    reg        dbg_tx_sample_overflow;
    reg        dbg_tx_desc_valid;
    reg        dbg_tx_rd0_valid;
    reg        dbg_tx_rd1_valid;
    reg [31:0] dbg_tx_desc_addr;
    reg [31:0] dbg_tx_desc_first;
    reg [31:0] dbg_tx_w0;
    reg [31:0] dbg_tx_w1;
    reg [31:0] dbg_tx_w2;
    reg [27:0] dbg_tx_bufptr;
    reg [12:0] dbg_tx_bcnt;
    reg [31:0] dbg_tx_rd_addr0;
    reg [31:0] dbg_tx_rd_addr1;
    reg [63:0] dbg_tx_rd_buf0;
    reg [63:0] dbg_tx_rd_buf1;
`endif

`ifndef SYNTHESIS
`ifdef COSIM_TB
    reg trace_enet;
    initial begin
        trace_enet = 1'b0;
        if ($test$plusargs("trace_enet"))
            trace_enet = 1'b1;
    end
`else
    wire trace_enet = 1'b0;
`endif
`endif

    wire [4:0] enet_idx = pio_addr[6:2];
    wire [2:0] seeq_idx = pio_addr[4:2];
    wire [31:0] pio_w32 = pio_addr[2] ? pio_wdata[31:0] : pio_wdata[63:32];
    wire [7:0]  pio_wbyte =
        pio_wmask[7] ? pio_wdata[63:56] :
        pio_wmask[6] ? pio_wdata[55:48] :
        pio_wmask[5] ? pio_wdata[47:40] :
        pio_wmask[4] ? pio_wdata[39:32] :
        pio_wmask[3] ? pio_wdata[31:24] :
        pio_wmask[2] ? pio_wdata[23:16] :
        pio_wmask[1] ? pio_wdata[15:8]  :
                       pio_wdata[7:0];
    wire        pio_seeq_word_write =
        (!pio_addr[2] && (pio_wmask[7:4] == 4'hf)) ||
        ( pio_addr[2] && (pio_wmask[3:0] == 4'hf));
    wire [7:0]  pio_seeq_wbyte = pio_seeq_word_write ? pio_w32[7:0] : pio_wbyte;
    wire tx_state_active = (tx_state != TX_IDLE);
    wire rx_state_active = (rx_state != RX_IDLE);
    wire rx_irq_pending = ((rx_stat & SEQ_RS_OLD) == 8'h00) &&
                          (((rx_cmd & 8'h1f) & (rx_stat & 8'h1f)) != 8'h00);
    wire tx_irq_pending = ((tx_stat & SEQ_XS_OLD) == 8'h00) &&
                          (((tx_cmd & 8'h0f) & (tx_stat & 8'h0f)) != 8'h00);
    wire enet_irq_pending = intpend || rx_irq_pending || tx_irq_pending;
    wire        rx_receiver_enabled = ((rx_cmd & 8'hc0) != SEQ_RC_RNONE);
    wire        rx_start_collect = rx_sof && rx_receiver_enabled;
    wire        rx_frame_store =
        rx_valid && rx_ready && (rx_start_collect || rx_collecting);
    wire [10:0] rx_frame_waddr = rx_sof ? 11'd0 : rx_wr_pos[10:0];
    wire [7:0]  rx_frame_rdata;

    assign tx_data = tx_data_r;
    assign tx_valid = tx_valid_r;
    assign tx_sof = tx_sof_r;
    assign tx_eof = tx_eof_r;
    assign rx_ready = !rx_frame_pending && !rx_state_active &&
                      (!rx_collecting || (rx_wr_pos < MAX_FRAME));
    // Suppress enet_irq after ERST for a holdoff period. Real HPC1/SEEQ
    // doesn't assert interrupt during controller init. Without holdoff,
    // unsolicited LAN traffic triggers interrupts before software
    // finishes populating wired TLB entries and descriptor chains.
    // Holdoff: ~100ms at 25MHz = 2.5M cycles.
    reg [21:0] enet_irq_holdoff;
    assign enet_irq = enet_irq_pending && (enet_irq_holdoff == 22'd0);
    assign dma_active = tx_state_active || rx_state_active;
    wire tx_start = pio_valid && pio_write && pio_sel_enet &&
                    enet_idx == 5'd13 && (pio_w32 & HPC_STTRDMA) != 32'h0;
    wire rx_start = pio_valid && pio_write && pio_sel_enet &&
                    enet_idx == 5'd14 && (pio_w32 & HPC_STRCVDMA) != 32'h0;
    wire tx_dma_pending = tx_start || ((enet_regs[13] & HPC_STTRDMA) != 32'h0);

    // HPC1 Ethernet interrupt delay is a 20-bit one-shot downcounter. Software
    // programs bits [23:4] in 30 ns units and observes TMREXP in bit 24.
    // A fractional phase accumulator gives the deployed 50 MHz clock the
    // required alternating two/one-cycle tick cadence without altering IRQ
    // source selection or the existing controller-initialization holdoff.
    localparam [63:0] ENET_INTDELAY_TICK_THRESHOLD = CLK_HZ * 64'd30;
    wire [64:0] enet_intdelay_phase_sum =
        {1'b0, enet_intdelay_phase} + 65'd1000000000;
    wire enet_intdelay_tick = (CLK_HZ <= 33333333) ||
        (enet_intdelay_phase_sum >=
         {1'b0, ENET_INTDELAY_TICK_THRESHOLD});
    wire [31:0] enet_intdelay_word =
        {7'd0, enet_intdelay_expired, enet_intdelay_count, 4'd0};

    function [31:0] merge32;
        input [31:0] oldv;
        input [31:0] newv;
        input        lower_word;
        input [7:0]  mask;
        reg [31:0] v;
        begin
            v = oldv;
            if (lower_word) begin
                if (mask[3]) v[31:24] = newv[31:24];
                if (mask[2]) v[23:16] = newv[23:16];
                if (mask[1]) v[15:8]  = newv[15:8];
                if (mask[0]) v[7:0]   = newv[7:0];
            end else begin
                if (mask[7]) v[31:24] = newv[31:24];
                if (mask[6]) v[23:16] = newv[23:16];
                if (mask[5]) v[15:8]  = newv[15:8];
                if (mask[4]) v[7:0]   = newv[7:0];
            end
            merge32 = v;
        end
    endfunction

    // Software writes HPC_DELAY_EXPIRED (intdelay[24]) after every ERST as the
    // explicit end of controller initialization.  Honor that driver-visible
    // event instead of forcing software to wait for the conservative startup
    // fallback.  An ERST write retains precedence and restarts the fallback.
    wire [31:0] pio_intdelay_merged =
        merge32(enet_intdelay_word, pio_w32, pio_addr[2], pio_wmask);
    wire [31:0] pio_control_merged =
        merge32(enet_regs[15], pio_w32, pio_addr[2], pio_wmask);
    wire pio_intdelay_expired_write = pio_valid && pio_write &&
        pio_sel_enet && (enet_idx == 5'd11) && pio_intdelay_merged[24];
    wire pio_intdelay_write = pio_valid && pio_write &&
        pio_sel_enet && (enet_idx == 5'd11);
    wire pio_ether_reset_write = pio_valid && pio_write &&
        pio_sel_enet && (enet_idx == 5'd15) &&
        ((pio_control_merged & HPC_ERST) != 32'h0);

    function [31:0] bus_word;
        input [63:0] data;
        input        lower_word;
        begin
            bus_word = lower_word ? data[31:0] : data[63:32];
        end
    endfunction

    function [7:0] bus_byte;
        input [63:0] data;
        input [2:0] off;
        begin
            case (off)
                3'd0: bus_byte = data[63:56];
                3'd1: bus_byte = data[55:48];
                3'd2: bus_byte = data[47:40];
                3'd3: bus_byte = data[39:32];
                3'd4: bus_byte = data[31:24];
                3'd5: bus_byte = data[23:16];
                3'd6: bus_byte = data[15:8];
                default: bus_byte = data[7:0];
            endcase
        end
    endfunction

    task set_dma_write_word;
        input [31:0] addr;
        input [31:0] val;
        begin
            dma_addr <= addr & ~32'h7;
            dma_wdata <= addr[2] ? {32'h0, val} : {val, 32'h0};
            dma_wmask <= addr[2] ? 8'h0f : 8'hf0;
            dma_valid <= 1'b1;
            dma_write <= 1'b1;
        end
    endtask

    function [7:0] seeq_read_byte;
        input [2:0] idx;
        begin
            case (idx)
                3'd0, 3'd1, 3'd2, 3'd3: seeq_read_byte = 8'h0; // collision counters
                3'd4: seeq_read_byte = 8'h0;
                3'd5: seeq_read_byte = SEQ_FLAGS_NO_SQE; // carrier present, no SQE
                3'd6: seeq_read_byte = rx_stat;
                3'd7: seeq_read_byte = tx_stat;
            endcase
        end
    endfunction

    function accept_rx_address;
        input [7:0] cmd;
        input [47:0] dst;
        reg station_match;
        reg broadcast_match;
        reg multicast_match;
        begin
            station_match =
                dst == {station_addr[0], station_addr[1], station_addr[2],
                        station_addr[3], station_addr[4], station_addr[5]};
            broadcast_match = (dst == 48'hffff_ffff_ffff);
            multicast_match = dst[40];
            case (cmd & 8'hc0)
                SEQ_RC_RNONE: accept_rx_address = 1'b0;
                SEQ_RC_RALL:  accept_rx_address = 1'b1;
                SEQ_RC_RSB:   accept_rx_address = station_match || broadcast_match;
                SEQ_RC_RSMB:  accept_rx_address = station_match || broadcast_match || multicast_match;
                default:      accept_rx_address = station_match || broadcast_match;
            endcase
        end
    endfunction

    function rx_pos_uses_frame;
        input [11:0] pos;
        input [11:0] len;
        begin
            rx_pos_uses_frame = (pos >= 12'd2) && (pos < (len + 12'd2));
        end
    endfunction

    function [3:0] rx_chunk_limit;
        input [2:0]  addr_off;
        input [12:0] remaining;
        reg [3:0] boundary;
        begin
            boundary = 4'd8 - {1'b0, addr_off};
            if (remaining < {9'd0, boundary})
                rx_chunk_limit = remaining[3:0];
            else
                rx_chunk_limit = boundary;
        end
    endfunction

    cyan_sync_byte_ram #(.ADDR_W(11)) rx_frame_ram (
        .clk  (clk),
        .we   (rx_frame_store),
        .waddr(rx_frame_waddr),
        .wdata(rx_data),
        .raddr(rx_frame_raddr),
        .rdata(rx_frame_rdata)
    );

    reg [31:0] enet_read_word;
    always @(*) begin
        enet_read_word = enet_regs[enet_idx];
        case (enet_idx)
            5'd6: begin
                enet_read_word = enet_regs[6];
                enet_read_word[31:24] = enet_xmit_flags[enet_regs[6][5:2]];
            end
            5'd7: begin
                enet_read_word = enet_xmit_fifo[enet_regs[6][5:2]];
            end
            5'd11: begin
                enet_read_word = enet_intdelay_word;
            end
            5'd13: begin
                enet_read_word = enet_regs[13];
            end
            5'd14: begin
                enet_read_word = enet_regs[14];
            end
            5'd15: begin
                enet_read_word = (enet_regs[15] & ~(HPC_INTPEND | HPC_ERST)) |
                                 (enet_irq_pending ? HPC_INTPEND : 32'h0);
            end
            5'd23: begin
                enet_read_word = enet_recv_fifo[enet_regs[22][5:2]];
            end
            default: begin
                enet_read_word = enet_regs[enet_idx];
            end
        endcase

        if (pio_sel_enet)
            pio_rdata = {enet_read_word, enet_read_word};
        else if (pio_sel_hpc_endian)
            pio_rdata = {hpc_endian, hpc_endian};
        else if (pio_sel_seeq)
            pio_rdata = {2{24'h0, seeq_read_byte(seeq_idx)}};
        else
            pio_rdata = 64'h0;
    end

    integer i;
    reg [31:0] merged;
    wire [31:0] rx_dma_cur_addr = {4'b0, rx_bufptr} + {20'h0, rx_dma_pos};
    wire [12:0] rx_dma_remaining = rx_total - {1'b0, rx_dma_pos};
    wire [3:0]  rx_dma_pack_limit =
        rx_chunk_limit(rx_dma_cur_addr[2:0], rx_dma_remaining);
    wire [12:0] rx_dma_next_pos =
        {1'b0, rx_dma_pos} + {9'd0, rx_pack_limit};
    wire [31:0] rx_pack_cur_addr = {4'b0, rx_bufptr} + {20'h0, rx_pack_pos};

    task rx_pack_store_byte;
        input [7:0] val;
        begin
            case (rx_pack_cur_addr[2:0])
                3'd0: begin rx_pack_data[63:56] <= val; rx_pack_wmask[7] <= 1'b1; end
                3'd1: begin rx_pack_data[55:48] <= val; rx_pack_wmask[6] <= 1'b1; end
                3'd2: begin rx_pack_data[47:40] <= val; rx_pack_wmask[5] <= 1'b1; end
                3'd3: begin rx_pack_data[39:32] <= val; rx_pack_wmask[4] <= 1'b1; end
                3'd4: begin rx_pack_data[31:24] <= val; rx_pack_wmask[3] <= 1'b1; end
                3'd5: begin rx_pack_data[23:16] <= val; rx_pack_wmask[2] <= 1'b1; end
                3'd6: begin rx_pack_data[15:8]  <= val; rx_pack_wmask[1] <= 1'b1; end
                default: begin rx_pack_data[7:0] <= val; rx_pack_wmask[0] <= 1'b1; end
            endcase
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 24; i = i + 1)
                enet_regs[i] <= 32'h0;
            for (i = 0; i < 16; i = i + 1) begin
                enet_xmit_fifo[i] <= 32'h0;
                enet_xmit_flags[i] <= 8'h0;
                enet_recv_fifo[i] <= 32'h0;
            end
            enet_xmit_flag_latch <= 8'h0;
            hpc_endian <= 32'h0;
            for (i = 0; i < 6; i = i + 1) begin
                station_addr[i] <= 8'h0;
                mcast_lsb[i] <= 8'h0;
            end
            mcast_msb[0] <= 8'h0;
            mcast_msb[1] <= 8'h0;
            seeq_pktgap <= 8'h0;
            seeq_ctl <= SEQ_CTL_NOCARR | SEQ_CTL_SHORT | SEQ_CTL_SQE |
                        SEQ_CTL_ACOLL | SEQ_CTL_XCOLL;
            rx_cmd <= 8'h0;
            rx_stat <= SEQ_RS_OLD;
            tx_cmd <= 8'h0;
            tx_stat <= SEQ_XS_OLD | SEQ_XS_SUCCESS;
            intpend <= 1'b0;
            enet_irq_holdoff <= 22'h3FFFFF;
            enet_intdelay_count <= 20'd0;
            enet_intdelay_expired <= 1'b1;
            enet_intdelay_phase <= 64'd0;
            rx_len <= 12'h0;
            rx_wr_pos <= 12'h0;
            rx_collecting <= 1'b0;
            rx_frame_pending <= 1'b0;
            rx_dst_addr <= 48'h0;
            tx_state <= TX_IDLE;
            tx_desc_addr <= 32'h0;
            tx_desc_first <= 32'h0;
            tx_w0 <= 32'h0;
            tx_w1 <= 32'h0;
            tx_w2 <= 32'h0;
            tx_bcnt <= 13'h0;
            tx_bufptr <= 28'h0;
            tx_next <= 28'h0;
            tx_eoxp <= 1'b0;
            tx_eox <= 1'b0;
            tx_xie <= 1'b0;
            tx_rd_buf <= 64'h0;
            tx_rd_addr <= 32'h0;
            tx_data_r <= 8'h0;
            tx_valid_r <= 1'b0;
            tx_sof_r <= 1'b0;
            tx_eof_r <= 1'b0;
            tx_first_byte <= 1'b1;
            tx_new_packet_pending <= 1'b0;
            rx_state <= RX_IDLE;
            rx_desc_addr <= 32'h0;
            rx_w0 <= 32'h0;
            rx_w1 <= 32'h0;
            rx_w2 <= 32'h0;
            rx_desc_bcnt <= 13'h0;
            rx_bufptr <= 28'h0;
            rx_next <= 28'h0;
            rx_eor <= 1'b0;
            rx_dma_pos <= 12'h0;
            rx_total <= 13'h0;
            rx_pack_base <= 32'h0;
            rx_pack_data <= 64'h0;
            rx_pack_wmask <= 8'h0;
            rx_pack_limit <= 4'h0;
            rx_pack_count <= 4'h0;
            rx_pack_pos <= 12'h0;
            rx_frame_raddr <= 11'h0;
            dbg_rx_stream_frames <= 16'h0;
            dbg_rx_pending_frames <= 16'h0;
            dbg_rx_dma_done <= 16'h0;
            dbg_rx_desc_fail <= 16'h0;
`ifdef KCU105_CYAN_SOC_DEBUG
            dbg_tx_stream_frames <= 16'h0;
            dbg_tx_sample_count <= 6'd0;
            dbg_tx_first24 <= 192'h0;
            dbg_tx_seen_frame <= 1'b0;
            dbg_tx_sample_overflow <= 1'b0;
            dbg_tx_desc_valid <= 1'b0;
            dbg_tx_rd0_valid <= 1'b0;
            dbg_tx_rd1_valid <= 1'b0;
            dbg_tx_desc_addr <= 32'd0;
            dbg_tx_desc_first <= 32'd0;
            dbg_tx_w0 <= 32'd0;
            dbg_tx_w1 <= 32'd0;
            dbg_tx_w2 <= 32'd0;
            dbg_tx_bufptr <= 28'd0;
            dbg_tx_bcnt <= 13'd0;
            dbg_tx_rd_addr0 <= 32'd0;
            dbg_tx_rd_addr1 <= 32'd0;
            dbg_tx_rd_buf0 <= 64'd0;
            dbg_tx_rd_buf1 <= 64'd0;
`endif
            dma_addr <= 32'h0;
            dma_wdata <= 64'h0;
            dma_wmask <= 8'h0;
            dma_valid <= 1'b0;
            dma_write <= 1'b0;
        end else begin
            dma_valid <= 1'b0;
            dma_write <= 1'b0;
            tx_sof_r <= tx_valid_r && !tx_ready ? tx_sof_r : 1'b0;
            tx_eof_r <= tx_valid_r && !tx_ready ? tx_eof_r : 1'b0;

            // Keep timer state changes exclusive.  ERST cancels a live timer;
            // a reload restarts the 30 ns phase; only an otherwise idle cycle
            // may advance the one-shot.
            if (pio_ether_reset_write) begin
                enet_intdelay_count <= 20'd0;
                enet_intdelay_expired <= 1'b1;
                enet_intdelay_phase <= 64'd0;
            end else if (pio_intdelay_write) begin
                enet_intdelay_count <= (pio_intdelay_merged[24] ||
                    (pio_intdelay_merged[23:4] == 20'd0)) ?
                    20'd0 : pio_intdelay_merged[23:4];
                enet_intdelay_expired <= pio_intdelay_merged[24] ||
                    (pio_intdelay_merged[23:4] == 20'd0);
                enet_intdelay_phase <= 64'd0;
            end else begin
                if (CLK_HZ <= 33333333) begin
                    enet_intdelay_phase <= 64'd0;
                end else if (enet_intdelay_tick) begin
                    enet_intdelay_phase <= enet_intdelay_phase_sum -
                                           ENET_INTDELAY_TICK_THRESHOLD;
                end else begin
                    enet_intdelay_phase <= enet_intdelay_phase_sum[63:0];
                end

                if (enet_intdelay_tick && !enet_intdelay_expired) begin
                    if (enet_intdelay_count <= 20'd1) begin
                        enet_intdelay_count <= 20'd0;
                        enet_intdelay_expired <= 1'b1;
                    end else begin
                        enet_intdelay_count <= enet_intdelay_count - 20'd1;
                    end
                end
            end

`ifdef KCU105_CYAN_SOC_DEBUG
            if (tx_valid_r && tx_ready) begin
                if (tx_sof_r) begin
                    dbg_tx_seen_frame <= 1'b1;
                    dbg_tx_sample_overflow <= 1'b0;
                    dbg_tx_sample_count <= 6'd1;
                    dbg_tx_first24 <= {184'd0, tx_data_r};
                end else if (dbg_tx_sample_count < 6'd24) begin
                    dbg_tx_sample_count <= dbg_tx_sample_count + 6'd1;
                    dbg_tx_first24 <= {dbg_tx_first24[183:0], tx_data_r};
                end else begin
                    dbg_tx_sample_overflow <= 1'b1;
                end

                if (tx_eof_r)
                    dbg_tx_stream_frames <= dbg_tx_stream_frames + 16'd1;
            end
`endif

            if (rx_valid && rx_ready) begin
                if (rx_eof)
                    dbg_rx_stream_frames <= dbg_rx_stream_frames + 16'd1;
                if (rx_sof && rx_receiver_enabled) begin
                    rx_wr_pos <= 12'd1;
                    rx_len <= 12'd1;
                    rx_collecting <= 1'b1;
                    rx_dst_addr <= {rx_data, 40'h0};
                end else if (rx_collecting) begin
                    rx_wr_pos <= rx_wr_pos + 12'd1;
                    rx_len <= rx_wr_pos + 12'd1;
                    case (rx_wr_pos)
                        12'd1: rx_dst_addr[39:32] <= rx_data;
                        12'd2: rx_dst_addr[31:24] <= rx_data;
                        12'd3: rx_dst_addr[23:16] <= rx_data;
                        12'd4: rx_dst_addr[15:8]  <= rx_data;
                        12'd5: rx_dst_addr[7:0]   <= rx_data;
                        default: ;
                    endcase
                end
                if (rx_eof && rx_collecting) begin
                    rx_collecting <= 1'b0;
                    rx_frame_pending <= (rx_len >= 12'd13) &&
                                        accept_rx_address(rx_cmd, rx_dst_addr);
                    if ((rx_len >= 12'd13) &&
                        accept_rx_address(rx_cmd, rx_dst_addr))
                        dbg_rx_pending_frames <= dbg_rx_pending_frames + 16'd1;
`ifndef SYNTHESIS
                    $display("ENET_RX_EOF: len=%0d dst=%012x rx_cmd=%02x",
                        rx_len, rx_dst_addr, rx_cmd);
`endif
                end
            end

            if (pio_valid && pio_write) begin
                if (pio_sel_enet) begin
                    merged = merge32(enet_regs[enet_idx], pio_w32,
                                     pio_addr[2], pio_wmask);
                    case (enet_idx)
                        5'd6: begin
                            enet_regs[6] <= merged;
                            enet_xmit_flag_latch <= pio_w32[31:24];
                        end
                        5'd7: begin
                            enet_regs[7] <= merged;
                            enet_xmit_fifo[enet_regs[6][5:2]] <= pio_w32;
                            enet_xmit_flags[enet_regs[6][5:2]] <= enet_xmit_flag_latch;
                        end
                        5'd11: begin
                            // Timer state is updated by the exclusive
                            // reset/write/tick chain above.
                        end
                        5'd13: begin
                            if ((merged & HPC_STTRDMA) != 32'h0) begin
                                enet_regs[13] <= HPC_STTRDMA;
                            end else begin
                                enet_regs[13] <= merged;
                            end
                        end
                        5'd14: begin
                            if ((merged & HPC_STRCVDMA) != 32'h0)
                                enet_regs[14] <= HPC_STRCVDMA;
                            else
                                enet_regs[14] <= merged;
                        end
                        5'd15: begin
                            enet_regs[15] <= merged & ~(HPC_INTPEND | HPC_ERST);
                            if ((merged & HPC_INTPEND) != 32'h0) begin
                                intpend <= 1'b0;
                                rx_stat <= rx_stat | SEQ_RS_OLD;
                                tx_stat <= tx_stat | SEQ_XS_OLD;
                            end
                            if ((merged & HPC_ERST) != 32'h0) begin
                                tx_state <= TX_IDLE;
                                rx_state <= RX_IDLE;
                                tx_valid_r <= 1'b0;
                                tx_new_packet_pending <= 1'b0;
                                rx_collecting <= 1'b0;
                                rx_frame_pending <= 1'b0;
                                rx_len <= 12'h0;
                                rx_wr_pos <= 12'h0;
                                intpend <= 1'b0;
                                enet_irq_holdoff <= 22'h3FFFFF;
                                enet_regs[4]  <= 32'h0; // nxbdp
                                enet_regs[8]  <= 32'h0; // cxbdp
                                enet_regs[9]  <= 32'h0; // cpfxbdp
                                enet_regs[10] <= 32'h0; // ppfxbdp
                                enet_regs[13] <= 32'h0; // trstat
                                enet_regs[14] <= 32'h0; // rcvstat
                                enet_regs[20] <= 32'h0; // nrbdp
                                enet_regs[21] <= 32'h0; // crbdp
                                rx_cmd <= 8'h0;
                                tx_cmd <= 8'h0;
                                rx_stat <= SEQ_RS_OLD;
                                tx_stat <= SEQ_XS_OLD;
`ifdef KCU105_CYAN_SOC_DEBUG
                                // Keep KCU debug captures sticky across the PROM's
                                // post-timeout ERST so VIO can inspect the failed TX.
`endif
                            end
                        end
                        5'd23: begin
                            enet_regs[23] <= merged;
                            enet_recv_fifo[enet_regs[22][13:10]] <= pio_w32;
                        end
                        default: enet_regs[enet_idx] <= merged;
                    endcase
                end else if (pio_sel_hpc_endian) begin
                    hpc_endian <= merge32(hpc_endian, pio_w32, pio_addr[2], pio_wmask);
                end else if (pio_sel_seeq) begin
                    case (seeq_idx)
                        3'd0, 3'd1, 3'd2, 3'd3, 3'd4, 3'd5: begin
                            case (tx_cmd & SEQ_XC_BANK_MASK)
                                SEQ_XC_BANK0: station_addr[seeq_idx] <= pio_seeq_wbyte;
                                SEQ_XC_BANK1: mcast_lsb[seeq_idx] <= pio_seeq_wbyte;
                                SEQ_XC_BANK2: begin
                                    if (seeq_idx < 3'd2)
                                        mcast_msb[seeq_idx[0]] <= pio_seeq_wbyte;
                                    else if (seeq_idx == 3'd2)
                                        seeq_pktgap <= pio_seeq_wbyte;
                                    else if (seeq_idx == 3'd3)
                                        seeq_ctl <= pio_seeq_wbyte;
                                end
                                default: ;
                            endcase
                        end
                        3'd6: rx_cmd <= pio_seeq_wbyte;
                        3'd7: tx_cmd <= pio_seeq_wbyte;
                    endcase
                end
            end

            if (pio_valid && !pio_write && pio_sel_seeq) begin
                if (seeq_idx == 3'd6) begin
                    rx_stat <= rx_stat | SEQ_RS_OLD;
                    intpend <= 1'b0;
                end
                if (seeq_idx == 3'd7) begin
                    tx_stat <= tx_stat | SEQ_XS_OLD;
                    intpend <= 1'b0;
                end
            end

            if (tx_state == TX_IDLE && rx_state == RX_IDLE && tx_dma_pending) begin
                tx_desc_addr <= enet_regs[4]; // nxbdp
                tx_desc_first <= enet_regs[4];
                enet_regs[8] <= enet_regs[4]; // cxbdp
                enet_regs[9] <= enet_regs[4]; // cpfxbdp
                enet_regs[10] <= enet_regs[4]; // ppfxbdp
                tx_first_byte <= 1'b1;
                tx_new_packet_pending <= 1'b0;
                tx_stat <= SEQ_XS_OLD;
                tx_state <= TX_FETCH_W0;
`ifndef SYNTHESIS
                if (trace_enet)
                    $display("ENET_TX_START nxbdp=%08x trstat=%08x",
                             enet_regs[4], enet_regs[13]);
`endif
            end

            if (rx_start && rx_state == RX_IDLE)
                enet_regs[14] <= HPC_STRCVDMA;

            // The driver explicitly ends initialization through
            // intdelay[24].  Retain the bounded countdown only as a fallback
            // for PROM/software sequences that omit that write.
            if (pio_intdelay_expired_write)
                enet_irq_holdoff <= 22'd0;
            else if (enet_irq_holdoff != 22'd0 &&
                     !pio_ether_reset_write)
                enet_irq_holdoff <= enet_irq_holdoff - 22'd1;

            case (tx_state)
                TX_IDLE: begin
                end
                TX_FETCH_W0: begin
                    dma_addr <= tx_desc_addr & ~32'h7;
                    dma_valid <= 1'b1;
                    dma_write <= 1'b0;
                    tx_state <= TX_FETCH_W0_W;
                end
                TX_FETCH_W0_W: begin
                    if (dma_ready) begin
                        tx_w0 <= bus_word(dma_rdata, tx_desc_addr[2]);
                        tx_state <= TX_FETCH_W1;
                    end else begin
                        dma_addr <= tx_desc_addr & ~32'h7;
                        dma_valid <= 1'b1;
                    end
                end
                TX_FETCH_W1: begin
                    dma_addr <= (tx_desc_addr + 32'd4) & ~32'h7;
                    dma_valid <= 1'b1;
                    dma_write <= 1'b0;
                    tx_state <= TX_FETCH_W1_W;
                end
                TX_FETCH_W1_W: begin
                    if (dma_ready) begin
                        tx_w1 <= bus_word(dma_rdata, tx_desc_addr[2] ^ 1'b1);
                        tx_state <= TX_FETCH_W2;
                    end else begin
                        dma_addr <= (tx_desc_addr + 32'd4) & ~32'h7;
                        dma_valid <= 1'b1;
                    end
                end
                TX_FETCH_W2: begin
                    dma_addr <= (tx_desc_addr + 32'd8) & ~32'h7;
                    dma_valid <= 1'b1;
                    dma_write <= 1'b0;
                    tx_state <= TX_FETCH_W2_W;
                end
                TX_FETCH_W2_W: begin
                    if (dma_ready) begin
                        tx_w2 <= bus_word(dma_rdata, tx_desc_addr[2]);
                        tx_bcnt <= tx_w0[12:0];
                        tx_xie <= tx_w0[15];
                        tx_eoxp <= tx_w0[31];
                        tx_eox <= tx_w1[31];
                        tx_bufptr <= tx_w1[27:0];
                        enet_regs[3] <= {4'b0, tx_w1[27:0]}; // cxbp
                        tx_next_word_tmp = bus_word(dma_rdata, tx_desc_addr[2]);
                        tx_next <= tx_next_word_tmp[27:0];
                        if (tx_new_packet_pending && tx_w0[12:0] != 13'h0) begin
                            enet_regs[8] <= tx_desc_addr;    // cxbdp
                            enet_regs[9] <= tx_desc_addr;    // cpfxbdp
                            enet_regs[10] <= tx_desc_first;  // ppfxbdp
                            tx_desc_first <= tx_desc_addr;
                            tx_new_packet_pending <= 1'b0;
                        end
`ifdef KCU105_CYAN_SOC_DEBUG
                        if (tx_w0[12:0] != 13'h0) begin
                            dbg_tx_desc_valid <= 1'b1;
                            dbg_tx_rd0_valid <= 1'b0;
                            dbg_tx_rd1_valid <= 1'b0;
                            dbg_tx_desc_addr <= tx_desc_addr;
                            dbg_tx_desc_first <= tx_desc_first;
                            dbg_tx_w0 <= tx_w0;
                            dbg_tx_w1 <= tx_w1;
                            dbg_tx_w2 <= tx_next_word_tmp;
                            dbg_tx_bufptr <= tx_w1[27:0];
                            dbg_tx_bcnt <= tx_w0[12:0];
                            dbg_tx_rd_addr0 <= 32'd0;
                            dbg_tx_rd_addr1 <= 32'd0;
                            dbg_tx_rd_buf0 <= 64'd0;
                            dbg_tx_rd_buf1 <= 64'd0;
                        end
`endif
`ifndef SYNTHESIS
                        if (trace_enet)
                            $display("ENET_TX_DESC desc=%08x w0=%08x w1=%08x w2=%08x bcnt=%0d eoxp=%0b eox=%0b next=%08x",
                                     tx_desc_addr, tx_w0, tx_w1,
                                     tx_next_word_tmp, tx_w0[12:0],
                                     tx_w0[31], tx_w1[31],
                                     {4'b0, tx_next_word_tmp[27:0]});
`endif
                        // Firmware descriptors do not set the driver's software
                        // x_inuse bit; hardware transmits the active descriptor.
                        if (tx_w0[12:0] != 13'h0)
                            tx_state <= TX_READ;
                        else begin
                            enet_regs[13] <= 32'h0;
                            tx_new_packet_pending <= 1'b0;
                            tx_state <= TX_IDLE;
                        end
                    end else begin
                        dma_addr <= (tx_desc_addr + 32'd8) & ~32'h7;
                        dma_valid <= 1'b1;
                    end
                end
                TX_READ: begin
                    dma_addr <= {4'b0, tx_bufptr} & ~32'h7;
                    dma_valid <= 1'b1;
                    dma_write <= 1'b0;
                    tx_state <= TX_READ_W;
                end
                TX_READ_W: begin
                    if (dma_ready) begin
                        tx_rd_buf <= dma_rdata;
                        tx_rd_addr <= {4'b0, tx_bufptr} & ~32'h7;
`ifdef KCU105_CYAN_SOC_DEBUG
                        if (dbg_tx_desc_valid && !dbg_tx_rd0_valid) begin
                            dbg_tx_rd0_valid <= 1'b1;
                            dbg_tx_rd_addr0 <= {4'b0, tx_bufptr} & ~32'h7;
                            dbg_tx_rd_buf0 <= dma_rdata;
                        end else if (dbg_tx_desc_valid && !dbg_tx_rd1_valid) begin
                            dbg_tx_rd1_valid <= 1'b1;
                            dbg_tx_rd_addr1 <= {4'b0, tx_bufptr} & ~32'h7;
                            dbg_tx_rd_buf1 <= dma_rdata;
                        end
`endif
                        tx_state <= TX_SEND;
                    end else begin
                        dma_addr <= {4'b0, tx_bufptr} & ~32'h7;
                        dma_valid <= 1'b1;
                    end
                end
                TX_SEND: begin
                    if (tx_valid_r) begin
                        if (tx_ready) begin
                            tx_valid_r <= 1'b0;
                            tx_bufptr <= tx_bufptr + 28'd1;
                            tx_bcnt <= tx_bcnt - 13'd1;
                            tx_first_byte <= 1'b0;
                            if (tx_bcnt == 13'd1) begin
                                enet_regs[4] <= {4'b0, tx_next}; // nxbdp
                                if (tx_eoxp) begin
                                    if (tx_eox) begin
                                        enet_regs[8] <= tx_desc_addr;  // cxbdp
                                        enet_regs[9] <= tx_desc_addr;  // cpfxbdp
                                    end else begin
                                        tx_new_packet_pending <= 1'b1;
                                    end
                                end
`ifndef SYNTHESIS
                                if (trace_enet)
                                    $display("ENET_TX_DESC_DONE desc=%08x first=%08x next=%08x eoxp=%0b eox=%0b cpfx_new=%08x",
                                             tx_desc_addr, tx_desc_first,
                                             {4'b0, tx_next}, tx_eoxp, tx_eox,
                                             tx_eoxp && tx_eox ? tx_desc_first :
                                             enet_regs[9]);
`endif
                                if (tx_eox) begin
                                    tx_state <= TX_DONE;
                                end else begin
                                    tx_desc_addr <= {4'b0, tx_next};
                                    if (tx_eoxp) begin
                                        tx_first_byte <= 1'b1;
                                    end
                                    tx_state <= TX_FETCH_W0;
                                end
                            end else if (tx_bufptr[2:0] == 3'd7) begin
                                tx_state <= TX_READ;
                            end
                        end
                    end else begin
                        tx_data_r <= bus_byte(tx_rd_buf, tx_bufptr[2:0]);
                        tx_sof_r <= tx_first_byte;
                        tx_eof_r <= (tx_bcnt == 13'd1) && tx_eoxp;
                        tx_valid_r <= 1'b1;
                    end
                end
                TX_DONE: begin
                    enet_regs[4] <= {4'b0, tx_next};  // nxbdp
                    enet_regs[8] <= tx_desc_addr;     // cxbdp
                    enet_regs[9] <= tx_desc_addr;     // cpfxbdp/PROM cleanup fence
                    enet_regs[13] <= {8'h0, SEQ_XS_SUCCESS, 16'h0};
                    tx_stat <= SEQ_XS_SUCCESS;
`ifndef SYNTHESIS
                    if (trace_enet)
                        $display("ENET_TX_DONE first=%08x nxbdp=%08x status=%08x",
                                 tx_desc_first, {4'b0, tx_next},
                                 {8'h0, SEQ_XS_SUCCESS, 16'h0});
`endif
                    if (tx_xie)
                        intpend <= 1'b1;
                    tx_new_packet_pending <= 1'b0;
                    tx_state <= TX_IDLE;
                end
                default: tx_state <= TX_IDLE;
            endcase

            case (rx_state)
                RX_IDLE: begin
                    if (!tx_dma_pending && rx_frame_pending &&
                        ((enet_regs[14] & HPC_STRCVDMA) != 32'h0) &&
                        (enet_regs[20] != 32'h0)) begin
                        rx_desc_addr <= enet_regs[20]; // nrbdp
                        rx_total <= {1'b0, rx_len} + 13'd3;
                        rx_state <= RX_FETCH_W0;
`ifndef SYNTHESIS
                        $display("ENET_RX_DMA_START: nrbdp=%08x rx_len=%0d",
                            enet_regs[20], rx_len);
`endif
                    end
                end
                RX_FETCH_W0: begin
                    dma_addr <= rx_desc_addr & ~32'h7;
                    dma_valid <= 1'b1;
                    dma_write <= 1'b0;
                    rx_state <= RX_FETCH_W0_W;
                end
                RX_FETCH_W0_W: begin
                    if (dma_ready) begin
                        rx_w0 <= bus_word(dma_rdata, rx_desc_addr[2]);
                        rx_state <= RX_FETCH_W1;
                    end else begin
                        dma_addr <= rx_desc_addr & ~32'h7;
                        dma_valid <= 1'b1;
                    end
                end
                RX_FETCH_W1: begin
                    dma_addr <= (rx_desc_addr + 32'd4) & ~32'h7;
                    dma_valid <= 1'b1;
                    dma_write <= 1'b0;
                    rx_state <= RX_FETCH_W1_W;
                end
                RX_FETCH_W1_W: begin
                    if (dma_ready) begin
                        rx_w1 <= bus_word(dma_rdata, rx_desc_addr[2] ^ 1'b1);
                        rx_state <= RX_FETCH_W2;
                    end else begin
                        dma_addr <= (rx_desc_addr + 32'd4) & ~32'h7;
                        dma_valid <= 1'b1;
                    end
                end
                RX_FETCH_W2: begin
                    dma_addr <= (rx_desc_addr + 32'd8) & ~32'h7;
                    dma_valid <= 1'b1;
                    dma_write <= 1'b0;
                    rx_state <= RX_FETCH_W2_W;
                end
                RX_FETCH_W2_W: begin
                    if (dma_ready) begin
                        rx_w2 <= bus_word(dma_rdata, rx_desc_addr[2]);
                        rx_desc_bcnt <= rx_w0[12:0];
                        rx_bufptr <= rx_w1[27:0];
                        rx_eor <= rx_w1[31];
                        rx_next_word_tmp = bus_word(dma_rdata, rx_desc_addr[2]);
                        rx_next <= rx_next_word_tmp[27:0];
                        rx_dma_pos <= 12'h0;
                        if (rx_w0[31] && rx_w0[12:0] >= rx_total)
                            rx_state <= RX_PACK_INIT;
                        else begin
                            // The descriptor cannot consume this buffered
                            // frame. Stop DMA as before, but discard the frame
                            // so a descriptor fault cannot hold rx_ready low
                            // until the whole Ethernet block is reset.
                            enet_regs[14] <= 32'h0;
                            dbg_rx_desc_fail <= dbg_rx_desc_fail + 16'd1;
                            rx_frame_pending <= 1'b0;
                            rx_state <= RX_IDLE;
                        end
                    end else begin
                        dma_addr <= (rx_desc_addr + 32'd8) & ~32'h7;
                        dma_valid <= 1'b1;
                    end
                end
                RX_READ: begin
                    rx_state <= RX_PACK_BYTE;
                end
                RX_PACK_INIT: begin
                    rx_pack_base <= rx_dma_cur_addr & ~32'h7;
                    rx_pack_data <= 64'h0;
                    rx_pack_wmask <= 8'h0;
                    rx_pack_limit <= rx_dma_pack_limit;
                    rx_pack_count <= 4'd0;
                    rx_pack_pos <= rx_dma_pos;
                    rx_state <= RX_PACK_PREP;
                end
                RX_PACK_PREP: begin
                    if (rx_pos_uses_frame(rx_pack_pos, rx_len)) begin
                        rx_frame_raddr <= rx_pack_pos[10:0] - 11'd2;
                        rx_state <= RX_READ;
                    end else begin
                        rx_pack_store_byte((rx_pack_pos < 12'd2) ?
                                           8'h0 : RX_STATUS_GOOD);
                        if (rx_pack_count + 4'd1 >= rx_pack_limit) begin
                            rx_state <= RX_WRITE;
                        end else begin
                            rx_pack_count <= rx_pack_count + 4'd1;
                            rx_pack_pos <= rx_pack_pos + 12'd1;
                        end
                    end
                end
                RX_PACK_BYTE: begin
                    rx_pack_store_byte(rx_frame_rdata);
                    if (rx_pack_count + 4'd1 >= rx_pack_limit) begin
                        rx_state <= RX_WRITE;
                    end else begin
                        rx_pack_count <= rx_pack_count + 4'd1;
                        rx_pack_pos <= rx_pack_pos + 12'd1;
                        rx_state <= RX_PACK_PREP;
                    end
                end
                RX_WRITE: begin
                    dma_addr <= rx_pack_base;
                    dma_wdata <= rx_pack_data;
                    dma_wmask <= rx_pack_wmask;
                    dma_valid <= 1'b1;
                    dma_write <= 1'b1;
                    rx_state <= RX_WRITE_W;
                end
                RX_WRITE_W: begin
                    if (dma_ready) begin
                        if (rx_dma_next_pos >= rx_total)
                            rx_state <= RX_DESC_WRITE;
                        else begin
                            rx_dma_pos <= rx_dma_next_pos[11:0];
                            rx_state <= RX_PACK_INIT;
                        end
                    end else begin
                        dma_addr <= rx_pack_base;
                        dma_wdata <= rx_pack_data;
                        dma_wmask <= rx_pack_wmask;
                        dma_valid <= 1'b1;
                        dma_write <= 1'b1;
                    end
                end
                RX_DESC_WRITE: begin
                    set_dma_write_word(rx_desc_addr,
                        {1'b0, rx_w0[30:13], rx_desc_bcnt - rx_total});
                    rx_state <= RX_DESC_WRITE_W;
                end
                RX_DESC_WRITE_W: begin
                    if (dma_ready)
                        rx_state <= RX_DONE;
                    else
                        set_dma_write_word(rx_desc_addr,
                            {1'b0, rx_w0[30:13], rx_desc_bcnt - rx_total});
                end
                RX_DONE: begin
                    enet_regs[21] <= rx_desc_addr;       // crbdp
                    enet_regs[20] <= {4'b0, rx_next};    // nrbdp
                    enet_regs[18] <= {19'h0, rx_total};  // rbc
                    rx_stat <= SEQ_RS_GOOD | SEQ_RS_END;
`ifndef SYNTHESIS
                    $display("ENET_RX_DMA_DONE: crbdp=%08x nrbdp=%08x rbc=%0d eor=%b bufptr=%08x rx_stat=%02x",
                        rx_desc_addr, {4'b0, rx_next}, rx_total, rx_eor,
                        rx_bufptr, SEQ_RS_GOOD | SEQ_RS_END);
`endif
                    if (rx_eor)
                        enet_regs[14] <= 32'h0;
                    rx_frame_pending <= 1'b0;
                    dbg_rx_dma_done <= dbg_rx_dma_done + 16'd1;
                    rx_state <= RX_IDLE;
                end
                default: rx_state <= RX_IDLE;
            endcase
        end
    end

    wire [31:0] debug_status = {
        4'h0,
        rx_ready,
        rx_frame_pending,
        rx_collecting,
        rx_receiver_enabled,
        rx_state_active,
        tx_state_active,
        tx_dma_pending,
        enet_irq_pending,
        dma_valid,
        dma_write,
        dma_ready,
        rx_w0[31],
        ((enet_regs[14] & HPC_STRCVDMA) != 32'h0),
        rx_state,
        tx_state,
        rx_len[4:0]
    };

    wire [31:0] debug_rx_lengths = {
        3'h0,
        rx_desc_bcnt,
        3'h0,
        rx_total
    };

    assign debug_probe = {
        32'h48504345,
        debug_status,
        dbg_rx_stream_frames,
        dbg_rx_pending_frames,
        dbg_rx_dma_done,
        dbg_rx_desc_fail,
        debug_rx_lengths,
        rx_desc_addr,
        enet_regs[20],
        enet_regs[14]
    };

    wire [31:0] debug_seeq = {
        rx_cmd,
        tx_cmd,
        rx_stat,
        tx_stat
    };

    wire [31:0] debug_dma_ctrl = {
        dma_wmask,
        dma_write,
        dma_valid,
        dma_ready,
        enet_irq,
        enet_irq_pending,
        intpend,
        rx_frame_pending,
        rx_ready,
        tx_state,
        rx_state,
        rx_len[5:0]
    };

`ifdef KCU105_CYAN_SOC_DEBUG
    wire [31:0] debug_tx_capture_status = {
        4'hd,
        dbg_tx_desc_valid,
        dbg_tx_rd0_valid,
        dbg_tx_rd1_valid,
        dbg_tx_seen_frame,
        dbg_tx_sample_overflow,
        tx_state,
        dbg_tx_bcnt,
        dbg_tx_stream_frames[4:0]
    };

    assign debug_snapshot = {
        32'h48504354,            // word 23: "HPCT"
        debug_tx_capture_status, // word 22
        debug_seeq,              // word 21: rx_cmd, tx_cmd, rx_stat, tx_stat
        {station_addr[0], station_addr[1], station_addr[2], station_addr[3]},
        {station_addr[4], station_addr[5], dbg_tx_stream_frames},
        dbg_tx_desc_addr,
        dbg_tx_w0,
        dbg_tx_w1,
        dbg_tx_w2,
        {4'h0, dbg_tx_bufptr},
        dbg_tx_rd_addr0,
        dbg_tx_rd_buf0[63:32],
        dbg_tx_rd_buf0[31:0],
        dbg_tx_rd_addr1,
        dbg_tx_rd_buf1[63:32],
        dbg_tx_rd_buf1[31:0],
        dbg_tx_first24,
        dma_addr,
        debug_dma_ctrl
    };
`else
    assign debug_snapshot = {
        32'h48504345,            // word 23: "HPCE"
        debug_status,            // word 22
        debug_seeq,              // word 21: rx_cmd, tx_cmd, rx_stat, tx_stat
        enet_regs[4],            // word 20
        enet_regs[8],            // word 19
        enet_regs[9],            // word 18
        enet_regs[10],           // word 17
        enet_regs[13],           // word 16
        enet_regs[15],           // word 15
        enet_regs[20],           // word 14
        enet_regs[21],           // word 13
        dma_addr,                // word 12
        debug_dma_ctrl,          // word 11
        tx_desc_addr,            // word 10
        tx_desc_first,           // word 9
        tx_w0,                   // word 8
        tx_w1,                   // word 7
        tx_w2,                   // word 6
        rx_desc_addr,            // word 5
        rx_w0,                   // word 4
        rx_w1,                   // word 3
        rx_w2,                   // word 2
        {dbg_rx_desc_fail, dbg_rx_dma_done},
        {dbg_rx_pending_frames, dbg_rx_stream_frames}
    };
`endif

endmodule

`default_nettype wire
