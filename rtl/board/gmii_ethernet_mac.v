// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - byte-stream Ethernet bridge for KCU105 GMII/SGMII PHY.
//
// The IP20/HPC1 Ethernet model exposes packets as a ready/valid byte stream.
// This block turns that stream into real GMII frames, including preamble, FCS,
// minimum-frame padding, PHY reset, and a small MDIO bring-up sequencer.

`default_nettype none

module gmii_ethernet_mac #(
    parameter integer CLK_HZ = 50000000,
    parameter integer FIFO_ADDR_W = 13,
    parameter integer TX_FIFO_ADDR_W = FIFO_ADDR_W,
    parameter integer RX_FIFO_ADDR_W = FIFO_ADDR_W,
    parameter integer MAX_FRAME_BYTES = 2048,
    parameter integer PHY_RESET_CYCLES = 500000,
    parameter integer PHY_POST_RESET_CYCLES = 500000,
    parameter integer PHY_SW_RESET_CYCLES = 50000000,
    parameter integer PHY_REFRESH_CYCLES = 5000000,
    parameter integer MDC_DIV = 10,
    parameter integer DEBUG_TX_PROBES = 0
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        gmii_rst_n,

    input  wire [7:0]  stream_tx_data,
    input  wire        stream_tx_valid,
    input  wire        stream_tx_sof,
    input  wire        stream_tx_eof,
    output wire        stream_tx_ready,

    output wire [7:0]  stream_rx_data,
    output wire        stream_rx_valid,
    output wire        stream_rx_sof,
    output wire        stream_rx_eof,
    input  wire        stream_rx_ready,

    input  wire        gmii_tx_clk,
    output reg  [7:0]  gmii_txd,
    output reg         gmii_tx_en,
    output wire        gmii_tx_er,

    input  wire        gmii_rx_clk,
    input  wire [7:0]  gmii_rxd,
    input  wire        gmii_rx_dv,
    input  wire        gmii_rx_er,

    output wire        phy_gtx_clk,
    output wire        phy_mdc,
    inout  wire        phy_mdio,
    output wire        phy_rst_n,

    output wire [223:0] debug_probe
);
    localparam [11:0] MIN_FRAME_PAYLOAD = 12'd60;
    localparam integer MAX_FRAME_LIMIT = MAX_FRAME_BYTES;
    localparam [31:0] CRC_INIT = 32'hffff_ffff;
    localparam [31:0] CRC_RESIDUE = 32'hdebb_20e3;
    localparam integer PHY_RESET_LIMIT = PHY_RESET_CYCLES;
    localparam integer PHY_POST_RESET_LIMIT = PHY_POST_RESET_CYCLES;
    localparam integer PHY_SW_RESET_LIMIT = PHY_SW_RESET_CYCLES;
    localparam integer PHY_REFRESH_LIMIT = PHY_REFRESH_CYCLES;
    localparam integer MDC_DIV_LIMIT = MDC_DIV;

    localparam [3:0]
        TX_IDLE       = 4'd0,
        TX_COLLECT    = 4'd1,
        TX_PRE_SEND   = 4'd2,
        TX_DATA_SEND  = 4'd3,
        TX_PAD_SEND   = 4'd4,
        TX_FCS_SEND   = 4'd5,
        TX_IFG        = 4'd10;

    localparam [2:0]
        RX_IDLE        = 3'd0,
        RX_PREAMBLE    = 3'd1,
        RX_FRAME       = 3'd2;

    localparam [1:0]
        RXR_IDLE       = 2'd0,
        RXR_LOAD       = 2'd1,
        RXR_SEND       = 2'd2;

    reg [3:0]  tx_state;

    wire [9:0] tx_fifo_wdata = {stream_tx_sof, stream_tx_eof, stream_tx_data};
    wire [9:0] tx_fifo_rdata;
    wire       tx_fifo_full;
    wire       tx_fifo_empty;
    wire       tx_fifo_valid;
    wire       tx_fifo_rdreq = (tx_state == TX_COLLECT) && tx_fifo_valid;

    gmii_async_fifo #(.WIDTH(10), .ADDR_W(TX_FIFO_ADDR_W)) u_tx_fifo (
        .wr_clk (clk),
        .wr_rst_n(rst_n),
        .wr_data(tx_fifo_wdata),
        .wr_en  (stream_tx_valid && stream_tx_ready),
        .wr_full(tx_fifo_full),
        .wr_count(),
        .rd_clk (gmii_tx_clk),
        .rd_rst_n(gmii_rst_n),
        .rd_data(tx_fifo_rdata),
        .rd_en  (tx_fifo_rdreq),
        .rd_empty(tx_fifo_empty),
        .rd_valid(tx_fifo_valid)
    );

    assign stream_tx_ready = !tx_fifo_full;

    wire [8:0] rx_fifo_wdata;
    reg        rx_fifo_wrreq;
    wire       rx_fifo_full;
    wire [8:0] rx_fifo_rdata;
    wire       rx_fifo_empty;
    wire       rx_fifo_valid;
    wire       rx_fifo_rdreq = stream_rx_valid && stream_rx_ready;
    reg        rx_stream_in_frame;

    gmii_async_fifo #(.WIDTH(9), .ADDR_W(RX_FIFO_ADDR_W)) u_rx_fifo (
        .wr_clk (gmii_rx_clk),
        .wr_rst_n(gmii_rst_n),
        .wr_data(rx_fifo_wdata),
        .wr_en  (rx_fifo_wrreq && !rx_fifo_full),
        .wr_full(rx_fifo_full),
        .wr_count(),
        .rd_clk (clk),
        .rd_rst_n(rst_n),
        .rd_data(rx_fifo_rdata),
        .rd_en  (rx_fifo_rdreq),
        .rd_empty(rx_fifo_empty),
        .rd_valid(rx_fifo_valid)
    );

    assign stream_rx_valid = rx_fifo_valid;
    assign stream_rx_sof   = !rx_stream_in_frame;
    assign stream_rx_eof   = rx_fifo_rdata[8];
    assign stream_rx_data  = rx_fifo_rdata[7:0];

    function [31:0] crc32_next;
        input [31:0] crc;
        input [7:0]  data;
        integer i;
        reg [31:0] c;
        begin
            c = crc;
            for (i = 0; i < 8; i = i + 1) begin
                if ((c[0] ^ data[i]) != 1'b0)
                    c = (c >> 1) ^ 32'hedb8_8320;
                else
                    c = c >> 1;
            end
            crc32_next = c;
        end
    endfunction

    // ---------------------------------------------------------------------
    // Transmit: collect a complete HPC1 packet, then serialize a legal GMII
    // frame. Full-frame buffering prevents underrun if memory DMA stalls.
    // ---------------------------------------------------------------------
    reg [11:0] tx_collect_len;
    reg [11:0] tx_frame_len;
    reg [11:0] tx_payload_sent;
    reg [11:0] tx_pad_sent;
    reg [2:0]  tx_pre_count;
    reg [1:0]  tx_fcs_count;
    reg [4:0]  tx_ifg_count;
    reg [31:0] tx_crc;
    reg [31:0] tx_next_crc;
    reg [31:0] tx_fcs;
    reg        tx_collect_overflow;
    reg [31:0] dbg_stream_tx_bytes;
    reg [15:0] dbg_stream_tx_frames;
    reg        tx_frame_we;
    reg [10:0] tx_frame_waddr;
    reg [7:0]  tx_frame_wdata;
    reg [10:0] tx_frame_raddr;
    wire [7:0] tx_frame_rdata;

    wire tx_fifo_sof = tx_fifo_rdata[9];
    wire tx_fifo_eof = tx_fifo_rdata[8];
    wire [7:0] tx_fifo_data = tx_fifo_rdata[7:0];

    assign gmii_tx_er = 1'b0;
    assign phy_gtx_clk = 1'b0;

    cyan_sync_byte_ram #(.ADDR_W(11)) tx_frame_ram (
        .clk  (gmii_tx_clk),
        .we   (tx_frame_we),
        .waddr(tx_frame_waddr),
        .wdata(tx_frame_wdata),
        .raddr(tx_frame_raddr),
        .rdata(tx_frame_rdata)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dbg_stream_tx_bytes <= 32'd0;
            dbg_stream_tx_frames <= 16'd0;
        end else if (stream_tx_valid && stream_tx_ready) begin
            dbg_stream_tx_bytes <= dbg_stream_tx_bytes + 32'd1;
            if (stream_tx_eof)
                dbg_stream_tx_frames <= dbg_stream_tx_frames + 16'd1;
        end
    end

    always @(posedge gmii_tx_clk or negedge gmii_rst_n) begin
        if (!gmii_rst_n) begin
            tx_state <= TX_IDLE;
            tx_collect_len <= 12'd0;
            tx_frame_len <= 12'd0;
            tx_payload_sent <= 12'd0;
            tx_pad_sent <= 12'd0;
            tx_pre_count <= 3'd0;
            tx_fcs_count <= 2'd0;
            tx_ifg_count <= 5'd0;
            tx_crc <= CRC_INIT;
            tx_fcs <= 32'h0;
            tx_collect_overflow <= 1'b0;
            gmii_txd <= 8'h00;
            gmii_tx_en <= 1'b0;
            tx_frame_we <= 1'b0;
            tx_frame_waddr <= 11'd0;
            tx_frame_wdata <= 8'h00;
            tx_frame_raddr <= 11'd0;
        end else begin
            tx_frame_we <= 1'b0;
            gmii_txd <= 8'h00;
            gmii_tx_en <= 1'b0;

            case (tx_state)
                TX_IDLE: begin
                    tx_collect_len <= 12'd0;
                    tx_collect_overflow <= 1'b0;
                    if (tx_fifo_valid)
                        tx_state <= TX_COLLECT;
                end

                TX_COLLECT: begin
                    if (tx_fifo_valid) begin
                        if (tx_fifo_sof) begin
                            tx_collect_len <= 12'd1;
                            tx_collect_overflow <= 1'b0;
                            tx_frame_we <= 1'b1;
                            tx_frame_waddr <= 11'd0;
                            tx_frame_wdata <= tx_fifo_data;
                            if (tx_fifo_eof) begin
                                tx_frame_len <= 12'd1;
                                tx_pre_count <= 3'd0;
                                tx_state <= TX_PRE_SEND;
                            end
                        end else if (!tx_collect_overflow) begin
                            if (tx_collect_len < MAX_FRAME_LIMIT) begin
                                tx_frame_we <= 1'b1;
                                tx_frame_waddr <= tx_collect_len[10:0];
                                tx_frame_wdata <= tx_fifo_data;
                                tx_collect_len <= tx_collect_len + 12'd1;
                                if (tx_fifo_eof) begin
                                    tx_frame_len <= tx_collect_len + 12'd1;
                                    tx_pre_count <= 3'd0;
                                    tx_state <= TX_PRE_SEND;
                                end
                            end else begin
                                tx_collect_overflow <= 1'b1;
                                if (tx_fifo_eof)
                                    tx_state <= TX_IDLE;
                            end
                        end else if (tx_fifo_eof) begin
                            tx_state <= TX_IDLE;
                        end
                    end
                end

                TX_PRE_SEND: begin
                    gmii_tx_en <= 1'b1;
                    gmii_txd <= (tx_pre_count == 3'd7) ? 8'hd5 : 8'h55;
                    if (tx_pre_count == 3'd7) begin
                        tx_crc <= CRC_INIT;
                        tx_payload_sent <= 12'd0;
                        tx_frame_raddr <= 11'd1;
                        tx_state <= TX_DATA_SEND;
                    end else begin
                        tx_frame_raddr <= 11'd0;
                        tx_pre_count <= tx_pre_count + 3'd1;
                    end
                end

                TX_DATA_SEND: begin
                    gmii_tx_en <= 1'b1;
                    gmii_txd <= tx_frame_rdata;
                    tx_next_crc = crc32_next(tx_crc, tx_frame_rdata);
                    tx_crc <= tx_next_crc;
                    tx_payload_sent <= tx_payload_sent + 12'd1;
                    tx_frame_raddr <= tx_payload_sent[10:0] + 11'd2;
                    if (tx_payload_sent + 12'd1 < tx_frame_len) begin
                        tx_state <= TX_DATA_SEND;
                    end else if (tx_payload_sent + 12'd1 < MIN_FRAME_PAYLOAD) begin
                        tx_pad_sent <= tx_payload_sent + 12'd1;
                        tx_state <= TX_PAD_SEND;
                    end else begin
                        tx_fcs <= ~tx_next_crc;
                        tx_fcs_count <= 2'd0;
                        tx_state <= TX_FCS_SEND;
                    end
                end

                TX_PAD_SEND: begin
                    gmii_tx_en <= 1'b1;
                    gmii_txd <= 8'h00;
                    tx_next_crc = crc32_next(tx_crc, 8'h00);
                    tx_crc <= tx_next_crc;
                    tx_pad_sent <= tx_pad_sent + 12'd1;
                    if (tx_pad_sent + 12'd1 < MIN_FRAME_PAYLOAD) begin
                        tx_state <= TX_PAD_SEND;
                    end else begin
                        tx_fcs <= ~tx_next_crc;
                        tx_fcs_count <= 2'd0;
                        tx_state <= TX_FCS_SEND;
                    end
                end

                TX_FCS_SEND: begin
                    gmii_tx_en <= 1'b1;
                    gmii_txd <= tx_fcs[7:0];
                    if (tx_fcs_count == 2'd3) begin
                        tx_ifg_count <= 5'd0;
                        tx_state <= TX_IFG;
                    end else begin
                        tx_fcs <= {8'h00, tx_fcs[31:8]};
                        tx_fcs_count <= tx_fcs_count + 2'd1;
                    end
                end

                TX_IFG: begin
                    if (tx_ifg_count == 5'd11)
                        tx_state <= TX_IDLE;
                    else
                        tx_ifg_count <= tx_ifg_count + 5'd1;
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    reg [31:0] dbg_gmii_tx_clk_edges;
    reg [31:0] dbg_gmii_tx_bytes;
    reg [15:0] dbg_gmii_tx_frames;
    reg        dbg_gmii_tx_en_prev;
    reg        dbg_tx_seen_frame;
    reg        dbg_tx_sample_overflow;
    reg        dbg_tx_collect_overflow_seen;
    reg [11:0] dbg_tx_active_frame_bytes;
    reg [11:0] dbg_tx_last_frame_bytes;
    reg [5:0]  dbg_tx_sample_count;
    reg [191:0] dbg_tx_first24;

    always @(posedge gmii_tx_clk or negedge gmii_rst_n) begin
        if (!gmii_rst_n) begin
            dbg_gmii_tx_clk_edges <= 32'd0;
            dbg_gmii_tx_bytes <= 32'd0;
            dbg_gmii_tx_frames <= 16'd0;
            dbg_gmii_tx_en_prev <= 1'b0;
            dbg_tx_seen_frame <= 1'b0;
            dbg_tx_sample_overflow <= 1'b0;
            dbg_tx_collect_overflow_seen <= 1'b0;
            dbg_tx_active_frame_bytes <= 12'd0;
            dbg_tx_last_frame_bytes <= 12'd0;
            dbg_tx_sample_count <= 6'd0;
            dbg_tx_first24 <= 192'h0;
        end else begin
            dbg_gmii_tx_clk_edges <= dbg_gmii_tx_clk_edges + 32'd1;
            if (tx_collect_overflow)
                dbg_tx_collect_overflow_seen <= 1'b1;
            if (gmii_tx_en) begin
                dbg_gmii_tx_bytes <= dbg_gmii_tx_bytes + 32'd1;
                if (!dbg_gmii_tx_en_prev) begin
                    dbg_gmii_tx_frames <= dbg_gmii_tx_frames + 16'd1;
                    dbg_tx_seen_frame <= 1'b1;
                    dbg_tx_sample_overflow <= 1'b0;
                    dbg_tx_active_frame_bytes <= 12'd1;
                    dbg_tx_sample_count <= 6'd1;
                    dbg_tx_first24 <= {184'd0, gmii_txd};
                end else begin
                    dbg_tx_active_frame_bytes <= dbg_tx_active_frame_bytes + 12'd1;
                    if (dbg_tx_sample_count < 6'd24) begin
                        dbg_tx_sample_count <= dbg_tx_sample_count + 6'd1;
                        dbg_tx_first24 <= {dbg_tx_first24[183:0], gmii_txd};
                    end else begin
                        dbg_tx_sample_overflow <= 1'b1;
                    end
                end
            end else if (dbg_gmii_tx_en_prev) begin
                dbg_tx_last_frame_bytes <= dbg_tx_active_frame_bytes;
            end
            dbg_gmii_tx_en_prev <= gmii_tx_en;
        end
    end

    // ---------------------------------------------------------------------
    // Receive: verify the Ethernet FCS residue, strip FCS, then replay one
    // complete frame to the HPC1 byte stream.
    // ---------------------------------------------------------------------
    reg [2:0] rx_state;
    reg [1:0] rx_replay_state;
    reg       rx_dv_prev;
    reg [11:0] rx_len;
    reg [11:0] rx_replay_len;
    reg [11:0] rx_replay_idx;
    reg        rx_collect_bank;
    reg        rx_replay_bank;
    reg        rx_pending_valid;
    reg        rx_pending_bank;
    reg [11:0] rx_pending_len;
    reg        rx_drop_current;
    reg [31:0] rx_crc;
    reg [31:0] rx_next_crc;
    reg        rx_frame_error;
    reg [8:0]  rx_fifo_wdata_r;
    reg [31:0] dbg_stream_rx_bytes;
    reg [15:0] dbg_stream_rx_frames;
    reg [15:0] dbg_gmii_rx_good_frames;
    reg [15:0] dbg_gmii_rx_bad_frames;
    reg [15:0] dbg_rx_fifo_full_cycles;
    reg [15:0] dbg_rx_pending_overflow_drops;
    reg [15:0] dbg_rx_collect_busy_drops;
    reg [15:0] dbg_rx_frame_error_drops;

    assign rx_fifo_wdata = rx_fifo_wdata_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dbg_stream_rx_bytes <= 32'd0;
            dbg_stream_rx_frames <= 16'd0;
            rx_stream_in_frame <= 1'b0;
        end else if (stream_rx_valid && stream_rx_ready) begin
            dbg_stream_rx_bytes <= dbg_stream_rx_bytes + 32'd1;
            if (stream_rx_eof) begin
                dbg_stream_rx_frames <= dbg_stream_rx_frames + 16'd1;
                rx_stream_in_frame <= 1'b0;
            end else begin
                rx_stream_in_frame <= 1'b1;
            end
        end
    end

    wire       rx_byte_ready = gmii_rx_dv;
    wire [7:0] rx_byte_in = gmii_rxd;
    wire       rx_start_data_byte =
        rx_byte_ready &&
        (rx_state == RX_IDLE || rx_state == RX_PREAMBLE) &&
        rx_byte_in != 8'h55 && rx_byte_in != 8'hd5;
    wire       rx_frame_data_byte = rx_byte_ready && rx_state == RX_FRAME;
    wire       rx_collect_bank_busy =
        ((rx_replay_state != RXR_IDLE) && (rx_replay_bank == rx_collect_bank)) ||
        (rx_pending_valid && (rx_pending_bank == rx_collect_bank));
    wire       rx_frame_store =
        (rx_start_data_byte && !rx_collect_bank_busy) ||
        (rx_frame_data_byte && !rx_drop_current && (rx_len < MAX_FRAME_LIMIT));
    wire [11:0] rx_frame_waddr = {
        rx_collect_bank,
        rx_start_data_byte ? 11'd0 : rx_len[10:0]
    };
    reg [11:0]  rx_frame_raddr;
    wire [7:0]  rx_frame_rdata;

    cyan_sync_byte_ram #(.ADDR_W(12)) rx_frame_ram (
        .clk  (gmii_rx_clk),
        .we   (rx_frame_store),
        .waddr(rx_frame_waddr),
        .wdata(rx_byte_in),
        .raddr(rx_frame_raddr),
        .rdata(rx_frame_rdata)
    );

    task rx_start_empty;
        begin
            rx_len <= 12'd0;
            rx_crc <= CRC_INIT;
            rx_drop_current <= rx_collect_bank_busy;
            rx_frame_error <= gmii_rx_er || rx_collect_bank_busy;
            rx_state <= RX_FRAME;
        end
    endtask

    task rx_start_with_byte;
        input [7:0] b;
        begin
            rx_drop_current <= rx_collect_bank_busy;
            if (rx_collect_bank_busy) begin
                rx_len <= 12'd0;
                rx_crc <= CRC_INIT;
            end else begin
                rx_len <= 12'd1;
                rx_crc <= crc32_next(CRC_INIT, b);
            end
            rx_frame_error <= gmii_rx_er || rx_collect_bank_busy;
            rx_state <= RX_FRAME;
        end
    endtask

    task rx_store_byte;
        input [7:0] b;
        begin
            if (!rx_drop_current) begin
                if (rx_len < MAX_FRAME_LIMIT) begin
                    rx_len <= rx_len + 12'd1;
                    rx_next_crc = crc32_next(rx_crc, b);
                    rx_crc <= rx_next_crc;
                end else begin
                    rx_frame_error <= 1'b1;
                end
            end
            if (gmii_rx_er)
                rx_frame_error <= 1'b1;
        end
    endtask

    task rx_process_byte;
        input [7:0] b;
        begin
            case (rx_state)
                RX_IDLE: begin
                    if (b == 8'h55)
                        rx_state <= RX_PREAMBLE;
                    else if (b == 8'hd5)
                        rx_start_empty();
                    else
                        rx_start_with_byte(b);
                end

                RX_PREAMBLE: begin
                    if (b == 8'h55)
                        rx_state <= RX_PREAMBLE;
                    else if (b == 8'hd5)
                        rx_start_empty();
                    else
                        rx_start_with_byte(b);
                end

                RX_FRAME: begin
                    rx_store_byte(b);
                end

                default: begin
                    if (gmii_rx_er)
                        rx_frame_error <= 1'b1;
                end
            endcase
        end
    endtask

    always @(posedge gmii_rx_clk or negedge gmii_rst_n) begin
        if (!gmii_rst_n) begin
            rx_state <= RX_IDLE;
            rx_replay_state <= RXR_IDLE;
            rx_dv_prev <= 1'b0;
            rx_len <= 12'd0;
            rx_replay_len <= 12'd0;
            rx_replay_idx <= 12'd0;
            rx_collect_bank <= 1'b0;
            rx_replay_bank <= 1'b0;
            rx_pending_valid <= 1'b0;
            rx_pending_bank <= 1'b0;
            rx_pending_len <= 12'd0;
            rx_drop_current <= 1'b0;
            rx_frame_raddr <= 12'd0;
            rx_crc <= CRC_INIT;
            rx_frame_error <= 1'b0;
            rx_fifo_wrreq <= 1'b0;
            rx_fifo_wdata_r <= 9'h0;
            dbg_gmii_rx_good_frames <= 16'd0;
            dbg_gmii_rx_bad_frames <= 16'd0;
            dbg_rx_fifo_full_cycles <= 16'd0;
            dbg_rx_pending_overflow_drops <= 16'd0;
            dbg_rx_collect_busy_drops <= 16'd0;
            dbg_rx_frame_error_drops <= 16'd0;
        end else begin
            rx_fifo_wrreq <= 1'b0;
            rx_dv_prev <= gmii_rx_dv;

            if (rx_dv_prev && !gmii_rx_dv) begin
                if (rx_state == RX_FRAME && !rx_frame_error &&
                    rx_len > 12'd4 && rx_crc == CRC_RESIDUE) begin
                    dbg_gmii_rx_good_frames <= dbg_gmii_rx_good_frames + 16'd1;
                    if (rx_replay_state == RXR_IDLE) begin
                        rx_replay_bank <= rx_collect_bank;
                        rx_replay_len <= rx_len - 12'd4;
                        rx_replay_idx <= 12'd0;
                        rx_frame_raddr <= {rx_collect_bank, 11'd0};
                        rx_replay_state <= RXR_LOAD;
                    end else if (!rx_pending_valid) begin
                        rx_pending_valid <= 1'b1;
                        rx_pending_bank <= rx_collect_bank;
                        rx_pending_len <= rx_len - 12'd4;
                    end else begin
                        dbg_gmii_rx_bad_frames <= dbg_gmii_rx_bad_frames + 16'd1;
                        if (dbg_rx_pending_overflow_drops != 16'hffff)
                            dbg_rx_pending_overflow_drops <=
                                dbg_rx_pending_overflow_drops + 16'd1;
                    end
                    rx_collect_bank <= !rx_collect_bank;
                    rx_drop_current <= 1'b0;
                    rx_state <= RX_IDLE;
                end else if (rx_state == RX_FRAME || rx_state == RX_PREAMBLE) begin
                    dbg_gmii_rx_bad_frames <= dbg_gmii_rx_bad_frames + 16'd1;
                    if (rx_drop_current) begin
                        if (dbg_rx_collect_busy_drops != 16'hffff)
                            dbg_rx_collect_busy_drops <=
                                dbg_rx_collect_busy_drops + 16'd1;
                    end else if (dbg_rx_frame_error_drops != 16'hffff) begin
                        dbg_rx_frame_error_drops <=
                            dbg_rx_frame_error_drops + 16'd1;
                    end
                    rx_drop_current <= 1'b0;
                    rx_state <= RX_IDLE;
                end
            end else if (gmii_rx_dv) begin
                rx_process_byte(gmii_rxd);
            end

            case (rx_replay_state)
                RXR_IDLE: begin
                    if (rx_pending_valid) begin
                        rx_replay_bank <= rx_pending_bank;
                        rx_replay_len <= rx_pending_len;
                        rx_replay_idx <= 12'd0;
                        rx_frame_raddr <= {rx_pending_bank, 11'd0};
                        rx_pending_valid <= 1'b0;
                        rx_replay_state <= RXR_LOAD;
                    end
                end

                RXR_LOAD: begin
                    if (rx_replay_len != 12'd0) begin
                        rx_frame_raddr <= {rx_replay_bank, 11'd1};
                        rx_replay_state <= RXR_SEND;
                    end else begin
                        rx_replay_state <= RXR_IDLE;
                    end
                end

                RXR_SEND: begin
                    if (!rx_fifo_full) begin
                        rx_fifo_wdata_r <= {
                            (rx_replay_idx + 12'd1 >= rx_replay_len),
                            rx_frame_rdata
                        };
                        rx_fifo_wrreq <= 1'b1;
                        if (rx_replay_idx + 12'd1 >= rx_replay_len) begin
                            if (rx_pending_valid) begin
                                rx_replay_bank <= rx_pending_bank;
                                rx_replay_len <= rx_pending_len;
                                rx_replay_idx <= 12'd0;
                                rx_frame_raddr <= {rx_pending_bank, 11'd0};
                                rx_pending_valid <= 1'b0;
                                rx_replay_state <= RXR_LOAD;
                            end else begin
                                rx_replay_state <= RXR_IDLE;
                            end
                        end else begin
                            rx_replay_idx <= rx_replay_idx + 12'd1;
                            rx_frame_raddr <= {
                                rx_replay_bank,
                                rx_replay_idx[10:0] + 11'd2
                            };
                        end
                    end else if (dbg_rx_fifo_full_cycles != 16'hffff) begin
                        dbg_rx_fifo_full_cycles <=
                            dbg_rx_fifo_full_cycles + 16'd1;
                    end
                end

                default: ;
            endcase
        end
    end

    reg [31:0] dbg_gmii_rx_clk_edges;
    reg [31:0] dbg_gmii_rx_bytes;
    reg [15:0] dbg_gmii_rx_frames;
    reg        dbg_gmii_rx_dv_prev;

    always @(posedge gmii_rx_clk or negedge gmii_rst_n) begin
        if (!gmii_rst_n) begin
            dbg_gmii_rx_clk_edges <= 32'd0;
            dbg_gmii_rx_bytes <= 32'd0;
            dbg_gmii_rx_frames <= 16'd0;
            dbg_gmii_rx_dv_prev <= 1'b0;
        end else begin
            dbg_gmii_rx_clk_edges <= dbg_gmii_rx_clk_edges + 32'd1;
            if (gmii_rx_dv) begin
                dbg_gmii_rx_bytes <= dbg_gmii_rx_bytes + 32'd1;
                if (!dbg_gmii_rx_dv_prev)
                    dbg_gmii_rx_frames <= dbg_gmii_rx_frames + 16'd1;
            end
            dbg_gmii_rx_dv_prev <= gmii_rx_dv;
        end
    end

    // ---------------------------------------------------------------------
    // PHY reset and MDIO. Scan Clause-22 addresses, configure the first
    // responding Marvell PHY, then keep the status readback live for probes.
    // ---------------------------------------------------------------------
    localparam [4:0]
        MDIO_RD_MARVELL_MODE       = 5'd0,
        MDIO_WR_MARVELL_MODE       = 5'd1,
        MDIO_RD_MARVELL_TIMING     = 5'd2,
        MDIO_WR_MARVELL_TIMING     = 5'd3,
        MDIO_WR_MARVELL_CTRL       = 5'd4,
        MDIO_WR_BMCR_RESET         = 5'd5,
        MDIO_WAIT_SW_RESET         = 5'd6,
        MDIO_WR_GB_ADV             = 5'd7,
        MDIO_WR_ADV                = 5'd8,
        MDIO_WR_BMCR               = 5'd9,
        MDIO_RD_FINAL_BMCR         = 5'd10,
        MDIO_RD_FINAL_BMSR_LATCH   = 5'd11,
        MDIO_RD_FINAL_BMSR         = 5'd12,
        MDIO_RD_FINAL_PHY_STATUS   = 5'd13,
        MDIO_RD_FINAL_MARVELL_MODE = 5'd14,
        MDIO_RD_FINAL_TIMING       = 5'd15,
        MDIO_REFRESH_WAIT          = 5'd16;

    reg        phy_rst_n_r;
    reg [31:0] phy_reset_count;
    reg [31:0] phy_sw_reset_count;
    reg [31:0] phy_refresh_count;
    reg [15:0] mdc_div_count;
    reg        mdc_r;
    reg        mdio_busy;
    reg        mdio_done;
    reg        mdio_oe;
    reg        mdio_out;
    reg        mdio_read_op;
    reg [63:0] mdio_shift;
    reg [6:0]  mdio_bit_index;
    reg [4:0]  mdio_phy_addr;
    reg [4:0]  mdio_step;
    reg [15:0] mdio_read_data;
    reg [15:0] mdio_mode_word;
    reg [15:0] mdio_timing_word;
    reg        mdio_phy_seen;
    reg        mdio_readback_done;
    reg [4:0]  mdio_seen_phy_addr;
    reg [15:0] mdio_seen_mode_word;
    reg [15:0] mdio_seen_timing_word;
    reg [15:0] mdio_final_bmcr_word;
    reg [15:0] mdio_final_bmsr_word;
    reg [15:0] mdio_final_phy_status_word;
    reg [15:0] mdio_final_mode_word;
    reg [15:0] mdio_final_timing_word;

    assign phy_rst_n = phy_rst_n_r;
    assign phy_mdc = mdc_r;
    assign phy_mdio = mdio_oe ? mdio_out : 1'bz;

    function [63:0] mdio_write_frame;
        input [4:0] phy_addr;
        input [4:0] reg_addr;
        input [15:0] data;
        begin
            mdio_write_frame = {
                32'hffff_ffff,
                2'b01,
                2'b01,
                phy_addr,
                reg_addr,
                2'b10,
                data
            };
        end
    endfunction

    function [63:0] mdio_read_frame;
        input [4:0] phy_addr;
        input [4:0] reg_addr;
        begin
            mdio_read_frame = {
                32'hffff_ffff,
                2'b01,
                2'b10,
                phy_addr,
                reg_addr,
                18'h0
            };
        end
    endfunction

    function [4:0] mdio_reg_for_step;
        input [4:0] step;
        begin
            case (step)
                MDIO_RD_MARVELL_MODE,
                MDIO_WR_MARVELL_MODE,
                MDIO_RD_FINAL_MARVELL_MODE:
                    mdio_reg_for_step = 5'd27;
                MDIO_RD_MARVELL_TIMING,
                MDIO_WR_MARVELL_TIMING,
                MDIO_RD_FINAL_TIMING:
                    mdio_reg_for_step = 5'd20;
                MDIO_WR_MARVELL_CTRL:   mdio_reg_for_step = 5'd16;
                MDIO_RD_FINAL_PHY_STATUS:
                    mdio_reg_for_step = 5'd17;
                MDIO_WR_GB_ADV:         mdio_reg_for_step = 5'd9;
                MDIO_WR_ADV:            mdio_reg_for_step = 5'd4;
                MDIO_RD_FINAL_BMSR_LATCH,
                MDIO_RD_FINAL_BMSR:
                    mdio_reg_for_step = 5'd1;
                default:                mdio_reg_for_step = 5'd0;
            endcase
        end
    endfunction

    function [15:0] mdio_data_for_step;
        input [4:0] step;
        begin
            case (step)
                // KCU105 has the 88E1111 MAC-side interface wired as SGMII.
                // Keep strap/status bits, set HWCFG_MODE[3:0] to SGMII (0x4),
                // then soft-reset the PHY so the mode latches.
                MDIO_WR_MARVELL_MODE:
                    mdio_data_for_step = (mdio_mode_word & 16'hfff0) | 16'h0004;
                MDIO_WR_MARVELL_TIMING:
                    mdio_data_for_step = mdio_timing_word & ~16'h0082;
                MDIO_WR_MARVELL_CTRL:
                    mdio_data_for_step = 16'h0078;
                MDIO_WR_BMCR_RESET:
                    mdio_data_for_step = 16'h8000;
                MDIO_WR_GB_ADV:
                    mdio_data_for_step = 16'h0300;
                MDIO_WR_ADV:
                    mdio_data_for_step = 16'h01e1;
                default:
                    mdio_data_for_step = 16'h1200;
            endcase
        end
    endfunction

    function mdio_step_is_read;
        input [4:0] step;
        begin
            case (step)
                MDIO_RD_MARVELL_MODE,
                MDIO_RD_MARVELL_TIMING,
                MDIO_RD_FINAL_BMCR,
                MDIO_RD_FINAL_BMSR_LATCH,
                MDIO_RD_FINAL_BMSR,
                MDIO_RD_FINAL_PHY_STATUS,
                MDIO_RD_FINAL_MARVELL_MODE,
                MDIO_RD_FINAL_TIMING:
                    mdio_step_is_read = 1'b1;
                default:
                    mdio_step_is_read = 1'b0;
            endcase
        end
    endfunction

    task mdio_start_write;
        input [4:0] reg_addr;
        input [15:0] data;
        begin
            mdio_shift <= mdio_write_frame(mdio_phy_addr, reg_addr, data);
            mdio_read_op <= 1'b0;
            mdio_read_data <= 16'h0;
            mdio_bit_index <= 7'd0;
            mdio_out <= 1'b1;
            mdio_oe <= 1'b1;
            mdio_busy <= 1'b1;
            mdc_r <= 1'b0;
            mdc_div_count <= 16'd0;
        end
    endtask

    task mdio_start_read;
        input [4:0] reg_addr;
        begin
            mdio_shift <= mdio_read_frame(mdio_phy_addr, reg_addr);
            mdio_read_op <= 1'b1;
            mdio_read_data <= 16'h0;
            mdio_bit_index <= 7'd0;
            mdio_out <= 1'b1;
            mdio_oe <= 1'b1;
            mdio_busy <= 1'b1;
            mdc_r <= 1'b0;
            mdc_div_count <= 16'd0;
        end
    endtask

    task mdio_advance_step;
        begin
            case (mdio_step)
                MDIO_RD_MARVELL_MODE: begin
                    mdio_mode_word <= mdio_read_data;
                    if (mdio_read_data != 16'hffff) begin
                        mdio_phy_seen <= 1'b1;
                        mdio_seen_phy_addr <= mdio_phy_addr;
                        mdio_seen_mode_word <= mdio_read_data;
                        mdio_step <= MDIO_WR_MARVELL_MODE;
                    end else if (mdio_phy_addr == 5'd31) begin
                        mdio_done <= 1'b1;
                    end else begin
                        mdio_phy_addr <= mdio_phy_addr + 5'd1;
                    end
                end
                MDIO_WR_MARVELL_MODE: begin
                    mdio_step <= MDIO_RD_MARVELL_TIMING;
                end
                MDIO_RD_MARVELL_TIMING: begin
                    mdio_timing_word <= mdio_read_data;
                    if (mdio_read_data != 16'hffff &&
                        mdio_phy_addr == mdio_seen_phy_addr) begin
                        mdio_seen_timing_word <= mdio_read_data;
                    end
                    mdio_step <= MDIO_WR_MARVELL_TIMING;
                end
                MDIO_WR_MARVELL_TIMING: begin
                    mdio_step <= MDIO_WR_MARVELL_CTRL;
                end
                MDIO_WR_MARVELL_CTRL: begin
                    mdio_step <= MDIO_WR_BMCR_RESET;
                end
                MDIO_WR_BMCR_RESET: begin
                    phy_sw_reset_count <= 32'd0;
                    mdio_step <= MDIO_WAIT_SW_RESET;
                end
                MDIO_WR_GB_ADV: begin
                    mdio_step <= MDIO_WR_ADV;
                end
                MDIO_WR_ADV: begin
                    mdio_step <= MDIO_WR_BMCR;
                end
                MDIO_WR_BMCR: begin
                    mdio_phy_addr <= mdio_seen_phy_addr;
                    mdio_step <= MDIO_RD_FINAL_BMCR;
                end
                MDIO_RD_FINAL_BMCR: begin
                    mdio_final_bmcr_word <= mdio_read_data;
                    mdio_step <= MDIO_RD_FINAL_BMSR_LATCH;
                end
                MDIO_RD_FINAL_BMSR_LATCH: begin
                    mdio_step <= MDIO_RD_FINAL_BMSR;
                end
                MDIO_RD_FINAL_BMSR: begin
                    mdio_final_bmsr_word <= mdio_read_data;
                    mdio_step <= MDIO_RD_FINAL_PHY_STATUS;
                end
                MDIO_RD_FINAL_PHY_STATUS: begin
                    mdio_final_phy_status_word <= mdio_read_data;
                    mdio_step <= MDIO_RD_FINAL_MARVELL_MODE;
                end
                MDIO_RD_FINAL_MARVELL_MODE: begin
                    mdio_final_mode_word <= mdio_read_data;
                    mdio_step <= MDIO_RD_FINAL_TIMING;
                end
                MDIO_RD_FINAL_TIMING: begin
                    mdio_final_timing_word <= mdio_read_data;
                    mdio_readback_done <= 1'b1;
                    mdio_done <= 1'b1;
                    phy_refresh_count <= 32'd0;
                    mdio_step <= MDIO_REFRESH_WAIT;
                end
                MDIO_REFRESH_WAIT: begin
                    mdio_phy_addr <= mdio_seen_phy_addr;
                    mdio_step <= MDIO_RD_FINAL_BMCR;
                end
                default: begin
                    mdio_step <= MDIO_RD_MARVELL_MODE;
                end
            endcase
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phy_rst_n_r <= 1'b0;
            phy_reset_count <= 32'd0;
            phy_sw_reset_count <= 32'd0;
            phy_refresh_count <= 32'd0;
            mdc_div_count <= 16'd0;
            mdc_r <= 1'b0;
            mdio_busy <= 1'b0;
            mdio_done <= 1'b0;
            mdio_oe <= 1'b0;
            mdio_out <= 1'b1;
            mdio_read_op <= 1'b0;
            mdio_shift <= 64'h0;
            mdio_bit_index <= 7'd0;
            mdio_phy_addr <= 5'd0;
            mdio_step <= MDIO_RD_MARVELL_MODE;
            mdio_read_data <= 16'h0;
            mdio_mode_word <= 16'h0;
            mdio_timing_word <= 16'h0;
            mdio_phy_seen <= 1'b0;
            mdio_readback_done <= 1'b0;
            mdio_seen_phy_addr <= 5'd0;
            mdio_seen_mode_word <= 16'h0;
            mdio_seen_timing_word <= 16'h0;
            mdio_final_bmcr_word <= 16'h0;
            mdio_final_bmsr_word <= 16'h0;
            mdio_final_phy_status_word <= 16'h0;
            mdio_final_mode_word <= 16'h0;
            mdio_final_timing_word <= 16'h0;
        end else begin
            if (phy_reset_count < PHY_RESET_LIMIT + PHY_POST_RESET_LIMIT)
                phy_reset_count <= phy_reset_count + 32'd1;
            phy_rst_n_r <= (phy_reset_count >= PHY_RESET_LIMIT);

            if (!mdio_busy &&
                phy_reset_count >= PHY_RESET_LIMIT + PHY_POST_RESET_LIMIT &&
                (!mdio_done || mdio_readback_done ||
                 mdio_step == MDIO_REFRESH_WAIT)) begin
                if (mdio_step == MDIO_WAIT_SW_RESET) begin
                    if (phy_sw_reset_count >= PHY_SW_RESET_LIMIT) begin
                        mdio_step <= MDIO_WR_GB_ADV;
                    end else begin
                        phy_sw_reset_count <= phy_sw_reset_count + 32'd1;
                    end
                end else if (mdio_step == MDIO_REFRESH_WAIT) begin
                    if (phy_refresh_count >= PHY_REFRESH_LIMIT) begin
                        phy_refresh_count <= 32'd0;
                        mdio_advance_step();
                    end else begin
                        phy_refresh_count <= phy_refresh_count + 32'd1;
                    end
                end else if (mdio_step_is_read(mdio_step)) begin
                    mdio_start_read(mdio_reg_for_step(mdio_step));
                end else begin
                    mdio_start_write(mdio_reg_for_step(mdio_step),
                                     mdio_data_for_step(mdio_step));
                end
            end else if (mdio_busy) begin
                if (mdc_div_count + 16'd1 >= MDC_DIV_LIMIT) begin
                    mdc_div_count <= 16'd0;
                    if (!mdc_r) begin
                        mdc_r <= 1'b1;
                        if (mdio_read_op && mdio_bit_index >= 7'd48)
                            mdio_read_data <= {mdio_read_data[14:0], phy_mdio};
                    end else begin
                        mdc_r <= 1'b0;
                        if (mdio_bit_index == 7'd63) begin
                            mdio_busy <= 1'b0;
                            mdio_oe <= 1'b0;
                            mdio_advance_step();
                        end else begin
                            mdio_shift <= {mdio_shift[62:0], 1'b0};
                            mdio_bit_index <= mdio_bit_index + 7'd1;
                            mdio_out <= mdio_shift[62];
                            mdio_oe <= !mdio_read_op ||
                                       (mdio_bit_index + 7'd1 < 7'd46);
                        end
                    end
                end else begin
                    mdc_div_count <= mdc_div_count + 16'd1;
                end
            end
        end
    end

    wire [31:0] debug_status = {
        phy_rst_n_r,
        mdio_done,
        mdio_busy,
        mdio_readback_done,
        stream_tx_ready,
        tx_fifo_empty,
        tx_fifo_full,
        gmii_tx_en,
        gmii_rx_dv,
        tx_state,
        rx_state,
        mdio_step,
        mdio_phy_addr,
        mdio_bit_index[5:0]
    };
    wire [31:0] debug_mdio_detect = {
        mdio_phy_seen,
        mdio_readback_done,
        9'd0,
        mdio_seen_phy_addr,
        mdio_read_data
    };

    wire [223:0] debug_normal_probe = {
        debug_status,
        debug_mdio_detect,
        {mdio_final_bmcr_word, mdio_final_bmsr_word},
        {mdio_final_phy_status_word, mdio_final_mode_word},
        {mdio_final_timing_word, mdio_seen_mode_word},
        {dbg_gmii_rx_frames, dbg_gmii_rx_good_frames},
        {dbg_stream_tx_frames, dbg_gmii_tx_frames}
    };
    wire [3:0] debug_tx_flags = {
        dbg_tx_seen_frame,
        dbg_tx_sample_overflow,
        dbg_tx_collect_overflow_seen,
        tx_fifo_full
    };
    wire [31:0] debug_tx_status = {
        4'hd,
        dbg_stream_tx_frames[3:0],
        dbg_gmii_tx_frames[3:0],
        dbg_tx_last_frame_bytes,
        tx_state,
        debug_tx_flags
    };
    wire [223:0] debug_tx_probe = {
        debug_tx_status,
        dbg_tx_first24
    };

    assign debug_probe = (DEBUG_TX_PROBES != 0) ?
        debug_tx_probe : debug_normal_probe;
endmodule

module gmii_async_fifo #(
    parameter integer WIDTH = 10,
    parameter integer ADDR_W = 11
) (
    input  wire             wr_clk,
    input  wire             wr_rst_n,
    input  wire [WIDTH-1:0] wr_data,
    input  wire             wr_en,
    output wire             wr_full,
    output wire [ADDR_W:0]  wr_count,

    input  wire             rd_clk,
    input  wire             rd_rst_n,
    output wire [WIDTH-1:0] rd_data,
    input  wire             rd_en,
    output wire             rd_empty,
    output wire             rd_valid
);
`ifdef SYNTHESIS
    wire [WIDTH-1:0] xpm_rd_data;
    wire             xpm_wr_full;
    wire             xpm_rd_empty;
    wire             xpm_rd_valid;
    wire             xpm_wr_rst_busy;
    wire             xpm_rd_rst_busy;
    wire [ADDR_W:0]  xpm_wr_data_count;
    reg              xpm_rd_en;
    reg [WIDTH-1:0]  rd_hold_data;
    reg              rd_hold_valid;
    reg              rd_pending;

    wire fifo_rst_n = wr_rst_n && rd_rst_n;
    wire rd_consume = rd_en && rd_hold_valid;

    // XPM requires both enables to remain inactive while reset-busy is high.
    // The producer and consumer resets are independent in several CYAN uses;
    // treating busy as full/empty prevents either live side from issuing into
    // an XPM whose other clock domain is still completing reset.
    assign wr_full = !fifo_rst_n || xpm_wr_rst_busy || xpm_wr_full;
    assign wr_count = (!fifo_rst_n || xpm_wr_rst_busy) ?
                      {ADDR_W+1{1'b0}} : xpm_wr_data_count;
    assign rd_data = rd_hold_data;
    assign rd_empty = !fifo_rst_n || xpm_rd_rst_busy || !rd_hold_valid;
    assign rd_valid = fifo_rst_n && !xpm_rd_rst_busy && rd_hold_valid;

    xpm_fifo_async #(
        .FIFO_MEMORY_TYPE("auto"),
        .ECC_MODE("no_ecc"),
        .RELATED_CLOCKS(0),
        .SIM_ASSERT_CHK(0),
        .CASCADE_HEIGHT(0),
        .FIFO_WRITE_DEPTH(1 << ADDR_W),
        .WRITE_DATA_WIDTH(WIDTH),
        .WR_DATA_COUNT_WIDTH(ADDR_W + 1),
        .PROG_FULL_THRESH(10),
        .FULL_RESET_VALUE(0),
        // Keep standard-mode read data-valid and expose the write-domain
        // occupancy count used by latency-aware clients.
        .USE_ADV_FEATURES("1004"),
        .READ_MODE("std"),
        .FIFO_READ_LATENCY(1),
        .READ_DATA_WIDTH(WIDTH),
        .RD_DATA_COUNT_WIDTH(ADDR_W + 1),
        .PROG_EMPTY_THRESH(10),
        .DOUT_RESET_VALUE("0"),
        .CDC_SYNC_STAGES(3),
        .WAKEUP_TIME(0)
    ) fifo_inst (
        .sleep(1'b0),
        .rst(!fifo_rst_n),
        .wr_clk(wr_clk),
        .wr_en(wr_en && fifo_rst_n && !xpm_wr_rst_busy && !xpm_wr_full),
        .din(wr_data),
        .full(xpm_wr_full),
        .prog_full(),
        .wr_data_count(xpm_wr_data_count),
        .overflow(),
        .wr_rst_busy(xpm_wr_rst_busy),
        .almost_full(),
        .wr_ack(),
        .rd_clk(rd_clk),
        .rd_en(xpm_rd_en),
        .dout(xpm_rd_data),
        .empty(xpm_rd_empty),
        .prog_empty(),
        .rd_data_count(),
        .underflow(),
        .rd_rst_busy(xpm_rd_rst_busy),
        .almost_empty(),
        .data_valid(xpm_rd_valid),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr(),
        .dbiterr()
    );

    always @(posedge rd_clk or negedge fifo_rst_n) begin
        if (!fifo_rst_n) begin
            xpm_rd_en <= 1'b0;
            rd_hold_data <= {WIDTH{1'b0}};
            rd_hold_valid <= 1'b0;
            rd_pending <= 1'b0;
        end else if (xpm_rd_rst_busy) begin
            xpm_rd_en <= 1'b0;
            rd_hold_data <= {WIDTH{1'b0}};
            rd_hold_valid <= 1'b0;
            rd_pending <= 1'b0;
        end else begin
            xpm_rd_en <= 1'b0;

            if (rd_consume)
                rd_hold_valid <= 1'b0;

            if (xpm_rd_valid) begin
                rd_hold_data <= xpm_rd_data;
                rd_hold_valid <= 1'b1;
                rd_pending <= 1'b0;
            end

            if (!rd_pending && !xpm_rd_empty &&
                (!rd_hold_valid || rd_consume)) begin
                xpm_rd_en <= 1'b1;
                rd_pending <= 1'b1;
            end
        end
    end
`else
    localparam integer DEPTH = (1 << ADDR_W);

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_W:0]  wr_bin;
    reg [ADDR_W:0]  wr_gray;
    reg [ADDR_W:0]  rd_bin;
    reg [ADDR_W:0]  rd_gray;
    reg [ADDR_W:0]  rd_gray_w1;
    reg [ADDR_W:0]  rd_gray_w2;
    reg [ADDR_W:0]  wr_gray_r1;
    reg [ADDR_W:0]  wr_gray_r2;
    reg [WIDTH-1:0] rd_hold_data;
    reg             rd_hold_valid;
    reg [2:0]       wr_ready_pipe;
    reg [2:0]       rd_ready_pipe;

    wire            fifo_rst_n = wr_rst_n && rd_rst_n;
    wire            wr_ready = wr_ready_pipe[2];
    wire            rd_ready = rd_ready_pipe[2];
    wire            wr_fire = wr_en && !wr_full;
    wire            rd_consume = rd_en && rd_hold_valid;
    wire            raw_empty = (rd_gray == wr_gray_r2);
    wire            rd_load = rd_ready &&
                               (!rd_hold_valid || rd_consume) && !raw_empty;
    wire [ADDR_W:0] wr_bin_plus_one = wr_bin + {{ADDR_W{1'b0}}, 1'b1};
    wire [ADDR_W:0] wr_gray_plus_one = (wr_bin_plus_one >> 1) ^ wr_bin_plus_one;
    wire [ADDR_W:0] wr_bin_next = wr_bin + {{ADDR_W{1'b0}}, wr_fire};
    wire [ADDR_W:0] wr_gray_next = (wr_bin_next >> 1) ^ wr_bin_next;
    wire [ADDR_W:0] rd_bin_next = rd_bin + {{ADDR_W{1'b0}}, rd_load};
    wire [ADDR_W:0] rd_gray_next = (rd_bin_next >> 1) ^ rd_bin_next;

    function [ADDR_W:0] gray_to_binary;
        input [ADDR_W:0] gray;
        integer bit_i;
        begin
            gray_to_binary[ADDR_W] = gray[ADDR_W];
            for (bit_i = ADDR_W - 1; bit_i >= 0; bit_i = bit_i - 1)
                gray_to_binary[bit_i] =
                    gray_to_binary[bit_i + 1] ^ gray[bit_i];
        end
    endfunction

    assign wr_full = !wr_ready ||
                     (wr_gray_plus_one == {~rd_gray_w2[ADDR_W:ADDR_W-1],
                                            rd_gray_w2[ADDR_W-2:0]});
    assign wr_count = wr_ready ?
        (wr_bin - gray_to_binary(rd_gray_w2)) : {ADDR_W+1{1'b0}};
    assign rd_empty = !rd_ready || !rd_hold_valid;
    assign rd_valid = rd_ready && rd_hold_valid;
    assign rd_data = rd_hold_data;

    always @(posedge wr_clk or negedge fifo_rst_n) begin
        if (!fifo_rst_n) begin
            wr_bin <= {ADDR_W+1{1'b0}};
            wr_gray <= {ADDR_W+1{1'b0}};
            rd_gray_w1 <= {ADDR_W+1{1'b0}};
            rd_gray_w2 <= {ADDR_W+1{1'b0}};
            wr_ready_pipe <= 3'b000;
        end else begin
            wr_ready_pipe <= {wr_ready_pipe[1:0], 1'b1};
            rd_gray_w1 <= rd_gray;
            rd_gray_w2 <= rd_gray_w1;
            if (wr_fire) begin
                mem[wr_bin[ADDR_W-1:0]] <= wr_data;
                wr_bin <= wr_bin_next;
                wr_gray <= wr_gray_next;
            end
        end
    end

    always @(posedge rd_clk or negedge fifo_rst_n) begin
        if (!fifo_rst_n) begin
            rd_bin <= {ADDR_W+1{1'b0}};
            rd_gray <= {ADDR_W+1{1'b0}};
            wr_gray_r1 <= {ADDR_W+1{1'b0}};
            wr_gray_r2 <= {ADDR_W+1{1'b0}};
            rd_hold_data <= {WIDTH{1'b0}};
            rd_hold_valid <= 1'b0;
            rd_ready_pipe <= 3'b000;
        end else begin
            rd_ready_pipe <= {rd_ready_pipe[1:0], 1'b1};
            wr_gray_r1 <= wr_gray;
            wr_gray_r2 <= wr_gray_r1;
            if (rd_consume)
                rd_hold_valid <= 1'b0;
            if (rd_load) begin
                rd_hold_data <= mem[rd_bin[ADDR_W-1:0]];
                rd_hold_valid <= 1'b1;
                rd_bin <= rd_bin_next;
                rd_gray <= rd_gray_next;
            end
        end
    end
`endif
endmodule

`default_nettype wire
