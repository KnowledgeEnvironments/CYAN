// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - SD card SPI-mode single-sector block backend.
//
// Init sequence: idle clocks, CMD0, CMD8, ACMD41, CMD58. Data path supports
// CMD17 single-block reads and CMD24 single-block writes. The caller supplies a
// 512-byte sector buffer through the byte-wide buffer port.

`default_nettype none

module sd_spi_byte (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [15:0] clk_div,
    input  wire [7:0]  tx_byte,
    output reg  [7:0]  rx_byte,
    output reg         busy,
    output reg         done,
    output reg         sclk,
    output reg         mosi,
    input  wire        miso
);
    reg [15:0] div_cnt;
    reg [2:0]  bit_idx;

    wire [15:0] div_limit = (clk_div == 16'd0) ? 16'd1 : clk_div;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_byte <= 8'hff;
            busy <= 1'b0;
            done <= 1'b0;
            sclk <= 1'b0;
            mosi <= 1'b1;
            div_cnt <= 16'd0;
            bit_idx <= 3'd7;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                busy <= 1'b1;
                sclk <= 1'b0;
                mosi <= tx_byte[7];
                rx_byte <= 8'h00;
                bit_idx <= 3'd7;
                div_cnt <= 16'd0;
            end else if (busy) begin
                if (div_cnt != div_limit - 16'd1) begin
                    div_cnt <= div_cnt + 16'd1;
                end else begin
                    div_cnt <= 16'd0;
                    if (!sclk) begin
                        sclk <= 1'b1;
                        rx_byte[bit_idx] <= miso;
                    end else begin
                        sclk <= 1'b0;
                        if (bit_idx == 3'd0) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            mosi <= 1'b1;
                        end else begin
                            bit_idx <= bit_idx - 3'd1;
                            mosi <= tx_byte[bit_idx - 3'd1];
                        end
                    end
                end
            end
        end
    end
endmodule

module sd_spi_block #(
    parameter [15:0] INIT_CLK_DIV = 16'd250,
    parameter [15:0] RUN_CLK_DIV = 16'd4,
    parameter [31:0] CMD_TIMEOUT = 32'd100000,
    parameter [31:0] DATA_TIMEOUT = 32'd1000000,
    parameter [31:0] INIT_RETRY_DELAY = 32'd50000000,
    parameter [15:0] ACMD41_RETRIES = 16'd4096
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        req_valid,
    input  wire        req_write,
    input  wire [31:0] req_lba,
    output wire        req_ready,
    output reg         done,
    output reg         error,

    output reg         media_ready,
    output reg  [31:0] media_sectors,

    output reg  [8:0]  buf_addr,
    input  wire [7:0]  buf_rdata,
    output reg         buf_we,
    output reg  [7:0]  buf_wdata,

    output wire        sd_cs_n,
    output wire        sd_clk,
    output wire        sd_mosi,
    input  wire        sd_miso,
    input  wire        sd_card_detect,
    input  wire        read_skip_first_byte,

    output wire [15:0] debug_read_crc_calc,
    output wire [15:0] debug_read_crc_recv,
    output wire        debug_read_crc_mismatch,
    output wire [15:0]  debug_direct_efs_crc,
    output wire [191:0] debug_direct_efs_words,
    output wire [127:0] debug_probe
);
    localparam [7:0]
        S_POWER_IDLE_START = 8'd0,
        S_POWER_IDLE_WAIT  = 8'd1,
        S_CMD_START        = 8'd2,
        S_CMD_WAIT_BYTE    = 8'd3,
        S_CMD_R1_START     = 8'd4,
        S_CMD_R1_WAIT      = 8'd5,
        S_IDLE_BYTE_START  = 8'd6,
        S_IDLE_BYTE_WAIT   = 8'd7,
        S_CMD0_DONE        = 8'd8,
        S_CMD8_DONE        = 8'd9,
        S_CMD8_R7_START    = 8'd10,
        S_CMD8_R7_WAIT     = 8'd11,
        S_CMD55_DONE       = 8'd12,
        S_ACMD41_DONE      = 8'd13,
        S_CMD58_DONE       = 8'd14,
        S_CMD58_OCR_START  = 8'd15,
        S_CMD58_OCR_WAIT   = 8'd16,
        S_READY            = 8'd17,
        S_READ_CMD_DONE    = 8'd18,
        S_READ_TOKEN_START = 8'd19,
        S_READ_TOKEN_WAIT  = 8'd20,
        S_READ_DATA_START  = 8'd21,
        S_READ_DATA_WAIT   = 8'd22,
        S_READ_CRC_START   = 8'd23,
        S_READ_CRC_WAIT    = 8'd24,
        S_WRITE_CMD_DONE   = 8'd25,
        S_WRITE_GAP_START  = 8'd26,
        S_WRITE_GAP_WAIT   = 8'd27,
        S_WRITE_TOKEN_START= 8'd28,
        S_WRITE_TOKEN_WAIT = 8'd29,
        S_WRITE_DATA_START = 8'd30,
        S_WRITE_DATA_WAIT  = 8'd31,
        S_WRITE_CRC_START  = 8'd32,
        S_WRITE_CRC_WAIT   = 8'd33,
        S_WRITE_RESP_START = 8'd34,
        S_WRITE_RESP_WAIT  = 8'd35,
        S_WRITE_BUSY_START = 8'd36,
        S_WRITE_BUSY_WAIT  = 8'd37,
        S_BLOCK_DONE       = 8'd38,
        S_BLOCK_ERROR      = 8'd39,
        S_INIT_ERROR       = 8'd40,
        S_ACMD41_START     = 8'd41,
        S_READ_ALIGN_START = 8'd42,
        S_READ_ALIGN_WAIT  = 8'd43,
        S_WRITE_DATA_LATCH = 8'd44;

    reg [7:0]  state;
    reg [7:0]  after_r1_state;
    reg [7:0]  after_idle_state;
    reg [7:0]  cur_cmd;
    reg [31:0] cur_arg;
    reg [7:0]  cur_crc;
    reg [2:0]  cmd_byte_idx;
    reg [7:0]  spi_tx;
    reg        spi_start;
    reg        cs_n_r;
    reg [15:0] clk_div_r;
    reg [31:0] timeout_cnt;
    reg [15:0] idle_count;
    reg [15:0] retry_count;
    reg [8:0]  byte_count;
    reg [1:0]  extra_count;
    reg [31:0] ocr_shift;
    reg        card_v2;
    reg        card_sdhc;
    reg        active_write;
    reg [31:0] active_lba;
    reg [7:0]  last_cmd;
    reg [7:0]  last_r1;
    reg [7:0]  last_spi_rx;
    reg [7:0]  sticky_error;
    reg        miso_low_seen;
    reg        cs_low_seen;
    reg        sclk_high_seen;
    reg        mosi_low_seen;
    reg [15:0] spi_byte_count;
    reg [15:0] sclk_edge_count;
    reg        spi_sclk_d;
    reg [31:0] direct_efs_word0;
    reg [31:0] direct_efs_word4;
    reg [31:0] direct_efs_word8;
    reg [31:0] direct_efs_word16;
    reg [31:0] direct_efs_word20;
    reg [31:0] direct_efs_word28;
    reg [7:0]  direct_efs_crc0;
    reg [7:0]  direct_efs_crc1;
    reg [15:0] read_crc_calc;
    reg [15:0] read_crc_recv;
    reg        read_crc_mismatch;

    wire spi_busy;
    wire spi_done;
    wire [7:0] spi_rx;
    wire spi_sclk;
    wire spi_mosi;

    sd_spi_byte u_spi_byte (
        .clk(clk),
        .rst_n(rst_n),
        .start(spi_start),
        .clk_div(clk_div_r),
        .tx_byte(spi_tx),
        .rx_byte(spi_rx),
        .busy(spi_busy),
        .done(spi_done),
        .sclk(spi_sclk),
        .mosi(spi_mosi),
        .miso(sd_miso)
    );

    assign sd_cs_n = cs_n_r;
    assign sd_clk = spi_sclk;
    assign sd_mosi = spi_mosi;
    assign req_ready = (state == S_READY);
    assign debug_direct_efs_words = {
        direct_efs_word0,
        direct_efs_word4,
        direct_efs_word8,
        direct_efs_word16,
        direct_efs_word20,
        direct_efs_word28
    };
    assign debug_direct_efs_crc = {direct_efs_crc0, direct_efs_crc1};
    assign debug_read_crc_calc = read_crc_calc;
    assign debug_read_crc_recv = read_crc_recv;
    assign debug_read_crc_mismatch = read_crc_mismatch;

    reg [7:0] cmd_byte;
    always @(*) begin
        case (cmd_byte_idx)
            3'd0: cmd_byte = 8'h40 | {2'b00, cur_cmd[5:0]};
            3'd1: cmd_byte = cur_arg[31:24];
            3'd2: cmd_byte = cur_arg[23:16];
            3'd3: cmd_byte = cur_arg[15:8];
            3'd4: cmd_byte = cur_arg[7:0];
            default: cmd_byte = cur_crc;
        endcase
    end

    function [15:0] crc16_next;
        input [15:0] crc_in;
        input [7:0] data;
        reg [15:0] crc;
        reg [7:0] d;
        integer i;
    begin
        crc = crc_in;
        d = data;
        for (i = 0; i < 8; i = i + 1) begin
            if ((crc[15] ^ d[7]) != 1'b0)
                crc = {crc[14:0], 1'b0} ^ 16'h1021;
            else
                crc = {crc[14:0], 1'b0};
            d = {d[6:0], 1'b0};
        end
        crc16_next = crc;
    end
    endfunction

    task start_cmd;
        input [7:0] cmd;
        input [31:0] arg;
        input [7:0] crc;
        input [7:0] next_state;
    begin
        cur_cmd <= cmd;
        cur_arg <= arg;
        cur_crc <= crc;
        after_r1_state <= next_state;
        cmd_byte_idx <= 3'd0;
        cs_n_r <= 1'b0;
        state <= S_CMD_START;
        last_cmd <= cmd;
        timeout_cnt <= 32'd0;
    end
    endtask

    task idle_byte_to;
        input [7:0] next_state;
    begin
        spi_tx <= 8'hff;
        spi_start <= 1'b1;
        after_idle_state <= next_state;
        state <= S_IDLE_BYTE_WAIT;
    end
    endtask

    task deselect_to;
        input [7:0] next_state;
    begin
        cs_n_r <= 1'b1;
        state <= S_IDLE_BYTE_START;
        after_idle_state <= next_state;
    end
    endtask

    task init_fail;
        input [7:0] code;
    begin
        if (sticky_error == 8'h00)
            sticky_error <= code;
        media_ready <= 1'b0;
        error <= 1'b1;
        state <= S_INIT_ERROR;
        cs_n_r <= 1'b1;
        timeout_cnt <= 32'd0;
    end
    endtask

    task block_fail;
        input [7:0] code;
    begin
        if (sticky_error == 8'h00)
            sticky_error <= code;
        error <= 1'b1;
        done <= 1'b1;
        state <= S_BLOCK_ERROR;
    end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_POWER_IDLE_START;
            after_r1_state <= S_INIT_ERROR;
            after_idle_state <= S_INIT_ERROR;
            cur_cmd <= 8'd0;
            cur_arg <= 32'd0;
            cur_crc <= 8'hff;
            cmd_byte_idx <= 3'd0;
            spi_tx <= 8'hff;
            spi_start <= 1'b0;
            cs_n_r <= 1'b1;
            clk_div_r <= INIT_CLK_DIV;
            timeout_cnt <= 32'd0;
            idle_count <= 16'd0;
            retry_count <= 16'd0;
            byte_count <= 9'd0;
            extra_count <= 2'd0;
            ocr_shift <= 32'd0;
            card_v2 <= 1'b0;
            card_sdhc <= 1'b0;
            active_write <= 1'b0;
            active_lba <= 32'd0;
            last_cmd <= 8'd0;
            last_r1 <= 8'hff;
            last_spi_rx <= 8'hff;
            sticky_error <= 8'h00;
            miso_low_seen <= 1'b0;
            cs_low_seen <= 1'b0;
            sclk_high_seen <= 1'b0;
            mosi_low_seen <= 1'b0;
            spi_byte_count <= 16'd0;
            sclk_edge_count <= 16'd0;
            spi_sclk_d <= 1'b0;
            direct_efs_word0 <= 32'd0;
            direct_efs_word4 <= 32'd0;
            direct_efs_word8 <= 32'd0;
            direct_efs_word16 <= 32'd0;
            direct_efs_word20 <= 32'd0;
            direct_efs_word28 <= 32'd0;
            direct_efs_crc0 <= 8'd0;
            direct_efs_crc1 <= 8'd0;
            read_crc_calc <= 16'd0;
            read_crc_recv <= 16'd0;
            read_crc_mismatch <= 1'b0;
            done <= 1'b0;
            error <= 1'b0;
            media_ready <= 1'b0;
            media_sectors <= 32'd0;
            buf_addr <= 9'd0;
            buf_we <= 1'b0;
            buf_wdata <= 8'h00;
        end else begin
            spi_start <= 1'b0;
            done <= 1'b0;
            error <= 1'b0;
            buf_we <= 1'b0;
            spi_sclk_d <= spi_sclk;
            if (!cs_n_r)
                cs_low_seen <= 1'b1;
            if (spi_sclk)
                sclk_high_seen <= 1'b1;
            if (!spi_mosi)
                mosi_low_seen <= 1'b1;
            if (spi_sclk != spi_sclk_d)
                sclk_edge_count <= sclk_edge_count + 16'd1;
            if (spi_done) begin
                spi_byte_count <= spi_byte_count + 16'd1;
                last_spi_rx <= spi_rx;
                if (spi_rx != 8'hff || !sd_miso)
                    miso_low_seen <= 1'b1;
            end

            case (state)
                S_POWER_IDLE_START: begin
                    cs_n_r <= 1'b1;
                    clk_div_r <= INIT_CLK_DIV;
                    idle_count <= 16'd0;
                    spi_tx <= 8'hff;
                    spi_start <= 1'b1;
                    state <= S_POWER_IDLE_WAIT;
                end

                S_POWER_IDLE_WAIT: begin
                    if (spi_done) begin
                        idle_count <= idle_count + 16'd1;
                        if (idle_count == 16'd9)
                            start_cmd(8'd0, 32'd0, 8'h95, S_CMD0_DONE);
                        else begin
                            spi_tx <= 8'hff;
                            spi_start <= 1'b1;
                        end
                    end
                end

                S_IDLE_BYTE_START: begin
                    spi_tx <= 8'hff;
                    spi_start <= 1'b1;
                    state <= S_IDLE_BYTE_WAIT;
                end

                S_IDLE_BYTE_WAIT: begin
                    if (spi_done)
                        state <= after_idle_state;
                end

                S_CMD_START: begin
                    spi_tx <= cmd_byte;
                    spi_start <= 1'b1;
                    state <= S_CMD_WAIT_BYTE;
                end

                S_CMD_WAIT_BYTE: begin
                    if (spi_done) begin
                        if (cmd_byte_idx == 3'd5) begin
                            timeout_cnt <= 32'd0;
                            state <= S_CMD_R1_START;
                        end else begin
                            cmd_byte_idx <= cmd_byte_idx + 3'd1;
                            state <= S_CMD_START;
                        end
                    end
                end

                S_CMD_R1_START: begin
                    spi_tx <= 8'hff;
                    spi_start <= 1'b1;
                    state <= S_CMD_R1_WAIT;
                end

                S_CMD_R1_WAIT: begin
                    if (spi_done) begin
                        if (!spi_rx[7]) begin
                            last_r1 <= spi_rx;
                            state <= after_r1_state;
                        end else if (timeout_cnt >= CMD_TIMEOUT) begin
                            last_r1 <= spi_rx;
                            if (media_ready)
                                block_fail(8'h11);
                            else
                                init_fail(8'h11);
                        end else begin
                            timeout_cnt <= timeout_cnt + 32'd1;
                            state <= S_CMD_R1_START;
                        end
                    end
                end

                S_CMD0_DONE: begin
                    if (last_r1 == 8'h01)
                        deselect_to(S_CMD8_DONE);
                    else
                        init_fail(8'h20);
                end

                S_CMD8_DONE: begin
                    start_cmd(8'd8, 32'h0000_01aa, 8'h87, S_CMD8_R7_START);
                end

                S_CMD8_R7_START: begin
                    if (last_r1[2]) begin
                        card_v2 <= 1'b0;
                        deselect_to(S_CMD55_DONE);
                    end else if (last_r1 == 8'h01) begin
                        card_v2 <= 1'b1;
                        extra_count <= 2'd0;
                        ocr_shift <= 32'd0;
                        spi_tx <= 8'hff;
                        spi_start <= 1'b1;
                        state <= S_CMD8_R7_WAIT;
                    end else begin
                        init_fail(8'h21);
                    end
                end

                S_CMD8_R7_WAIT: begin
                    if (spi_done) begin
                        ocr_shift <= {ocr_shift[23:0], spi_rx};
                        if (extra_count == 2'd3) begin
                            if (ocr_shift[7:0] != 8'h01 || spi_rx != 8'haa)
                                init_fail(8'h22);
                            else begin
                                retry_count <= 16'd0;
                                deselect_to(S_CMD55_DONE);
                            end
                        end else begin
                            extra_count <= extra_count + 2'd1;
                            spi_tx <= 8'hff;
                            spi_start <= 1'b1;
                        end
                    end
                end

                S_CMD55_DONE: begin
                    start_cmd(8'd55, 32'd0, 8'h65, S_ACMD41_DONE);
                end

                S_ACMD41_DONE: begin
                    if (last_cmd == 8'd55) begin
                        if (last_r1 == 8'h01 || last_r1 == 8'h00)
                            deselect_to(S_ACMD41_START);
                        else
                            init_fail(8'h23);
                    end else begin
                        if (last_r1 == 8'h00) begin
                            deselect_to(S_CMD58_DONE);
                        end else if (last_r1 == 8'h01 &&
                                     retry_count < ACMD41_RETRIES) begin
                            retry_count <= retry_count + 16'd1;
                            deselect_to(S_CMD55_DONE);
                        end else begin
                            init_fail(8'h24);
                        end
                    end
                end

                S_ACMD41_START: begin
                    start_cmd(8'd41, card_v2 ? 32'h4000_0000 : 32'd0,
                              card_v2 ? 8'h77 : 8'he5, S_ACMD41_DONE);
                end

                S_CMD58_DONE: begin
                    start_cmd(8'd58, 32'd0, 8'hfd, S_CMD58_OCR_START);
                end

                S_CMD58_OCR_START: begin
                    if (last_r1 != 8'h00) begin
                        init_fail(8'h25);
                    end else begin
                        extra_count <= 2'd0;
                        ocr_shift <= 32'd0;
                        spi_tx <= 8'hff;
                        spi_start <= 1'b1;
                        state <= S_CMD58_OCR_WAIT;
                    end
                end

                S_CMD58_OCR_WAIT: begin
                    if (spi_done) begin
                        ocr_shift <= {ocr_shift[23:0], spi_rx};
                        if (extra_count == 2'd3) begin
                            card_sdhc <= ocr_shift[22];
                            media_ready <= 1'b1;
                            clk_div_r <= RUN_CLK_DIV;
                            deselect_to(S_READY);
                        end else begin
                            extra_count <= extra_count + 2'd1;
                            spi_tx <= 8'hff;
                            spi_start <= 1'b1;
                        end
                    end
                end

                S_READY: begin
                    if (req_valid) begin
                        active_lba <= req_lba;
                        active_write <= req_write;
                        byte_count <= 9'd0;
                        if (!req_write && req_lba == 32'h0000_7d83) begin
                            direct_efs_word0 <= 32'd0;
                            direct_efs_word4 <= 32'd0;
                            direct_efs_word8 <= 32'd0;
                            direct_efs_word16 <= 32'd0;
                            direct_efs_word20 <= 32'd0;
                            direct_efs_word28 <= 32'd0;
                            direct_efs_crc0 <= 8'd0;
                            direct_efs_crc1 <= 8'd0;
                        end
                        if (req_write)
                            start_cmd(8'd24,
                                      card_sdhc ? req_lba : {req_lba[22:0], 9'd0},
                                      8'hff, S_WRITE_CMD_DONE);
                        else
                            start_cmd(8'd17,
                                      card_sdhc ? req_lba : {req_lba[22:0], 9'd0},
                                      8'hff, S_READ_CMD_DONE);
                    end
                end

                S_READ_CMD_DONE: begin
                    if (last_r1 == 8'h00) begin
                        timeout_cnt <= 32'd0;
                        state <= S_READ_TOKEN_START;
                    end else begin
                        block_fail(8'h30);
                    end
                end

                S_READ_TOKEN_START: begin
                    spi_tx <= 8'hff;
                    spi_start <= 1'b1;
                    state <= S_READ_TOKEN_WAIT;
                end

                S_READ_TOKEN_WAIT: begin
                    if (spi_done) begin
                        if (spi_rx == 8'hfe) begin
                            byte_count <= 9'd0;
                            read_crc_calc <= 16'd0;
                            read_crc_recv <= 16'd0;
                            read_crc_mismatch <= 1'b0;
                            state <= read_skip_first_byte ? S_READ_ALIGN_START :
                                                          S_READ_DATA_START;
                        end else if (timeout_cnt >= DATA_TIMEOUT) begin
                            block_fail(8'h31);
                        end else begin
                            timeout_cnt <= timeout_cnt + 32'd1;
                            state <= S_READ_TOKEN_START;
                        end
                    end
                end

                S_READ_ALIGN_START: begin
                    spi_tx <= 8'hff;
                    spi_start <= 1'b1;
                    state <= S_READ_ALIGN_WAIT;
                end

                S_READ_ALIGN_WAIT: begin
                    if (spi_done)
                        state <= S_READ_DATA_START;
                end

                S_READ_DATA_START: begin
                    spi_tx <= 8'hff;
                    spi_start <= 1'b1;
                    state <= S_READ_DATA_WAIT;
                end

                S_READ_DATA_WAIT: begin
                    if (spi_done) begin
                        buf_addr <= byte_count;
                        buf_wdata <= spi_rx;
                        buf_we <= 1'b1;
                        read_crc_calc <= crc16_next(read_crc_calc, spi_rx);
                        if (active_lba == 32'h0000_7d83) begin
                            case (byte_count)
                                9'd0:  direct_efs_word0[31:24] <= spi_rx;
                                9'd1:  direct_efs_word0[23:16] <= spi_rx;
                                9'd2:  direct_efs_word0[15:8]  <= spi_rx;
                                9'd3:  direct_efs_word0[7:0]   <= spi_rx;
                                9'd4:  direct_efs_word4[31:24] <= spi_rx;
                                9'd5:  direct_efs_word4[23:16] <= spi_rx;
                                9'd6:  direct_efs_word4[15:8]  <= spi_rx;
                                9'd7:  direct_efs_word4[7:0]   <= spi_rx;
                                9'd8:  direct_efs_word8[31:24] <= spi_rx;
                                9'd9:  direct_efs_word8[23:16] <= spi_rx;
                                9'd10: direct_efs_word8[15:8]  <= spi_rx;
                                9'd11: direct_efs_word8[7:0]   <= spi_rx;
                                9'd16: direct_efs_word16[31:24] <= spi_rx;
                                9'd17: direct_efs_word16[23:16] <= spi_rx;
                                9'd18: direct_efs_word16[15:8]  <= spi_rx;
                                9'd19: direct_efs_word16[7:0]   <= spi_rx;
                                9'd20: direct_efs_word20[31:24] <= spi_rx;
                                9'd21: direct_efs_word20[23:16] <= spi_rx;
                                9'd22: direct_efs_word20[15:8]  <= spi_rx;
                                9'd23: direct_efs_word20[7:0]   <= spi_rx;
                                9'd28: direct_efs_word28[31:24] <= spi_rx;
                                9'd29: direct_efs_word28[23:16] <= spi_rx;
                                9'd30: direct_efs_word28[15:8]  <= spi_rx;
                                9'd31: direct_efs_word28[7:0]   <= spi_rx;
                                default: ;
                            endcase
                        end
                        if (byte_count == 9'd511) begin
                            extra_count <= 2'd0;
                            state <= S_READ_CRC_START;
                        end else begin
                            byte_count <= byte_count + 9'd1;
                            state <= S_READ_DATA_START;
                        end
                    end
                end

                S_READ_CRC_START: begin
                    spi_tx <= 8'hff;
                    spi_start <= 1'b1;
                    state <= S_READ_CRC_WAIT;
                end

                S_READ_CRC_WAIT: begin
                    if (spi_done) begin
                        if (active_lba == 32'h0000_7d83) begin
                            if (extra_count == 2'd0)
                                direct_efs_crc0 <= spi_rx;
                            else if (extra_count == 2'd1)
                                direct_efs_crc1 <= spi_rx;
                        end
                        if (extra_count == 2'd0) begin
                            read_crc_recv[15:8] <= spi_rx;
                            extra_count <= extra_count + 2'd1;
                            state <= S_READ_CRC_START;
                        end else if (extra_count == 2'd1) begin
                            read_crc_recv[7:0] <= spi_rx;
                            read_crc_mismatch <=
                                ({read_crc_recv[15:8], spi_rx} != read_crc_calc);
                            deselect_to(S_BLOCK_DONE);
                        end else begin
                            extra_count <= extra_count + 2'd1;
                            state <= S_READ_CRC_START;
                        end
                    end
                end

                S_WRITE_CMD_DONE: begin
                    if (last_r1 == 8'h00)
                        state <= S_WRITE_GAP_START;
                    else
                        block_fail(8'h40);
                end

                S_WRITE_GAP_START: begin
                    spi_tx <= 8'hff;
                    spi_start <= 1'b1;
                    state <= S_WRITE_GAP_WAIT;
                end

                S_WRITE_GAP_WAIT: begin
                    if (spi_done)
                        state <= S_WRITE_TOKEN_START;
                end

                S_WRITE_TOKEN_START: begin
                    spi_tx <= 8'hfe;
                    spi_start <= 1'b1;
                    byte_count <= 9'd0;
                    state <= S_WRITE_TOKEN_WAIT;
                end

                S_WRITE_TOKEN_WAIT: begin
                    if (spi_done)
                        state <= S_WRITE_DATA_START;
                end

                S_WRITE_DATA_START: begin
                    buf_addr <= byte_count;
                    state <= S_WRITE_DATA_LATCH;
                end

                S_WRITE_DATA_LATCH: begin
                    spi_tx <= buf_rdata;
                    spi_start <= 1'b1;
                    state <= S_WRITE_DATA_WAIT;
                end

                S_WRITE_DATA_WAIT: begin
                    if (spi_done) begin
                        if (byte_count == 9'd511) begin
                            extra_count <= 2'd0;
                            state <= S_WRITE_CRC_START;
                        end else begin
                            byte_count <= byte_count + 9'd1;
                            state <= S_WRITE_DATA_START;
                        end
                    end
                end

                S_WRITE_CRC_START: begin
                    spi_tx <= 8'hff;
                    spi_start <= 1'b1;
                    state <= S_WRITE_CRC_WAIT;
                end

                S_WRITE_CRC_WAIT: begin
                    if (spi_done) begin
                        if (extra_count == 2'd1)
                            state <= S_WRITE_RESP_START;
                        else begin
                            extra_count <= extra_count + 2'd1;
                            state <= S_WRITE_CRC_START;
                        end
                    end
                end

                S_WRITE_RESP_START: begin
                    spi_tx <= 8'hff;
                    spi_start <= 1'b1;
                    state <= S_WRITE_RESP_WAIT;
                end

                S_WRITE_RESP_WAIT: begin
                    if (spi_done) begin
                        if ((spi_rx & 8'h1f) == 8'h05) begin
                            timeout_cnt <= 32'd0;
                            state <= S_WRITE_BUSY_START;
                        end else begin
                            block_fail(8'h41);
                        end
                    end
                end

                S_WRITE_BUSY_START: begin
                    spi_tx <= 8'hff;
                    spi_start <= 1'b1;
                    state <= S_WRITE_BUSY_WAIT;
                end

                S_WRITE_BUSY_WAIT: begin
                    if (spi_done) begin
                        if (spi_rx == 8'hff) begin
                            deselect_to(S_BLOCK_DONE);
                        end else if (timeout_cnt >= DATA_TIMEOUT) begin
                            block_fail(8'h42);
                        end else begin
                            timeout_cnt <= timeout_cnt + 32'd1;
                            state <= S_WRITE_BUSY_START;
                        end
                    end
                end

                S_BLOCK_DONE: begin
                    done <= 1'b1;
                    state <= S_READY;
                end

                S_BLOCK_ERROR: begin
                    deselect_to(S_READY);
                end

                S_INIT_ERROR: begin
                    cs_n_r <= 1'b1;
                    if (timeout_cnt >= INIT_RETRY_DELAY) begin
                        timeout_cnt <= 32'd0;
                        idle_count <= 16'd0;
                        retry_count <= 16'd0;
                        last_r1 <= 8'hff;
                        card_v2 <= 1'b0;
                        card_sdhc <= 1'b0;
                        media_ready <= 1'b0;
                        state <= S_POWER_IDLE_START;
                    end else begin
                        timeout_cnt <= timeout_cnt + 32'd1;
                    end
                end

                default: state <= S_INIT_ERROR;
            endcase
        end
    end

    assign debug_probe = {
        32'h5344_5350, // "SDSP"
        state,
        media_ready,
        card_v2,
        card_sdhc,
        req_valid,
        req_write,
        req_ready,
        done,
        error,
        last_cmd,
        last_r1,
        sticky_error,
        sd_card_detect,
        cs_n_r,
        spi_mosi,
        sd_miso,
        cs_low_seen,
        sclk_high_seen,
        mosi_low_seen,
        miso_low_seen,
        last_spi_rx,
        read_skip_first_byte,
        idle_count[6:0],
        spi_byte_count,
        sclk_edge_count
    };

endmodule

`default_nettype wire
