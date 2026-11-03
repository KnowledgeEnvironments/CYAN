// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - SD-card backed raw SCSI target.
//
// KCU105 local boot uses the onboard micro-SD connector in SPI mode as a raw
// block device. The card contains an SGI disk image written 1:1; no filesystem
// parser lives in FPGA RTL.

`default_nettype none

module sd_scsi_bridge #(
    parameter [31:0] DISK_SECTORS = 32'd4_194_304,
    parameter READ_ONLY = 0,
    parameter [15:0] INIT_CLK_DIV = 16'd250,
    parameter [15:0] RUN_CLK_DIV = 16'd4
) (
    input  wire        clk,
    input  wire        rst_n,

    // ---- WD33C93 target backend side ----
    input  wire        scsi_bus_reset,
    input  wire        scsi_cmd_pending,
    output wire        scsi_cmd_done,
    output wire [7:0]  scsi_cmd_status,
    output wire [23:0] scsi_resp_len,
    output wire        scsi_status_valid,
    input  wire [2:0]  scsi_target_id,
    input  wire [95:0] scsi_cdb,
    input  wire [23:0] scsi_xfer_count,
    input  wire [23:0] scsi_data_index,
    output wire [7:0]  scsi_read_data,
    output wire        scsi_read_valid,
    input  wire        scsi_read_consume,
    input  wire        scsi_write_valid,
    input  wire [7:0]  scsi_write_data,
    output wire        scsi_write_ready,
    input  wire        scsi_data_out_done,

    // ---- Physical SD card pins (SPI mode) ----
    output wire        sd_cs_n,
    output wire        sd_clk,
    output wire        sd_mosi,
    input  wire        sd_miso,
    input  wire        sd_card_detect,
    input  wire        read_skip_first_byte,
    input  wire        debug_read_trigger,
    input  wire [2:0]  debug_lba_select,
    input  wire        debug_probe_select,

    output wire [31:0] debug_inode1856_write_status,
    output wire [255:0] debug_probe
);
    wire        block_req_valid;
    wire        block_req_write;
    wire [31:0] block_req_lba;
    wire        block_req_ready;
    wire        sd_req_valid;
    wire        sd_req_write;
    wire [31:0] sd_req_lba;
    wire        sd_req_ready;
    wire        sd_done;
    wire        sd_error;
    wire        block_done;
    wire        block_error;
    wire        media_ready;
    wire [31:0] media_sectors;

    wire [8:0]  target_buf_addr;
    wire [7:0]  target_buf_rdata;
    wire        target_buf_we;
    wire [7:0]  target_buf_wdata;
    wire [8:0]  sd_buf_addr;
    wire [7:0]  sd_buf_rdata;
    wire        sd_buf_we;
    wire [7:0]  sd_buf_wdata;

    reg [7:0] sector_buf [0:511];

    assign target_buf_rdata = sector_buf[target_buf_addr];
    assign sd_buf_rdata = sector_buf[sd_buf_addr];

    always @(posedge clk) begin
        if (target_buf_we)
            sector_buf[target_buf_addr] <= target_buf_wdata;
        if (sd_buf_we)
            sector_buf[sd_buf_addr] <= sd_buf_wdata;
    end

    wire [255:0] target_debug_probe;
    wire [127:0] sd_debug_probe;
    wire [7:0] sd_sticky_error = sd_debug_probe[63:56];
    wire [15:0] sd_direct_efs_crc;
    wire [191:0] sd_direct_efs_words;
    wire [15:0] sd_read_crc_calc;
    wire [15:0] sd_read_crc_recv;
    wire        sd_read_crc_mismatch;
    wire [7:0] target_scsi_read_data;
    wire       target_scsi_read_valid;

    reg        debug_read_trigger_d;
    reg        debug_trigger_sample_valid;
    reg        debug_req_pending;
    reg        debug_active;
    reg [31:0] debug_lba_r;
    reg [31:0] debug_capture_lba;
    reg [31:0] debug_word0;
    reg [31:0] debug_word4;
    reg [31:0] debug_word8;
    reg [31:0] debug_word28;
    reg [15:0] debug_crc_calc;
    reg [15:0] debug_crc_recv;
    reg        debug_crc_mismatch;
    reg [8:0]  debug_word_offset0;
    reg [8:0]  debug_word_offset1;
    reg [8:0]  debug_word_offset2;
    reg [8:0]  debug_word_offset3;
    reg [7:0]  debug_status;
    reg [7:0]  debug_read_count;
    reg [31:0] last_read_lba;
    reg [31:0] last_read_word0;
    reg [31:0] last_read_word4;
    reg [31:0] last_read_word8;
    reg [31:0] last_read_word28;
    reg [31:0] efs_lba_word0;
    reg [31:0] efs_lba_word28;
    reg [15:0] read_done_count;
    reg        seen_lba0;
    reg        seen_part0_start;
    reg        seen_efs_lba;
    reg        seen_efs_magic;
    reg [31:0] last_write_lba;
    reg [31:0] last_write_word0;
    reg [31:0] last_write_word4;
    reg [31:0] last_write_word8;
    reg [31:0] last_write_word28;
    reg [15:0] write_accept_count;
    reg [15:0] write_done_count;
    reg [7:0]  write_debug_status;
    reg [7:0]  last_write_error;
    reg [15:0] inode1856_write_accept_count;
    reg [7:0]  inode1856_write_done_count;
    reg [7:0]  inode1856_write_error_count;
    reg [2:0]  scsi_reset_count;
    reg        scsi_bus_reset_d;

    function [31:0] selected_debug_lba;
        input [2:0] select;
    begin
        case (select)
            3'd0: selected_debug_lba = 32'h0000_0000;
            3'd1: selected_debug_lba = 32'h0000_0116;
            3'd2: selected_debug_lba = 32'h0000_7d83;
            // Retained storage-debug sector used to distinguish media data
            // from cache or transfer corruption during startup.
            3'd3: selected_debug_lba = 32'h0000_8133;
            // Retained sector that once returned zero words during a live
            // target transfer. Reading it directly separates storage data
            // from cache corruption.
            3'd4: selected_debug_lba = 32'h0000_8de1;
            3'd5: selected_debug_lba = last_read_lba;
            3'd6: selected_debug_lba = last_write_lba;
            3'd7: selected_debug_lba = last_write_lba;
            default: selected_debug_lba = 32'h0000_7d83;
        endcase
    end
    endfunction

    function [8:0] selected_debug_word_offset;
        input [2:0] select;
        input [1:0] slot;
    begin
        if (select == 3'd4) begin
            case (slot)
                2'd0: selected_debug_word_offset = 9'h038;
                2'd1: selected_debug_word_offset = 9'h048;
                2'd2: selected_debug_word_offset = 9'h0a0;
                default: selected_debug_word_offset = 9'h1b8;
            endcase
        end else begin
            case (slot)
                2'd0: selected_debug_word_offset = 9'd0;
                2'd1: selected_debug_word_offset = 9'd4;
                2'd2: selected_debug_word_offset = 9'd8;
                default: selected_debug_word_offset = 9'd28;
            endcase
        end
    end
    endfunction

    wire debug_trigger_edge =
        debug_trigger_sample_valid && (debug_read_trigger ^ debug_read_trigger_d);
    wire debug_req_valid = debug_req_pending && !block_req_valid;
    wire debug_req_accept = debug_req_valid && sd_req_ready;

    assign sd_req_valid = debug_req_valid || block_req_valid;
    assign sd_req_write = debug_req_valid ? 1'b0 : block_req_write;
    assign sd_req_lba = debug_req_valid ? debug_lba_r : block_req_lba;
    assign block_req_ready = !debug_req_valid && sd_req_ready;
    assign block_done = !debug_active && sd_done;
    assign block_error = !debug_active && sd_error;
    wire block_write_accept = block_req_valid && block_req_write &&
                              block_req_ready;

	always @(posedge clk) begin
	    if (!rst_n || scsi_bus_reset) begin
	        debug_read_trigger_d <= debug_read_trigger;
	        debug_trigger_sample_valid <= 1'b0;
	    end else begin
	        debug_read_trigger_d <= debug_read_trigger;
	        debug_trigger_sample_valid <= 1'b1;
	    end
	end

	scsi_raw_disk_target #(
        .DISK_SECTORS(DISK_SECTORS),
        .READ_ONLY(READ_ONLY)
    ) u_target (
        .clk(clk),
        .rst_n(rst_n),
        .scsi_bus_reset(scsi_bus_reset),
        .media_ready(media_ready),
        .media_sectors(media_sectors),
        .scsi_cmd_pending(scsi_cmd_pending),
        .scsi_cmd_done(scsi_cmd_done),
        .scsi_cmd_status(scsi_cmd_status),
        .scsi_resp_len(scsi_resp_len),
        .scsi_status_valid(scsi_status_valid),
        .scsi_target_id(scsi_target_id),
        .scsi_cdb(scsi_cdb),
        .scsi_xfer_count(scsi_xfer_count),
        .scsi_data_index(scsi_data_index),
        .scsi_read_data(target_scsi_read_data),
        .scsi_read_valid(target_scsi_read_valid),
        .scsi_read_consume(scsi_read_consume),
        .scsi_write_valid(scsi_write_valid),
        .scsi_write_data(scsi_write_data),
        .scsi_write_ready(scsi_write_ready),
        .scsi_data_out_done(scsi_data_out_done),
        .block_req_valid(block_req_valid),
        .block_req_write(block_req_write),
        .block_req_lba(block_req_lba),
        .block_req_ready(block_req_ready),
        .block_done(block_done),
        .block_error(block_error),
        .buf_addr(target_buf_addr),
        .buf_rdata(target_buf_rdata),
        .buf_we(target_buf_we),
        .buf_wdata(target_buf_wdata),
        .debug_probe(target_debug_probe)
    );

    sd_spi_block #(
        .INIT_CLK_DIV(INIT_CLK_DIV),
        .RUN_CLK_DIV(RUN_CLK_DIV)
    ) u_sd (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(sd_req_valid),
        .req_write(sd_req_write),
        .req_lba(sd_req_lba),
        .req_ready(sd_req_ready),
        .done(sd_done),
        .error(sd_error),
        .media_ready(media_ready),
        .media_sectors(media_sectors),
        .buf_addr(sd_buf_addr),
        .buf_rdata(sd_buf_rdata),
        .buf_we(sd_buf_we),
        .buf_wdata(sd_buf_wdata),
        .sd_cs_n(sd_cs_n),
        .sd_clk(sd_clk),
        .sd_mosi(sd_mosi),
        .sd_miso(sd_miso),
        .sd_card_detect(sd_card_detect),
        .read_skip_first_byte(read_skip_first_byte),
        .debug_read_crc_calc(sd_read_crc_calc),
        .debug_read_crc_recv(sd_read_crc_recv),
        .debug_read_crc_mismatch(sd_read_crc_mismatch),
        .debug_direct_efs_crc(sd_direct_efs_crc),
        .debug_direct_efs_words(sd_direct_efs_words),
        .debug_probe(sd_debug_probe)
    );

    wire [31:0] sector_word0 = {
        sector_buf[9'd0], sector_buf[9'd1],
        sector_buf[9'd2], sector_buf[9'd3]
    };
    wire [31:0] sector_word4 = {
        sector_buf[9'd4], sector_buf[9'd5],
        sector_buf[9'd6], sector_buf[9'd7]
    };
    wire [31:0] sector_word8 = {
        sector_buf[9'd8], sector_buf[9'd9],
        sector_buf[9'd10], sector_buf[9'd11]
    };
    wire [31:0] sector_word28 = {
        sector_buf[9'd28], sector_buf[9'd29],
        sector_buf[9'd30], sector_buf[9'd31]
    };
    wire efs_shifted_superblock =
        (last_read_lba == 32'h0000_7d83) &&
        (sector_word0 == 32'h0000_1de7) &&
        (sector_word4 == 32'hd500_0001) &&
        (sector_word8 == 32'he100_009c) &&
        (sector_word28 == 32'h3f00_0729);
    wire [8:0] corrected_efs_addr = scsi_data_index[8:0] + 9'd1;
    wire [7:0] corrected_efs_data =
        (scsi_data_index[8:0] == 9'd511) ? 8'h00 :
                                           sector_buf[corrected_efs_addr];

    assign scsi_read_valid = target_scsi_read_valid;
    assign scsi_read_data =
        (target_scsi_read_valid && efs_shifted_superblock) ?
        corrected_efs_data : target_scsi_read_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            debug_req_pending <= 1'b0;
            debug_active <= 1'b0;
            debug_lba_r <= 32'd0;
            debug_capture_lba <= 32'd0;
            debug_word0 <= 32'd0;
            debug_word4 <= 32'd0;
            debug_word8 <= 32'd0;
            debug_word28 <= 32'd0;
            debug_crc_calc <= 16'd0;
            debug_crc_recv <= 16'd0;
            debug_crc_mismatch <= 1'b0;
            debug_word_offset0 <= 9'd0;
            debug_word_offset1 <= 9'd4;
            debug_word_offset2 <= 9'd8;
            debug_word_offset3 <= 9'd28;
            debug_status <= 8'h00;
            debug_read_count <= 8'd0;
        end else if (scsi_bus_reset) begin
            debug_req_pending <= 1'b0;
            debug_active <= 1'b0;
            debug_lba_r <= 32'd0;
            debug_capture_lba <= 32'd0;
            debug_word0 <= 32'd0;
            debug_word4 <= 32'd0;
            debug_word8 <= 32'd0;
            debug_word28 <= 32'd0;
            debug_crc_calc <= 16'd0;
            debug_crc_recv <= 16'd0;
            debug_crc_mismatch <= 1'b0;
            debug_word_offset0 <= 9'd0;
            debug_word_offset1 <= 9'd4;
            debug_word_offset2 <= 9'd8;
            debug_word_offset3 <= 9'd28;
            debug_status <= 8'h00;
            debug_read_count <= 8'd0;
        end else begin
            if (debug_trigger_edge) begin
                if (!debug_req_pending && !debug_active) begin
                    debug_req_pending <= 1'b1;
                    debug_lba_r <= selected_debug_lba(debug_lba_select);
                    debug_status <= 8'h01;
                end else begin
                    debug_status <= 8'he1;
                end
            end

            if (debug_req_accept) begin
                debug_req_pending <= 1'b0;
                debug_active <= 1'b1;
                debug_capture_lba <= debug_lba_r;
                debug_word_offset0 <= selected_debug_word_offset(debug_lba_select, 2'd0);
                debug_word_offset1 <= selected_debug_word_offset(debug_lba_select, 2'd1);
                debug_word_offset2 <= selected_debug_word_offset(debug_lba_select, 2'd2);
                debug_word_offset3 <= selected_debug_word_offset(debug_lba_select, 2'd3);
                debug_status <= 8'h02;
            end

            if (debug_active && sd_done) begin
                debug_active <= 1'b0;
                debug_status <= sd_error ? 8'h80 : 8'h10;
                if (debug_read_count != 8'hff)
                    debug_read_count <= debug_read_count + 8'd1;
                if (!sd_error) begin
                    debug_word0 <= {
                        sector_buf[debug_word_offset0 + 9'd0],
                        sector_buf[debug_word_offset0 + 9'd1],
                        sector_buf[debug_word_offset0 + 9'd2],
                        sector_buf[debug_word_offset0 + 9'd3]
                    };
                    debug_word4 <= {
                        sector_buf[debug_word_offset1 + 9'd0],
                        sector_buf[debug_word_offset1 + 9'd1],
                        sector_buf[debug_word_offset1 + 9'd2],
                        sector_buf[debug_word_offset1 + 9'd3]
                    };
                    debug_word8 <= {
                        sector_buf[debug_word_offset2 + 9'd0],
                        sector_buf[debug_word_offset2 + 9'd1],
                        sector_buf[debug_word_offset2 + 9'd2],
                        sector_buf[debug_word_offset2 + 9'd3]
                    };
                    debug_word28 <= {
                        sector_buf[debug_word_offset3 + 9'd0],
                        sector_buf[debug_word_offset3 + 9'd1],
                        sector_buf[debug_word_offset3 + 9'd2],
                        sector_buf[debug_word_offset3 + 9'd3]
                    };
	                    debug_crc_calc <= sd_read_crc_calc;
	                    debug_crc_recv <= sd_read_crc_recv;
	                    debug_crc_mismatch <= sd_read_crc_mismatch;
	                end
	            end
	        end
	    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_read_lba <= 32'd0;
            last_read_word0 <= 32'd0;
            last_read_word4 <= 32'd0;
            last_read_word8 <= 32'd0;
            last_read_word28 <= 32'd0;
            efs_lba_word0 <= 32'd0;
            efs_lba_word28 <= 32'd0;
            read_done_count <= 16'd0;
            seen_lba0 <= 1'b0;
            seen_part0_start <= 1'b0;
            seen_efs_lba <= 1'b0;
            seen_efs_magic <= 1'b0;
            last_write_lba <= 32'd0;
            last_write_word0 <= 32'd0;
            last_write_word4 <= 32'd0;
            last_write_word8 <= 32'd0;
            last_write_word28 <= 32'd0;
            write_accept_count <= 16'd0;
            write_done_count <= 16'd0;
            write_debug_status <= 8'h00;
            last_write_error <= 8'h00;
            inode1856_write_accept_count <= 16'd0;
            inode1856_write_done_count <= 8'd0;
            inode1856_write_error_count <= 8'd0;
            scsi_reset_count <= 3'd0;
            scsi_bus_reset_d <= 1'b0;
        end else begin
            scsi_bus_reset_d <= scsi_bus_reset;
            if (scsi_bus_reset && !scsi_bus_reset_d &&
                scsi_reset_count != 3'h7)
                scsi_reset_count <= scsi_reset_count + 3'd1;

            if (!scsi_bus_reset && block_done && !block_error && !block_req_write) begin
                last_read_lba <= block_req_lba;
                last_read_word0 <= sector_word0;
                last_read_word4 <= sector_word4;
                last_read_word8 <= sector_word8;
                last_read_word28 <= sector_word28;
                if (read_done_count != 16'hffff)
                    read_done_count <= read_done_count + 16'd1;
                if (block_req_lba == 32'd0)
                    seen_lba0 <= 1'b1;
                if (block_req_lba == 32'h0000_7d82)
                    seen_part0_start <= 1'b1;
                if (block_req_lba == 32'h0000_7d83) begin
                    seen_efs_lba <= 1'b1;
                    efs_lba_word0 <= sector_word0;
                    efs_lba_word28 <= sector_word28;
                    if ((sector_word28 == 32'h0007_2959) ||
                        ((sector_word0 == 32'h0000_1de7) &&
                         (sector_word4 == 32'hd500_0001) &&
                         (sector_word8 == 32'he100_009c) &&
                         (sector_word28 == 32'h3f00_0729)))
                        seen_efs_magic <= 1'b1;
                end
            end

            if (!scsi_bus_reset && block_write_accept) begin
                last_write_lba <= block_req_lba;
                last_write_word0 <= sector_word0;
                last_write_word4 <= sector_word4;
                last_write_word8 <= sector_word8;
                last_write_word28 <= sector_word28;
                write_debug_status <= 8'h02;
                last_write_error <= 8'h00;
                if (write_accept_count != 16'hffff)
                    write_accept_count <= write_accept_count + 16'd1;
                if (block_req_lba == 32'h0000_8133 &&
                    inode1856_write_accept_count != 16'hffff)
                    inode1856_write_accept_count <=
                        inode1856_write_accept_count + 16'd1;
            end

            if (!scsi_bus_reset && block_done && block_req_write) begin
                write_debug_status <= block_error ? 8'h80 : 8'h10;
                last_write_error <= block_error ? sd_sticky_error : 8'h00;
                if (write_done_count != 16'hffff)
                    write_done_count <= write_done_count + 16'd1;
                if (block_req_lba == 32'h0000_8133) begin
                    if (inode1856_write_done_count != 8'hff)
                        inode1856_write_done_count <=
                            inode1856_write_done_count + 8'd1;
                    if (block_error && inode1856_write_error_count != 8'hff)
                        inode1856_write_error_count <=
                            inode1856_write_error_count + 8'd1;
                end
            end
        end
    end

    assign debug_inode1856_write_status = {
        inode1856_write_accept_count,
        inode1856_write_done_count,
        inode1856_write_error_count
    };

    wire [31:0] read_history_word = {
        read_done_count,
        scsi_reset_count,
        read_skip_first_byte,
        seen_efs_magic,
        seen_efs_lba,
        seen_part0_start,
        seen_lba0,
        sd_sticky_error
    };

    wire [31:0] debug_history_word = {
        debug_read_count,
        debug_status,
        debug_lba_select,
        debug_req_pending,
        debug_active,
        debug_probe_select,
        debug_read_trigger,
        debug_crc_mismatch,
        sd_sticky_error
    };

    wire [31:0] efs_direct_history_word = {
        sd_direct_efs_crc,
        read_history_word[15:0]
    };

    wire [31:0] write_status_word = {
        write_debug_status,
        last_write_error,
        sd_sticky_error,
        scsi_reset_count,
        debug_lba_select,
        debug_req_pending,
        debug_active
    };

    wire [31:0] sd_status_word = sd_debug_probe[95:64];

    wire [255:0] read_debug_probe = {
        32'h5343_4452,              // "SCDR"
        sd_status_word,
        last_read_lba,
        read_history_word,
        last_read_word0,
        last_read_word4,
        last_read_word8,
        last_read_word28
    };

    wire [255:0] normal_debug_probe =
        (debug_lba_select == 3'd5) ? read_debug_probe : target_debug_probe;

    wire [255:0] diag_debug_probe = {
        32'h5344_4447,              // "SDDG"
        debug_crc_recv,
        debug_crc_calc,
        debug_capture_lba,
        debug_history_word,
        debug_word0,
        debug_word4,
        debug_word8,
        debug_word28
    };

    wire [255:0] efs_direct_debug_probe = {
        32'h4546_5344,              // "EFSD"
        efs_direct_history_word,
        sd_direct_efs_words
    };

    wire [255:0] write_debug_probe = {
        32'h5344_5752,              // "SDWR"
        {write_accept_count, write_done_count},
        last_write_lba,
        write_status_word,
        last_write_word0,
        last_write_word4,
        last_write_word8,
        last_write_word28
    };

    assign debug_probe =
        debug_probe_select ?
        ((debug_lba_select == 3'd7) ? write_debug_probe :
         (debug_lba_select == 3'd2) ? efs_direct_debug_probe :
                                      diag_debug_probe) :
        normal_debug_probe;
endmodule

`default_nettype wire
