// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - resolved-RGB DDR frame capture and 65 MHz HDMI scanout.
//
// The LG1 core deliberately remains in the 50 MHz CYAN clock domain.  This
// module captures its final resolved RGB stream into three private DDR frame
// banks and repeats the newest complete bank using standard 1024x768 timing.
// No palette, overlay, or drawing behavior is reimplemented here.

`default_nettype none

module kcu105_hdmi_frame_resampler #(
    parameter integer APP_ADDR_WIDTH = 28,
    parameter integer APP_DATA_WIDTH = 512,
    parameter integer APP_MASK_WIDTH = 64,
    parameter [25:0] FRAME_BASE_DW_ADDR = 26'h3e60000,
    parameter [25:0] FRAME_BANK_DW_STRIDE = 26'h0060000,
    parameter integer H_ACTIVE = 1024,
    parameter integer H_FP = 24,
    parameter integer H_SYNC = 136,
    parameter integer H_BP = 160,
    parameter integer V_ACTIVE = 768,
    parameter integer V_FP = 3,
    parameter integer V_SYNC = 6,
    parameter integer V_BP = 29,
    parameter integer SCAN_URGENT_LOW_WATER = 20,
    parameter integer SCAN_URGENT_HIGH_WATER = 48,
    parameter integer WRITE_URGENT_LOW_WATER = 16,
    parameter integer WRITE_URGENT_HIGH_WATER = 32
) (
    input  wire                       src_clk,
    input  wire                       src_rst_n,
    input  wire                       src_valid,
    input  wire                       src_frame_start,
    input  wire [9:0]                 src_x,
    input  wire [9:0]                 src_y,
    input  wire [7:0]                 src_raw_index,
    input  wire [4:0]                 src_did,
    input  wire [15:0]                src_xmap_mode,
    input  wire [1:0]                 src_palette_bank,
    input  wire [2:0]                 src_resolver_class,
    input  wire [7:0]                 src_display_index,
    input  wire [15:0]                src_fetch_low_signature,
    input  wire [15:0]                src_fetch_high_signature,
    input  wire [7:0]                 src_fetch_signature_generation,
    input  wire                       src_fetch_signature_valid,
    input  wire [91:0]                src_resolver_telemetry,
    input  wire [91:0]                src_resolver_detail,
    input  wire                       src_resolver_frame_done,
    input  wire [7:0]                 src_r,
    input  wire [7:0]                 src_g,
    input  wire [7:0]                 src_b,
    input  wire [31:0]                src_underrun_count,
    input  wire [31:0]                src_protocol_error_count,

    input  wire                       video_clk,
    input  wire                       video_rst_n,
    output reg                        video_de,
    output reg                        video_hsync,
    output reg                        video_vsync,
    output reg  [7:0]                 video_r,
    output reg  [7:0]                 video_g,
    output reg  [7:0]                 video_b,

    input  wire                       mig_clk,
    input  wire                       mig_rst,
    input  wire                       init_calib_complete,
    output wire [APP_ADDR_WIDTH-1:0]  app_addr,
    output wire [2:0]                 app_cmd,
    output wire                       app_en,
    input  wire                       app_rdy,
    output wire                       app_hi_pri,
    output wire [APP_DATA_WIDTH-1:0]  app_wdf_data,
    output wire                       app_wdf_end,
    output wire [APP_MASK_WIDTH-1:0]  app_wdf_mask,
    output wire                       app_wdf_wren,
    input  wire                       app_wdf_rdy,
    input  wire [APP_DATA_WIDTH-1:0]  app_rd_data,
    input  wire                       app_rd_data_end,
    input  wire                       app_rd_data_valid,

    input  wire [1:0]                 capture_command,
    input  wire                       capture_command_toggle,
    input  wire [15:0]                capture_word_index,
    output reg                        capture_ack_toggle,
    output wire [127:0]               capture_info,
    output reg  [511:0]               capture_word_data,
    output reg                        capture_word_valid,

    input  wire                       status_request_toggle,
    output reg                        status_ack_toggle,
    output reg  [127:0]               status,
    output reg  [127:0]               signature_status,
    output reg  [127:0]               stage_signature_status,
    output reg  [127:0]               resolver_status,
    output reg  [127:0]               resolver_detail_status,
    output reg  [127:0]               visual_stage_status,
    output reg  [127:0]               scan_service_status,
    output reg  [127:0]               scan_latency_status
);
    localparam integer PIXELS_PER_WORD = 16;
    localparam integer WORDS_PER_LINE = H_ACTIVE / PIXELS_PER_WORD;
    localparam integer FRAME_WORDS = WORDS_PER_LINE * V_ACTIVE;
    localparam integer H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP;
    localparam integer V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP;
    localparam integer WRITE_FIFO_WIDTH = APP_DATA_WIDTH + 19;
    localparam [2:0] MAX_SCAN_READS = 3'd4;
    localparam [2:0] MIG_CMD_WRITE = 3'b000;
    localparam [2:0] MIG_CMD_READ = 3'b001;

    function [15:0] fold_payload_half;
        input [191:0] value;
        begin
            fold_payload_half = value[15:0] ^ value[31:16] ^
                value[47:32] ^ value[63:48] ^ value[79:64] ^
                value[95:80] ^ value[111:96] ^ value[127:112] ^
                value[143:128] ^ value[159:144] ^ value[175:160] ^
                value[191:176];
        end
    endfunction

    function [15:0] signature_step;
        input [15:0] current;
        input [191:0] value;
        input [15:0] index;
        begin
            signature_step = {current[14:0], current[15]} ^
                             fold_payload_half(value) ^ index;
        end
    endfunction

    function [15:0] index_signature_step;
        input [15:0] current;
        input [7:0] value;
        integer bit_i;
        reg [15:0] crc;
        begin
            // CRC-16/CCITT over the ordered byte stream. The earlier XOR fold
            // allowed a constant raw/display difference to cancel over an
            // even-sized frame, defeating this classifier.
            crc = current ^ {value, 8'h00};
            for (bit_i = 0; bit_i < 8; bit_i = bit_i + 1)
                crc = crc[15] ? ((crc << 1) ^ 16'h1021) : (crc << 1);
            index_signature_step = crc;
        end
    endfunction

    function [7:0] gray_to_binary8;
        input [7:0] gray;
        integer bit_i;
        begin
            gray_to_binary8[7] = gray[7];
            for (bit_i = 6; bit_i >= 0; bit_i = bit_i - 1)
                gray_to_binary8[bit_i] =
                    gray_to_binary8[bit_i + 1] ^ gray[bit_i];
        end
    endfunction

    // ------------------------------------------------------------------
    // LG1 serial-cadence resolved-RGB capture.  Sixteen RGB888 pixels occupy
    // the low 384 bits of each native DDR word.  Unused bits are deterministic.
    // ------------------------------------------------------------------
    reg [511:0] src_pack;
    reg [15:0]  src_pack_lane_mask;
    reg [15:0]  src_pack_word_index;
    reg         src_pack_word_valid;
    reg [1:0]   src_bank;
    reg         src_capture_enable;
    reg         src_frame_active;
    reg         src_frame_good;
    reg [9:0]   src_expected_x;
    reg [9:0]   src_expected_y;
    reg [2:0]   src_inflight;
    reg [31:0]  src_drop_count;
    reg [31:0]  src_word_count;
    reg [15:0]  src_low_signature;
    reg [15:0]  src_high_signature;
    reg [15:0]  src_low_signature_bank [0:2];
    reg [15:0]  src_high_signature_bank [0:2];
    reg [15:0]  src_raw_low_signature;
    reg [15:0]  src_raw_high_signature;
    reg [15:0]  src_raw_low_signature_bank [0:2];
    reg [15:0]  src_raw_high_signature_bank [0:2];
    reg [15:0]  src_did_signature;
    reg [15:0]  src_xmap_mode_signature;
    reg [15:0]  src_route_signature;
    reg [15:0]  src_display_index_signature;
    reg [63:0]  src_visual_stage_signatures_bank [0:2];
    reg [15:0]  src_fetch_low_signature_bank [0:2];
    reg [15:0]  src_fetch_high_signature_bank [0:2];
    reg [7:0]   src_fetch_signature_generation_bank [0:2];
    reg         src_fetch_signature_valid_bank [0:2];
    reg [91:0]  src_resolver_telemetry_bank [0:2];
    reg [91:0]  src_resolver_detail_bank [0:2];
    reg         src_resolver_valid_bank [0:2];
    reg [1:0]   src_resolver_pending_bank;
    reg         src_resolver_pending;
    reg [38:0]  src_status_hold;
    (* ASYNC_REG = "TRUE" *) reg [2:0] status_request_src_sync;
    reg         status_request_src_seen;
    reg         status_src_ack_toggle;

    reg [WRITE_FIFO_WIDTH-1:0] write_fifo_in;
    reg                        write_fifo_wr_en;
    wire                       write_fifo_full;
    wire [6:0]                 write_fifo_wr_count;
    wire [WRITE_FIFO_WIDTH-1:0] write_fifo_out;
    wire                       write_fifo_empty;
    wire                       write_fifo_valid;
    wire                       write_fifo_rd_en;
    reg                        write_urgent_src;

    wire [15:0] src_word_index = (src_y * WORDS_PER_LINE) +
                                 (src_x / PIXELS_PER_WORD);
    wire src_word_last = (src_x == H_ACTIVE - 1) &&
                         (src_y == V_ACTIVE - 1);
    reg [511:0] src_pack_next;
    reg [15:0] src_pack_lane_mask_next;
    integer src_lane;
    always @* begin
        if (!src_pack_word_valid || src_pack_word_index != src_word_index) begin
            src_pack_next = 512'd0;
            src_pack_lane_mask_next = 16'd0;
        end else begin
            src_pack_next = src_pack;
            src_pack_lane_mask_next = src_pack_lane_mask;
        end
        src_lane = src_x[3:0] * 24;
        src_pack_next[src_lane +: 24] = {src_r, src_g, src_b};
        // Preserve the pre-XMAP packed framebuffer byte alongside each
        // resolved RGB pixel.  The upper 128 bits are not consumed by HDMI;
        // capture tooling uses them to distinguish buffer-selection faults
        // from producer or palette faults at an exact physical coordinate.
        src_pack_next[384 + src_x[3:0] * 8 +: 8] = src_raw_index;
        src_pack_lane_mask_next[src_x[3:0]] = 1'b1;
    end

    wire src_pixel_in_sequence = src_frame_start ?
        ((src_x == 0) && (src_y == 0)) :
        (src_frame_active && (src_x == src_expected_x) &&
         (src_y == src_expected_y));
    wire src_word_complete = (src_pack_lane_mask_next == 16'hffff);
    wire src_frame_complete_word = src_word_last && src_pixel_in_sequence &&
        src_frame_good && src_word_complete &&
        (src_word_count == FRAME_WORDS - 1);
    wire [15:0] src_low_signature_next = signature_step(
        (src_word_index == 0) ? 16'd0 : src_low_signature,
        src_pack_next[191:0], src_word_index);
    wire [15:0] src_high_signature_next = signature_step(
        (src_word_index == 0) ? 16'd0 : src_high_signature,
        src_pack_next[383:192], src_word_index);
    wire [15:0] src_raw_low_signature_next = index_signature_step(
        src_frame_start ? 16'hffff : src_raw_low_signature,
        src_raw_index);
    wire [15:0] src_raw_high_signature_next = index_signature_step(
        src_frame_start ? 16'hffff : src_raw_high_signature,
        src_raw_index);
    wire [15:0] src_did_signature_next = index_signature_step(
        src_frame_start ? 16'hffff : src_did_signature,
        {3'b000, src_did});
    wire [15:0] src_xmap_mode_signature_next = index_signature_step(
        index_signature_step(
            src_frame_start ? 16'hffff : src_xmap_mode_signature,
            src_xmap_mode[15:8]),
        src_xmap_mode[7:0]);
    wire [15:0] src_route_signature_next = index_signature_step(
        src_frame_start ? 16'hffff : src_route_signature,
        {3'b000, src_resolver_class, src_palette_bank});
    wire [15:0] src_display_index_signature_next = index_signature_step(
        src_frame_start ? 16'hffff : src_display_index_signature,
        src_display_index);

    // Completion information is produced only after MIG accepts the last
    // word, so a partially queued frame can never become displayable.
    reg [1:0] completed_bank_mig;
    reg       completed_toggle_mig;
    reg       completed_valid_mig;
    reg [5:0] ownership_payload_mig;
    reg       ownership_event_toggle_mig;
    reg [2:0] ownership_guard_mask_mig;
    (* ASYNC_REG = "TRUE" *) reg [1:0] ownership_ack_sync_mig;
    (* ASYNC_REG = "TRUE" *) reg [5:0] ownership_payload_src_sync1;
    (* ASYNC_REG = "TRUE" *) reg [5:0] ownership_payload_src_sync2;
    (* ASYNC_REG = "TRUE" *) reg [2:0] ownership_event_src_sync;
    reg ownership_event_src_seen;
    // This is the protocol acknowledgement state, not merely a duplicate of
    // ownership_event_src_seen.  Keep the explicit source register so the
    // return CDC has a single auditable launch point after the source domain
    // has consumed the atomic ownership mailbox.
    (* DONT_TOUCH = "TRUE" *) reg ownership_ack_toggle_src;
    reg ownership_completed_toggle_src_seen;
    reg [2:0] ownership_reserved_mask_src_latched;

    reg [1:0] display_bank_video;
    reg       scan_restart_toggle_video;
    // The binary value tags returned pixels.  Its Gray encoding is the
    // durable restart generation: unlike the legacy one-bit toggle it cannot
    // alias when an old DDR response delays acceptance across two vblanks.
    reg [7:0] scan_epoch_video;
    wire [7:0] scan_epoch_gray_video =
        scan_epoch_video ^ (scan_epoch_video >> 1);
    reg       capture_pinned_mig;
    reg [1:0] capture_bank_mig;

    reg [1:0] next_src_bank;
    reg       next_src_bank_valid;
    integer bank_i;
    always @* begin
        next_src_bank = 2'd0;
        next_src_bank_valid = 1'b0;
        for (bank_i = 0; bank_i < 3; bank_i = bank_i + 1) begin
            if (!next_src_bank_valid &&
                !src_inflight[bank_i] &&
                !ownership_reserved_mask_src_latched[bank_i]) begin
                next_src_bank = bank_i[1:0];
                next_src_bank_valid = 1'b1;
            end
        end
    end

    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            src_pack <= 512'd0;
            src_pack_lane_mask <= 16'd0;
            src_pack_word_index <= 16'd0;
            src_pack_word_valid <= 1'b0;
            src_bank <= 2'd1;
            src_capture_enable <= 1'b0;
            src_frame_active <= 1'b0;
            src_frame_good <= 1'b0;
            src_expected_x <= 10'd0;
            src_expected_y <= 10'd0;
            src_inflight <= 3'b000;
            src_drop_count <= 32'd0;
            src_word_count <= 32'd0;
            write_urgent_src <= 1'b0;
            src_low_signature <= 16'd0;
            src_high_signature <= 16'd0;
            src_low_signature_bank[0] <= 16'd0;
            src_low_signature_bank[1] <= 16'd0;
            src_low_signature_bank[2] <= 16'd0;
            src_high_signature_bank[0] <= 16'd0;
            src_high_signature_bank[1] <= 16'd0;
            src_high_signature_bank[2] <= 16'd0;
            src_raw_low_signature <= 16'd0;
            src_raw_high_signature <= 16'd0;
            src_raw_low_signature_bank[0] <= 16'd0;
            src_raw_low_signature_bank[1] <= 16'd0;
            src_raw_low_signature_bank[2] <= 16'd0;
            src_raw_high_signature_bank[0] <= 16'd0;
            src_raw_high_signature_bank[1] <= 16'd0;
            src_raw_high_signature_bank[2] <= 16'd0;
            src_did_signature <= 16'd0;
            src_xmap_mode_signature <= 16'd0;
            src_route_signature <= 16'd0;
            src_display_index_signature <= 16'd0;
            src_visual_stage_signatures_bank[0] <= 64'd0;
            src_visual_stage_signatures_bank[1] <= 64'd0;
            src_visual_stage_signatures_bank[2] <= 64'd0;
            src_fetch_low_signature_bank[0] <= 16'd0;
            src_fetch_low_signature_bank[1] <= 16'd0;
            src_fetch_low_signature_bank[2] <= 16'd0;
            src_fetch_high_signature_bank[0] <= 16'd0;
            src_fetch_high_signature_bank[1] <= 16'd0;
            src_fetch_high_signature_bank[2] <= 16'd0;
            src_fetch_signature_generation_bank[0] <= 8'd0;
            src_fetch_signature_generation_bank[1] <= 8'd0;
            src_fetch_signature_generation_bank[2] <= 8'd0;
            src_fetch_signature_valid_bank[0] <= 1'b0;
            src_fetch_signature_valid_bank[1] <= 1'b0;
            src_fetch_signature_valid_bank[2] <= 1'b0;
            src_resolver_telemetry_bank[0] <= 92'd0;
            src_resolver_telemetry_bank[1] <= 92'd0;
            src_resolver_telemetry_bank[2] <= 92'd0;
            src_resolver_detail_bank[0] <= 92'd0;
            src_resolver_detail_bank[1] <= 92'd0;
            src_resolver_detail_bank[2] <= 92'd0;
            src_resolver_valid_bank[0] <= 1'b0;
            src_resolver_valid_bank[1] <= 1'b0;
            src_resolver_valid_bank[2] <= 1'b0;
            src_resolver_pending_bank <= 2'd0;
            src_resolver_pending <= 1'b0;
            src_status_hold <= 39'd0;
            status_request_src_sync <= 3'b000;
            status_request_src_seen <= 1'b0;
            status_src_ack_toggle <= 1'b0;
            write_fifo_in <= {WRITE_FIFO_WIDTH{1'b0}};
            write_fifo_wr_en <= 1'b0;
            ownership_payload_src_sync1 <= {1'b0, 2'd0, 3'b001};
            ownership_payload_src_sync2 <= {1'b0, 2'd0, 3'b001};
            ownership_event_src_sync <= 3'b000;
            ownership_event_src_seen <= 1'b0;
            ownership_ack_toggle_src <= 1'b0;
            ownership_completed_toggle_src_seen <= 1'b0;
            ownership_reserved_mask_src_latched <= 3'b001;
        end else begin
            // Capture has a physical source deadline just as scanout does.
            // Derive urgency entirely in the FIFO write clock domain, where
            // the binary occupancy is coherent, then synchronize only the
            // resulting hysteretic level into the MIG domain below.
            if (write_fifo_wr_count >= WRITE_URGENT_HIGH_WATER)
                write_urgent_src <= 1'b1;
            else if (write_fifo_wr_count <= WRITE_URGENT_LOW_WATER)
                write_urgent_src <= 1'b0;

            ownership_payload_src_sync1 <= ownership_payload_mig;
            ownership_payload_src_sync2 <= ownership_payload_src_sync1;
            ownership_event_src_sync <= {
                ownership_event_src_sync[1:0],
                ownership_event_toggle_mig
            };
            write_fifo_wr_en <= 1'b0;
            status_request_src_sync <=
                {status_request_src_sync[1:0], status_request_toggle};

            if (status_request_src_sync[2] != status_request_src_seen) begin
                status_request_src_seen <= status_request_src_sync[2];
                src_status_hold <= {
                    src_bank, src_capture_enable, write_fifo_full,
                    src_drop_count[15:0], src_word_count[15:0], src_inflight
                };
                status_src_ack_toggle <= status_request_src_sync[2];
            end

            // Display, completion, and host-capture ownership are one atomic
            // MIG-owned mailbox.  Consuming separate asynchronous bank buses
            // can transiently free a bank which another domain just acquired.
            if (ownership_event_src_sync[2] !=
                ownership_event_src_seen) begin
                ownership_event_src_seen <= ownership_event_src_sync[2];
                ownership_ack_toggle_src <= ownership_event_src_sync[2];
                ownership_reserved_mask_src_latched <=
                    ownership_payload_src_sync2[2:0];
                if (ownership_payload_src_sync2[5] !=
                    ownership_completed_toggle_src_seen) begin
                    ownership_completed_toggle_src_seen <=
                        ownership_payload_src_sync2[5];
                    if (ownership_payload_src_sync2[4:3] < 3)
                        src_inflight[ownership_payload_src_sync2[4:3]] <=
                            1'b0;
                end
            end

            if (src_frame_start) begin
                // A resolver record belongs only to the immediately
                // preceding complete source frame.  Never let a delayed
                // pulse attach stale detail after the next frame starts.
                src_resolver_pending <= 1'b0;
                src_word_count <= 32'd0;
                src_low_signature <= 16'd0;
                src_high_signature <= 16'd0;
                src_raw_low_signature <= 16'hffff;
                src_raw_high_signature <= 16'hffff;
                src_did_signature <= 16'hffff;
                src_xmap_mode_signature <= 16'hffff;
                src_route_signature <= 16'hffff;
                src_display_index_signature <= 16'hffff;
                src_frame_active <= 1'b1;
                src_frame_good <= (src_x == 0) && (src_y == 0);
                src_expected_x <= 10'd1;
                src_expected_y <= 10'd0;
                src_pack_word_valid <= 1'b0;
                src_pack_lane_mask <= 16'd0;
                if (next_src_bank_valid) begin
                    src_bank <= next_src_bank;
                    src_capture_enable <= 1'b1;
                    src_resolver_valid_bank[next_src_bank] <= 1'b0;
                end else begin
                    src_capture_enable <= 1'b0;
                    src_drop_count <= src_drop_count + 32'd1;
                end
            end

            if (src_valid &&
                (src_capture_enable ||
                 (src_frame_start && next_src_bank_valid)) &&
                (src_x < H_ACTIVE) && (src_y < V_ACTIVE)) begin
                src_pack <= src_pack_next;
                src_pack_lane_mask <= src_pack_lane_mask_next;
                src_pack_word_index <= src_word_index;
                src_pack_word_valid <= 1'b1;
                if (!src_x[3]) begin
                    src_raw_low_signature <= src_raw_low_signature_next;
                end else begin
                    src_raw_high_signature <= src_raw_high_signature_next;
                end
                src_did_signature <= src_did_signature_next;
                src_xmap_mode_signature <= src_xmap_mode_signature_next;
                src_route_signature <= src_route_signature_next;
                src_display_index_signature <=
                    src_display_index_signature_next;
                if (!src_pixel_in_sequence)
                    src_frame_good <= 1'b0;
                if (src_x == H_ACTIVE - 1) begin
                    src_expected_x <= 10'd0;
                    src_expected_y <= src_y + 10'd1;
                end else begin
                    src_expected_x <= src_x + 10'd1;
                    src_expected_y <= src_y;
                end
                if (src_x[3:0] == 4'hf) begin
                    src_pack_word_valid <= 1'b0;
                    src_pack_lane_mask <= 16'd0;
                    if (!write_fifo_full && src_pixel_in_sequence &&
                        src_frame_good && src_word_complete) begin
                        write_fifo_in <= {src_frame_complete_word, src_bank,
                                          src_word_index, src_pack_next};
                        write_fifo_wr_en <= 1'b1;
                        src_word_count <= src_word_count + 32'd1;
                        src_low_signature <= src_low_signature_next;
                        src_high_signature <= src_high_signature_next;
                        if (src_frame_complete_word) begin
                            src_low_signature_bank[src_bank] <=
                                src_low_signature_next;
                            src_high_signature_bank[src_bank] <=
                                src_high_signature_next;
                            src_raw_low_signature_bank[src_bank] <=
                                src_raw_low_signature;
                            src_raw_high_signature_bank[src_bank] <=
                                src_raw_high_signature_next;
                            src_visual_stage_signatures_bank[src_bank] <= {
                                src_did_signature_next,
                                src_xmap_mode_signature_next,
                                src_route_signature_next,
                                src_display_index_signature_next
                            };
                            src_fetch_low_signature_bank[src_bank] <=
                                src_fetch_low_signature;
                            src_fetch_high_signature_bank[src_bank] <=
                                src_fetch_high_signature;
                            src_fetch_signature_generation_bank[src_bank] <=
                                src_fetch_signature_generation;
                            src_fetch_signature_valid_bank[src_bank] <=
                                src_fetch_signature_valid;
                            src_resolver_pending_bank <= src_bank;
                            src_resolver_pending <= 1'b1;
                            src_inflight[src_bank] <= 1'b1;
                            src_frame_active <= 1'b0;
                        end else if (src_word_last) begin
                            src_capture_enable <= 1'b0;
                            src_frame_good <= 1'b0;
                            src_frame_active <= 1'b0;
                            src_drop_count <= src_drop_count + 32'd1;
                        end
                    end else begin
                        src_capture_enable <= 1'b0;
                        src_frame_good <= 1'b0;
                        if (src_word_last)
                            src_frame_active <= 1'b0;
                        src_drop_count <= src_drop_count + 32'd1;
                    end
                end
            end

            if (src_resolver_frame_done && src_resolver_pending &&
                !src_frame_start) begin
                src_resolver_telemetry_bank[src_resolver_pending_bank] <=
                    src_resolver_telemetry;
                src_resolver_detail_bank[src_resolver_pending_bank] <=
                    src_resolver_detail;
                src_resolver_valid_bank[src_resolver_pending_bank] <= 1'b1;
                src_resolver_pending <= 1'b0;
            end
        end
    end

    gmii_async_fifo #(
        .WIDTH(WRITE_FIFO_WIDTH), .ADDR_W(6)
    ) u_write_fifo (
        .wr_clk(src_clk), .wr_rst_n(src_rst_n),
        .wr_data(write_fifo_in), .wr_en(write_fifo_wr_en),
        .wr_full(write_fifo_full), .wr_count(write_fifo_wr_count),
        .rd_clk(mig_clk), .rd_rst_n(!mig_rst),
        .rd_data(write_fifo_out), .rd_en(write_fifo_rd_en),
        .rd_empty(write_fifo_empty), .rd_valid(write_fifo_valid)
    );

    // ------------------------------------------------------------------
    // Standard 1024x768 scan timing and response FIFO in the 65 MHz domain.
    // A new DDR read pass begins at the start of vertical blank, leaving the
    // complete blanking interval to prime the next visible frame.
    // ------------------------------------------------------------------
    reg [10:0] h_ctr;
    reg [9:0]  v_ctr;
    reg [511:0] video_word;
    reg         video_word_valid;
    reg         scan_valid_video;
    reg [15:0]  video_expected_word_index;
    reg [31:0]  video_underrun_count;
    reg [19:0]  video_status_hold;
    reg [71:0]  video_service_hold;
    reg [15:0]  frame_miss_count_video;
    reg         frame_first_miss_seen_video;
    reg [15:0]  frame_first_miss_index_video;
    reg [7:0]   frame_first_head_epoch_video;
    reg [15:0]  frame_first_head_index_video;
    reg [6:0]   frame_first_flags_video;
    reg         last_service_valid_video;
    reg [7:0]   last_service_epoch_video;
    reg [15:0]  last_frame_miss_count_video;
    reg [15:0]  last_first_miss_index_video;
    reg [7:0]   last_first_head_epoch_video;
    reg [15:0]  last_first_head_index_video;
    reg [6:0]   last_first_flags_video;
    reg [2:0]   status_request_video_sync;
    reg         status_request_video_seen;
    reg         status_video_ack_toggle;

    wire [511:0] read_fifo_data;
    wire         read_fifo_empty;
    wire         read_fifo_valid;
    reg          read_fifo_rd_en;
    wire         read_fifo_full;
    wire [6:0]   read_fifo_wr_count;
    wire         read_fifo_wr_en;
    reg [511:0]  scan_response_data_stage_mig;
    reg [15:0]   scan_response_index_stage_mig;
    reg [7:0]    scan_response_epoch_stage_mig;
    reg          scan_response_stage_valid_mig;
    reg [511:0]  scan_response_data_push_mig;
    reg [15:0]   scan_response_index_push_mig;
    reg [7:0]    scan_response_epoch_push_mig;
    reg          scan_response_push_valid_mig;
    reg [15:0]   scan_response_index_mig;
    reg [7:0]    scan_epoch_mig;

    // MIG's UI reset is generated inside the DDR block, far from the HDMI
    // read FIFO BRAMs.  Feeding it directly into the XPM FIFO reset gating
    // put that board-wide net on every BRAM write-enable path.  Assert the
    // local copy immediately and release it synchronously after two MIG
    // clocks; this preserves the reset boundary while making the functional
    // reset fanout local to the FIFO.
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
    reg [1:0] read_fifo_mig_reset_pipe;
    always @(posedge mig_clk or posedge mig_rst) begin
        if (mig_rst)
            read_fifo_mig_reset_pipe <= 2'b11;
        else
            read_fifo_mig_reset_pipe <=
                {read_fifo_mig_reset_pipe[0], 1'b0};
    end
    wire read_fifo_mig_rst_n = !read_fifo_mig_reset_pipe[1];

    (* ASYNC_REG = "TRUE" *) reg [1:0] completed_bank_video_sync1;
    (* ASYNC_REG = "TRUE" *) reg [1:0] completed_bank_video_sync2;
    (* ASYNC_REG = "TRUE" *) reg [2:0] completed_toggle_video_sync;
    reg completed_toggle_video_seen;
    reg [1:0] completed_bank_video_latched;
    reg completed_bank_video_latched_valid;
    reg [1:0] completed_seed_bank_video;
    reg completed_seed_toggle_video;
    reg completed_seed_sample_valid_video;
    (* ASYNC_REG = "TRUE" *) reg [1:0] completed_valid_video_sync;

    wire timing_visible = (h_ctr < H_ACTIVE) && (v_ctr < V_ACTIVE);
    wire timing_group_start = timing_visible && (h_ctr[3:0] == 4'h0);
    wire [7:0] read_fifo_epoch = read_fifo_data[407:400];
    wire [15:0] read_fifo_word_index = read_fifo_data[399:384];
    wire [7:0] read_fifo_epoch_delta =
        read_fifo_epoch - scan_epoch_video;
    wire read_fifo_exact = read_fifo_valid && !read_fifo_rd_en &&
                           (read_fifo_epoch == scan_epoch_video) &&
                           (read_fifo_word_index ==
                            video_expected_word_index);
    // Epochs within 127 frames ahead are future data and must be retained;
    // epochs behind the active scan have the high modular-delta bit set and
    // are stale.  A 64-word FIFO cannot retain data for 128 frames, so the
    // half-range comparison is unambiguous here, including at 8-bit wrap.
    wire read_fifo_obsolete = read_fifo_valid &&
                              (read_fifo_epoch_delta[7] ||
                               ((read_fifo_epoch_delta == 0) &&
                                (read_fifo_word_index <
                                 video_expected_word_index)));
    wire [511:0] selected_word = timing_group_start ? read_fifo_data : video_word;
    wire selected_valid = timing_group_start ? read_fifo_exact :
                                               video_word_valid;
    wire [8:0] video_lane = h_ctr[3:0] * 24;
    wire [23:0] selected_rgb = selected_word[video_lane +: 24];

    always @(posedge video_clk or negedge video_rst_n) begin
        if (!video_rst_n) begin
            h_ctr <= 11'd0;
            v_ctr <= V_ACTIVE[9:0];
            video_word <= 512'd0;
            video_word_valid <= 1'b0;
            read_fifo_rd_en <= 1'b0;
            scan_restart_toggle_video <= 1'b0;
            scan_epoch_video <= 8'd0;
            scan_valid_video <= 1'b0;
            video_expected_word_index <= 16'd0;
            display_bank_video <= 2'd0;
            completed_bank_video_sync1 <= 2'd0;
            completed_bank_video_sync2 <= 2'd0;
            completed_toggle_video_sync <= 3'b000;
            completed_toggle_video_seen <= 1'b0;
            completed_bank_video_latched <= 2'd0;
            completed_bank_video_latched_valid <= 1'b0;
            completed_seed_bank_video <= 2'd0;
            completed_seed_toggle_video <= 1'b0;
            completed_seed_sample_valid_video <= 1'b0;
            completed_valid_video_sync <= 2'b00;
            video_underrun_count <= 32'd0;
            video_status_hold <= 20'd0;
            video_service_hold <= 72'd0;
            frame_miss_count_video <= 16'd0;
            frame_first_miss_seen_video <= 1'b0;
            frame_first_miss_index_video <= 16'd0;
            frame_first_head_epoch_video <= 8'd0;
            frame_first_head_index_video <= 16'd0;
            frame_first_flags_video <= 7'd0;
            last_service_valid_video <= 1'b0;
            last_service_epoch_video <= 8'd0;
            last_frame_miss_count_video <= 16'd0;
            last_first_miss_index_video <= 16'd0;
            last_first_head_epoch_video <= 8'd0;
            last_first_head_index_video <= 16'd0;
            last_first_flags_video <= 7'd0;
            status_request_video_sync <= 3'b000;
            status_request_video_seen <= 1'b0;
            status_video_ack_toggle <= 1'b0;
            video_de <= 1'b0;
            video_hsync <= 1'b1;
            video_vsync <= 1'b1;
            video_r <= 8'd0;
            video_g <= 8'd0;
            video_b <= 8'd0;
        end else begin
            completed_bank_video_sync1 <= completed_bank_mig;
            completed_bank_video_sync2 <= completed_bank_video_sync1;
            completed_toggle_video_sync <= {
                completed_toggle_video_sync[1:0], completed_toggle_mig
            };
            completed_valid_video_sync <=
                {completed_valid_video_sync[0], completed_valid_mig};
            read_fifo_rd_en <= 1'b0;
            status_request_video_sync <=
                {status_request_video_sync[1:0], status_request_toggle};

            // completed_bank_mig is a multi-bit asynchronous payload.  The
            // completion toggle arrives one stage later and qualifies the
            // settled payload.  After a video-only reset there may be no
            // toggle mismatch (retained producer parity zero), so seed only
            // from two consecutive matching samples of the fully
            // synchronized toggle/payload tuple.  Never inspect stages zero
            // or one here: they are the metastability containment chain and
            // may only drive the next synchronizer stage.
            if (!completed_bank_video_latched_valid) begin
                if (completed_valid_video_sync[1] &&
                    (completed_bank_video_sync2 < 3)) begin
                    if (completed_seed_sample_valid_video &&
                        (completed_seed_bank_video ==
                         completed_bank_video_sync2) &&
                        (completed_seed_toggle_video ==
                         completed_toggle_video_sync[2])) begin
                        completed_toggle_video_seen <=
                            completed_toggle_video_sync[2];
                        completed_bank_video_latched <=
                            completed_bank_video_sync2;
                        completed_bank_video_latched_valid <= 1'b1;
                        completed_seed_sample_valid_video <= 1'b0;
                    end else begin
                        completed_seed_bank_video <=
                            completed_bank_video_sync2;
                        completed_seed_toggle_video <=
                            completed_toggle_video_sync[2];
                        completed_seed_sample_valid_video <= 1'b1;
                    end
                end else begin
                    completed_seed_sample_valid_video <= 1'b0;
                end
            end else if ((completed_toggle_video_sync[2] !=
                          completed_toggle_video_seen) &&
                         (completed_bank_video_sync2 < 3)) begin
                completed_toggle_video_seen <=
                    completed_toggle_video_sync[2];
                completed_bank_video_latched <=
                    completed_bank_video_sync2;
            end

            if (status_request_video_sync[2] != status_request_video_seen) begin
                status_request_video_seen <= status_request_video_sync[2];
                video_status_hold <= {
                    display_bank_video, read_fifo_valid,
                    video_underrun_count[15:0], read_fifo_empty
                };
                video_service_hold <= {
                    last_service_valid_video,
                    last_service_epoch_video,
                    last_frame_miss_count_video,
                    last_first_miss_index_video,
                    last_first_head_epoch_video,
                    last_first_head_index_video,
                    last_first_flags_video
                };
                status_video_ack_toggle <= status_request_video_sync[2];
            end

            video_de <= timing_visible;
            video_hsync <= !((h_ctr >= H_ACTIVE + H_FP) &&
                             (h_ctr < H_ACTIVE + H_FP + H_SYNC));
            video_vsync <= !((v_ctr >= V_ACTIVE + V_FP) &&
                             (v_ctr < V_ACTIVE + V_FP + V_SYNC));
            if (timing_visible && selected_valid && scan_valid_video) begin
                video_r <= selected_rgb[23:16];
                video_g <= selected_rgb[15:8];
                video_b <= selected_rgb[7:0];
            end else begin
                video_r <= 8'd0;
                video_g <= 8'd0;
                video_b <= 8'd0;
            end

            // Once display timing has passed a tagged word, consume it as
            // soon as possible.  This is especially important in vertical
            // blank, where an old-epoch tail must drain before word zero of
            // the new scan is presented.  read_fifo_rd_en is a one-cycle
            // pulse; do not reassert it on the edge which performs the pop.
            if (read_fifo_obsolete && !read_fifo_rd_en)
                read_fifo_rd_en <= 1'b1;

            if (timing_group_start) begin
                video_expected_word_index <=
                    video_expected_word_index + 16'd1;
                if (read_fifo_exact) begin
                    video_word <= read_fifo_data;
                    video_word_valid <= 1'b1;
                    read_fifo_rd_en <= 1'b1;
                end else begin
                    video_word_valid <= 1'b0;
                    video_underrun_count <= video_underrun_count + 32'd1;
                    if (scan_valid_video) begin
                        if (frame_miss_count_video != 16'hffff)
                            frame_miss_count_video <=
                                frame_miss_count_video + 16'd1;
                        if (!frame_first_miss_seen_video) begin
                            frame_first_miss_seen_video <= 1'b1;
                            frame_first_miss_index_video <=
                                video_expected_word_index;
                            frame_first_head_epoch_video <= read_fifo_epoch;
                            frame_first_head_index_video <=
                                read_fifo_word_index;
                            frame_first_flags_video <= {
                                1'b1, read_fifo_valid, read_fifo_exact,
                                read_fifo_obsolete,
                                read_fifo_valid && !read_fifo_obsolete &&
                                    !read_fifo_exact,
                                read_fifo_empty, scan_valid_video
                            };
                        end
                    end
                end
            end

            if ((h_ctr == 0) && (v_ctr == V_ACTIVE)) begin
                last_service_valid_video <= scan_valid_video;
                last_service_epoch_video <= scan_epoch_video;
                last_frame_miss_count_video <= frame_miss_count_video;
                last_first_miss_index_video <=
                    frame_first_miss_index_video;
                last_first_head_epoch_video <=
                    frame_first_head_epoch_video;
                last_first_head_index_video <=
                    frame_first_head_index_video;
                last_first_flags_video <= frame_first_flags_video;
                frame_miss_count_video <= 16'd0;
                frame_first_miss_seen_video <= 1'b0;
                frame_first_miss_index_video <= 16'd0;
                frame_first_head_epoch_video <= 8'd0;
                frame_first_head_index_video <= 16'd0;
                frame_first_flags_video <= 7'd0;
                if (completed_valid_video_sync[1] &&
                    completed_bank_video_latched_valid) begin
                    display_bank_video <= completed_bank_video_latched;
                    scan_valid_video <= 1'b1;
                end
                scan_restart_toggle_video <= ~scan_restart_toggle_video;
                scan_epoch_video <= scan_epoch_video + 8'd1;
                video_expected_word_index <= 16'd0;
                video_word_valid <= 1'b0;
            end

            if (h_ctr == H_TOTAL - 1) begin
                h_ctr <= 11'd0;
                if (v_ctr == V_TOTAL - 1)
                    v_ctr <= 10'd0;
                else
                    v_ctr <= v_ctr + 10'd1;
            end else begin
                h_ctr <= h_ctr + 11'd1;
            end
        end
    end

    // Register native responses twice before they drive the XPM FIFO.  The
    // first stage terminates the synchronized epoch/restart comparison.  The
    // second is an elastic, FIFO-local admission boundary: the first exact
    // placed implementation that closed the 100 MHz CPU still spent 2.741 ns
    // routing the first-stage valid bit into the wide BRAM write-enable tree.
    // Both stages retain the response's original epoch/index; the low 384
    // payload bits remain the resolved RGB pixels.
    wire [511:0] read_fifo_write_data = {
        scan_response_data_push_mig[511:408],
        scan_response_epoch_push_mig,
        scan_response_index_push_mig,
        scan_response_data_push_mig[383:0]
    };

    gmii_async_fifo #(
        .WIDTH(APP_DATA_WIDTH), .ADDR_W(6)
    ) u_read_fifo (
        .wr_clk(mig_clk), .wr_rst_n(read_fifo_mig_rst_n),
        .wr_data(read_fifo_write_data), .wr_en(read_fifo_wr_en),
        .wr_full(read_fifo_full), .wr_count(read_fifo_wr_count),
        .rd_clk(video_clk), .rd_rst_n(video_rst_n),
        .rd_data(read_fifo_data), .rd_en(read_fifo_rd_en),
        .rd_empty(read_fifo_empty), .rd_valid(read_fifo_valid)
    );

    // ------------------------------------------------------------------
    // MIG-domain read/write engine.  It is one client of the board arbiter
    // and keeps at most one native transaction outstanding.
    // ------------------------------------------------------------------
    localparam [2:0] ST_IDLE = 3'd0;
    localparam [2:0] ST_READ_CMD = 3'd1;
    localparam [2:0] ST_READ_WAIT = 3'd2;
    localparam [2:0] ST_WRITE = 3'd3;
    reg [2:0] mig_state;
    reg       op_is_write;
    reg       op_is_capture;
    reg       op_hi_pri;
    reg [1:0] op_bank;
    reg [15:0] op_index;
    reg       op_last;
    reg [511:0] op_data;
    reg       write_cmd_accepted;
    reg       write_data_accepted;
    reg       last_was_read;
    reg [15:0] scan_index_mig;
    reg [2:0] scan_read_outstanding;
    reg       scan_issue_active_mig;
    reg       scan_active_mig;
    reg       scan_urgent_mig;
    reg [1:0] scan_bank_mig;
    reg       scan_valid_mig;
    reg [31:0] mig_read_count;
    reg [31:0] mig_write_count;
    reg [15:0] completed_frame_count_mig;
    reg [15:0] service_cycle_mig;
    reg [15:0] service_timestamp_0_mig;
    reg [15:0] service_timestamp_1_mig;
    reg [15:0] service_timestamp_2_mig;
    reg [15:0] service_timestamp_3_mig;
    reg [15:0] service_command_wait_run_mig;
    reg [15:0] service_command_count_mig;
    reg [15:0] service_response_count_mig;
    reg [15:0] service_max_command_wait_mig;
    reg [15:0] service_max_response_latency_mig;
    reg [6:0]  service_min_resident_mig;
    reg [2:0]  service_max_outstanding_mig;
    reg [7:0]  service_response_fast_mig;
    reg [7:0]  service_response_mid_mig;
    reg [7:0]  service_response_slow_mig;
    reg        last_service_valid_mig;
    reg [7:0]  last_service_epoch_mig;
    reg [15:0] last_service_command_count_mig;
    reg [15:0] last_service_response_count_mig;
    reg [15:0] last_service_max_command_wait_mig;
    reg [15:0] last_service_max_response_latency_mig;
    reg [6:0]  last_service_min_resident_mig;
    reg [2:0]  last_service_max_outstanding_mig;
    reg [7:0]  last_service_response_fast_mig;
    reg [7:0]  last_service_response_mid_mig;
    reg [7:0]  last_service_response_slow_mig;
    reg [15:0] capture_frame_mig;
    reg [15:0] mig_low_signature;
    reg [15:0] mig_high_signature;
    reg [15:0] completed_src_low_signature_mig;
    reg [15:0] completed_src_high_signature_mig;
    reg [15:0] completed_mig_low_signature;
    reg [15:0] completed_mig_high_signature;
    reg [15:0] completed_raw_low_signature_mig;
    reg [15:0] completed_raw_high_signature_mig;
    reg [63:0] completed_visual_stage_signatures_mig;
    reg [15:0] completed_fetch_low_signature_mig;
    reg [15:0] completed_fetch_high_signature_mig;
    reg [7:0]  completed_fetch_signature_generation_mig;
    reg        completed_fetch_signature_valid_mig;
    reg [15:0] capture_src_low_signature_mig;
    reg [15:0] capture_src_high_signature_mig;
    reg [15:0] capture_mig_low_signature;
    reg [15:0] capture_mig_high_signature;
    reg [15:0] capture_raw_low_signature_mig;
    reg [15:0] capture_raw_high_signature_mig;
    reg [63:0] capture_visual_stage_signatures_mig;
    reg [15:0] capture_fetch_low_signature_mig;
    reg [15:0] capture_fetch_high_signature_mig;
    reg [7:0]  capture_fetch_signature_generation_mig;
    reg        capture_fetch_signature_valid_mig;
    reg [91:0] completed_resolver_telemetry_mig;
    reg        completed_resolver_valid_mig;
    reg [91:0] capture_resolver_telemetry_mig;
    reg        capture_resolver_valid_mig;
    reg [91:0] completed_resolver_detail_mig;
    reg [91:0] capture_resolver_detail_mig;
    reg        payload_signature_valid;
    reg [7:0]  payload_signature_generation;
    reg [7:0]  capture_signature_generation_mig;
    (* ASYNC_REG = "TRUE" *) reg [15:0] src_low_signature_sync1 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [15:0] src_low_signature_sync2 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [15:0] src_high_signature_sync1 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [15:0] src_high_signature_sync2 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [15:0] src_raw_low_signature_sync1 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [15:0] src_raw_low_signature_sync2 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [15:0] src_raw_high_signature_sync1 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [15:0] src_raw_high_signature_sync2 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [63:0]
        src_visual_stage_signatures_sync1 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [63:0]
        src_visual_stage_signatures_sync2 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [15:0] src_fetch_low_signature_sync1 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [15:0] src_fetch_low_signature_sync2 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [15:0] src_fetch_high_signature_sync1 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [15:0] src_fetch_high_signature_sync2 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [7:0]
        src_fetch_signature_generation_sync1 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [7:0]
        src_fetch_signature_generation_sync2 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [2:0] src_fetch_signature_valid_sync1;
    (* ASYNC_REG = "TRUE" *) reg [2:0] src_fetch_signature_valid_sync2;
    (* ASYNC_REG = "TRUE" *) reg [91:0]
        src_resolver_telemetry_sync1 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [91:0]
        src_resolver_telemetry_sync2 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [91:0]
        src_resolver_detail_sync1 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [91:0]
        src_resolver_detail_sync2 [0:2];
    (* ASYNC_REG = "TRUE" *) reg [2:0] src_resolver_valid_sync1;
    (* ASYNC_REG = "TRUE" *) reg [2:0] src_resolver_valid_sync2;
    (* ASYNC_REG = "TRUE" *) reg [31:0] src_underrun_count_sync1;
    (* ASYNC_REG = "TRUE" *) reg [31:0] src_underrun_count_sync2;
    (* ASYNC_REG = "TRUE" *) reg [31:0] src_protocol_error_count_sync1;
    (* ASYNC_REG = "TRUE" *) reg [31:0] src_protocol_error_count_sync2;
    reg        capture_cache_valid_mig;
    reg [15:0] capture_cache_index_mig;
    reg        capture_pending_mig;
    reg [15:0] capture_pending_index_mig;
    reg        capture_pending_toggle_mig;
    reg        capture_error_mig;
    (* ASYNC_REG = "TRUE" *) reg [2:0] capture_command_sync_mig;
    reg        capture_command_seen_mig;
    reg [1:0]  capture_command_mig1;
    reg [1:0]  capture_command_mig2;
    reg [15:0] capture_word_index_mig1;
    reg [15:0] capture_word_index_mig2;
    (* ASYNC_REG = "TRUE" *) reg [2:0] status_request_sync_mig;
    (* ASYNC_REG = "TRUE" *) reg [1:0] status_src_ack_sync_mig;
    (* ASYNC_REG = "TRUE" *) reg [1:0] status_video_ack_sync_mig;
    reg status_request_seen_mig;
    reg status_snapshot_pending_mig;
    (* ASYNC_REG = "TRUE" *) reg [2:0] scan_restart_sync_mig;
    (* ASYNC_REG = "TRUE" *) reg [7:0] scan_epoch_gray_sync_mig1;
    (* ASYNC_REG = "TRUE" *) reg [7:0] scan_epoch_gray_sync_mig2;
    (* ASYNC_REG = "TRUE" *) reg [7:0] scan_epoch_gray_sync_mig3;
    (* ASYNC_REG = "TRUE" *) reg [1:0] scan_bank_sync_mig1;
    (* ASYNC_REG = "TRUE" *) reg [1:0] scan_bank_sync_mig2;
    (* ASYNC_REG = "TRUE" *) reg [1:0] scan_valid_sync_mig;
    (* ASYNC_REG = "TRUE" *) reg [2:0] video_alive_sync_mig;
    (* ASYNC_REG = "TRUE" *) reg [1:0] write_urgent_sync_mig;
    reg scan_restart_seen_mig;
    reg [7:0] scan_epoch_gray_seen_mig;
    reg op_scan_discard_mig;
    reg read_fifo_count_lag_credit_mig;

    wire [2:0] completed_owner_mask_mig = completed_valid_mig &&
        (completed_bank_mig < 3) ?
        (3'b001 << completed_bank_mig) : 3'b001;
    wire [2:0] scan_owner_mask_mig = scan_active_mig &&
        (scan_bank_mig < 3) ? (3'b001 << scan_bank_mig) : 3'b000;
    wire [2:0] capture_owner_mask_mig = capture_pinned_mig &&
        (capture_bank_mig < 3) ? (3'b001 << capture_bank_mig) : 3'b000;
    wire [2:0] ownership_reserved_mask_mig =
        completed_owner_mask_mig | scan_owner_mask_mig |
        capture_owner_mask_mig | ownership_guard_mask_mig;
    wire [5:0] ownership_state_mig = {
        completed_toggle_mig, completed_bank_mig,
        ownership_reserved_mask_mig
    };

    wire [1:0] signature_bank_mig = capture_pinned_mig ?
        capture_bank_mig : completed_bank_mig;
    wire [15:0] signature_frame_mig = capture_pinned_mig ?
        capture_frame_mig : completed_frame_count_mig;
    wire [15:0] signature_src_low_mig = capture_pinned_mig ?
        capture_src_low_signature_mig : completed_src_low_signature_mig;
    wire [15:0] signature_src_high_mig = capture_pinned_mig ?
        capture_src_high_signature_mig : completed_src_high_signature_mig;
    wire [15:0] signature_mig_low_mig = capture_pinned_mig ?
        capture_mig_low_signature : completed_mig_low_signature;
    wire [15:0] signature_mig_high_mig = capture_pinned_mig ?
        capture_mig_high_signature : completed_mig_high_signature;
    wire [7:0] signature_generation_mig = capture_pinned_mig ?
        capture_signature_generation_mig : payload_signature_generation;
    wire [15:0] signature_raw_low_mig = capture_pinned_mig ?
        capture_raw_low_signature_mig : completed_raw_low_signature_mig;
    wire [15:0] signature_raw_high_mig = capture_pinned_mig ?
        capture_raw_high_signature_mig : completed_raw_high_signature_mig;
    wire [63:0] signature_visual_stage_mig = capture_pinned_mig ?
        capture_visual_stage_signatures_mig :
        completed_visual_stage_signatures_mig;
    wire [15:0] signature_fetch_low_mig = capture_pinned_mig ?
        capture_fetch_low_signature_mig : completed_fetch_low_signature_mig;
    wire [15:0] signature_fetch_high_mig = capture_pinned_mig ?
        capture_fetch_high_signature_mig : completed_fetch_high_signature_mig;
    wire [7:0] signature_fetch_generation_mig = capture_pinned_mig ?
        capture_fetch_signature_generation_mig :
        completed_fetch_signature_generation_mig;
    wire signature_fetch_valid_mig = capture_pinned_mig ?
        capture_fetch_signature_valid_mig :
        completed_fetch_signature_valid_mig;
    wire [91:0] signature_resolver_telemetry_mig = capture_pinned_mig ?
        capture_resolver_telemetry_mig : completed_resolver_telemetry_mig;
    wire signature_resolver_valid_mig = capture_pinned_mig ?
        capture_resolver_valid_mig : completed_resolver_valid_mig;
    wire [91:0] signature_resolver_detail_mig = capture_pinned_mig ?
        capture_resolver_detail_mig : completed_resolver_detail_mig;

    // Only the final synchronizer stage is functional state.  ANDing the
    // complete shift register would fan metastable stages into the scan
    // arbiter and could asynchronously discard/restart a DDR scan epoch.
    wire video_alive_mig = video_alive_sync_mig[2];
    // Vivado 2020.1's XPM wr_data_count reports the pre-write occupancy for
    // one write clock after accepting a word.  The matching scan response
    // retires from scan_read_outstanding on that same edge.  Preserve that
    // just-written reservation explicitly until the registered count catches
    // up; otherwise the transient undercount can issue word 64 into a full
    // FIFO, drop it, and leave same-epoch word 65 at the consumer head.
    wire [7:0] read_fifo_reserved =
        {1'b0, read_fifo_wr_count} + {5'd0, scan_read_outstanding} +
        {7'd0, read_fifo_count_lag_credit_mig} +
        {7'd0, scan_response_stage_valid_mig} +
        {7'd0, scan_response_push_valid_mig};
    wire write_urgent_mig = write_urgent_sync_mig[1];
    wire deadline_urgent_mig = scan_urgent_mig || write_urgent_mig;
    wire choose_capture = capture_pending_mig && !deadline_urgent_mig &&
                          (scan_read_outstanding == 0);
    // gmii_async_fifo's portable implementation reserves its final physical
    // slot to distinguish full from empty, so 63 raw words are admissible in
    // this 64-entry FIFO.  Include every accepted-but-not-returned scan read
    // before issuing another command; otherwise the last response can arrive
    // with wr_full asserted and silently remove one 16-pixel display word.
    wire scan_read_eligible =
                              (!capture_pending_mig || deadline_urgent_mig) &&
                              video_alive_mig &&
                              scan_issue_active_mig && !read_fifo_full &&
                              (scan_read_outstanding < MAX_SCAN_READS) &&
                              (read_fifo_reserved < 8'd63);
    // Scanout and source capture are both finite-deadline clients of this
    // one video port.  A sole urgent side wins.  If both sides have crossed
    // their hysteresis threshold (or neither has), preserve round-robin
    // progress between them.  An empty write FIFO always lets scan advance.
    wire choose_read = scan_read_eligible &&
                       (!write_fifo_valid ||
                        (scan_urgent_mig && !write_urgent_mig) ||
                        ((scan_urgent_mig == write_urgent_mig) &&
                         !last_was_read));
    wire choose_write = !choose_capture && write_fifo_valid && !choose_read;
    // The toggle preserves the video-reset/relock request below.  The Gray
    // generation makes ordinary vblank requests durable even if an even
    // number accumulate while one old scan response is still outstanding.
    wire scan_restart_pending_mig =
        (scan_restart_sync_mig[2] != scan_restart_seen_mig) ||
        (scan_epoch_gray_sync_mig3 != scan_epoch_gray_seen_mig);
    wire scan_restart_accept_mig =
        scan_restart_pending_mig && video_alive_mig &&
        (mig_state == ST_IDLE) && (scan_read_outstanding == 0) &&
        !scan_response_stage_valid_mig && !scan_response_push_valid_mig;
    assign write_fifo_rd_en = (mig_state == ST_IDLE) &&
                              !scan_restart_pending_mig && choose_write;
    wire scan_read_response_fire = app_rd_data_valid &&
                                   (scan_read_outstanding != 0);
    wire scan_response_push_ready_mig =
        !scan_response_push_valid_mig || !read_fifo_full;
    wire scan_response_stage_ready_mig =
        !scan_response_stage_valid_mig || scan_response_push_ready_mig;
    wire scan_response_admit_mig = scan_read_response_fire &&
                                   scan_response_stage_ready_mig &&
                                   !read_fifo_full && video_alive_mig &&
                                   !scan_restart_pending_mig &&
                                   !op_scan_discard_mig;
    assign read_fifo_wr_en = scan_response_push_valid_mig &&
                             !read_fifo_full;

    wire [25:0] bank_offset = op_bank * FRAME_BANK_DW_STRIDE;
    wire [25:0] word_offset = {7'd0, op_index, 3'b000};
    wire [25:0] op_dw_addr = FRAME_BASE_DW_ADDR + bank_offset + word_offset;
    assign app_addr = {{(APP_ADDR_WIDTH-26){1'b0}}, op_dw_addr};
    assign app_cmd = op_is_write ? MIG_CMD_WRITE : MIG_CMD_READ;
    assign app_en = ((mig_state == ST_READ_CMD) ||
                     ((mig_state == ST_WRITE) && !write_cmd_accepted));
    // Priority is a stable property of one held native request.  It is never
    // asserted for management capture reads.  A selected scan read or source
    // write may bypass ordinary outer round robin only while its own bounded
    // FIFO hysteresis marks it urgent.
    assign app_hi_pri = op_hi_pri;
    assign app_wdf_data = op_data;
    assign app_wdf_end = app_wdf_wren;
    assign app_wdf_mask = {APP_MASK_WIDTH{1'b0}};
    assign app_wdf_wren = (mig_state == ST_WRITE) && !write_data_accepted;

    wire write_cmd_fire = app_en && app_rdy;
    wire scan_read_cmd_fire = (mig_state == ST_READ_CMD) &&
                              !op_is_capture && write_cmd_fire;
    wire write_data_fire = app_wdf_wren && app_wdf_rdy;
    // Modular 16-bit subtraction is exact for any response below 218 us at
    // 300.12 MHz, far beyond the 16-pixel display deadline.
    wire [15:0] service_response_latency_mig =
        service_cycle_mig - service_timestamp_0_mig;
    // Retire only from registered native-interface acceptance.  Publishing a
    // completed frame directly from app_rdy/app_wdf_rdy makes raw MIG/arbiter
    // feedback drive every completed-bank snapshot CE in the same 3.332 ns
    // cycle.  The registered edge preserves exactly-once command/data issue
    // and atomic publication while adding only one internal UI-clock cycle.
    wire write_complete = write_cmd_accepted && write_data_accepted;
    wire [15:0] mig_low_signature_next = signature_step(
        (op_index == 0) ? 16'd0 : mig_low_signature,
        op_data[191:0], op_index);
    wire [15:0] mig_high_signature_next = signature_step(
        (op_index == 0) ? 16'd0 : mig_high_signature,
        op_data[383:192], op_index);

    always @(posedge mig_clk) begin
        if (mig_rst || !init_calib_complete) begin
            mig_state <= ST_IDLE;
            op_is_write <= 1'b0;
            op_is_capture <= 1'b0;
            op_hi_pri <= 1'b0;
            op_bank <= 2'd0;
            op_index <= 16'd0;
            op_last <= 1'b0;
            op_data <= 512'd0;
            write_cmd_accepted <= 1'b0;
            write_data_accepted <= 1'b0;
            last_was_read <= 1'b0;
            scan_index_mig <= 16'd0;
            scan_response_index_mig <= 16'd0;
            scan_read_outstanding <= 3'd0;
            scan_issue_active_mig <= 1'b0;
            scan_active_mig <= 1'b0;
            scan_urgent_mig <= 1'b0;
            scan_bank_mig <= 2'd0;
            scan_valid_mig <= 1'b0;
            scan_restart_sync_mig <= 3'b000;
            scan_epoch_gray_sync_mig1 <= 8'd0;
            scan_epoch_gray_sync_mig2 <= 8'd0;
            scan_epoch_gray_sync_mig3 <= 8'd0;
            scan_bank_sync_mig1 <= 2'd0;
            scan_bank_sync_mig2 <= 2'd0;
            scan_valid_sync_mig <= 2'b00;
            video_alive_sync_mig <= 3'b000;
            write_urgent_sync_mig <= 2'b00;
            scan_restart_seen_mig <= 1'b0;
            scan_epoch_gray_seen_mig <= 8'd0;
            scan_epoch_mig <= 8'd0;
            scan_response_data_stage_mig <= 512'd0;
            scan_response_index_stage_mig <= 16'd0;
            scan_response_epoch_stage_mig <= 8'd0;
            scan_response_stage_valid_mig <= 1'b0;
            scan_response_data_push_mig <= 512'd0;
            scan_response_index_push_mig <= 16'd0;
            scan_response_epoch_push_mig <= 8'd0;
            scan_response_push_valid_mig <= 1'b0;
            op_scan_discard_mig <= 1'b0;
            read_fifo_count_lag_credit_mig <= 1'b0;
            completed_bank_mig <= 2'd0;
            completed_toggle_mig <= 1'b0;
            completed_valid_mig <= 1'b0;
            ownership_payload_mig <= {1'b0, 2'd0, 3'b001};
            ownership_event_toggle_mig <= 1'b0;
            ownership_guard_mask_mig <= 3'b000;
            ownership_ack_sync_mig <= 2'b00;
            mig_read_count <= 32'd0;
            mig_write_count <= 32'd0;
            completed_frame_count_mig <= 16'd0;
            service_cycle_mig <= 16'd0;
            service_timestamp_0_mig <= 16'd0;
            service_timestamp_1_mig <= 16'd0;
            service_timestamp_2_mig <= 16'd0;
            service_timestamp_3_mig <= 16'd0;
            service_command_wait_run_mig <= 16'd0;
            service_command_count_mig <= 16'd0;
            service_response_count_mig <= 16'd0;
            service_max_command_wait_mig <= 16'd0;
            service_max_response_latency_mig <= 16'd0;
            service_min_resident_mig <= 7'd63;
            service_max_outstanding_mig <= 3'd0;
            service_response_fast_mig <= 8'd0;
            service_response_mid_mig <= 8'd0;
            service_response_slow_mig <= 8'd0;
            last_service_valid_mig <= 1'b0;
            last_service_epoch_mig <= 8'd0;
            last_service_command_count_mig <= 16'd0;
            last_service_response_count_mig <= 16'd0;
            last_service_max_command_wait_mig <= 16'd0;
            last_service_max_response_latency_mig <= 16'd0;
            last_service_min_resident_mig <= 7'd0;
            last_service_max_outstanding_mig <= 3'd0;
            last_service_response_fast_mig <= 8'd0;
            last_service_response_mid_mig <= 8'd0;
            last_service_response_slow_mig <= 8'd0;
            capture_pinned_mig <= 1'b0;
            capture_bank_mig <= 2'd0;
            capture_frame_mig <= 16'd0;
            mig_low_signature <= 16'd0;
            mig_high_signature <= 16'd0;
            completed_src_low_signature_mig <= 16'd0;
            completed_src_high_signature_mig <= 16'd0;
            completed_mig_low_signature <= 16'd0;
            completed_mig_high_signature <= 16'd0;
            completed_raw_low_signature_mig <= 16'd0;
            completed_raw_high_signature_mig <= 16'd0;
            completed_visual_stage_signatures_mig <= 64'd0;
            completed_fetch_low_signature_mig <= 16'd0;
            completed_fetch_high_signature_mig <= 16'd0;
            completed_fetch_signature_generation_mig <= 8'd0;
            completed_fetch_signature_valid_mig <= 1'b0;
            capture_src_low_signature_mig <= 16'd0;
            capture_src_high_signature_mig <= 16'd0;
            capture_mig_low_signature <= 16'd0;
            capture_mig_high_signature <= 16'd0;
            capture_raw_low_signature_mig <= 16'd0;
            capture_raw_high_signature_mig <= 16'd0;
            capture_visual_stage_signatures_mig <= 64'd0;
            capture_fetch_low_signature_mig <= 16'd0;
            capture_fetch_high_signature_mig <= 16'd0;
            capture_fetch_signature_generation_mig <= 8'd0;
            capture_fetch_signature_valid_mig <= 1'b0;
            completed_resolver_telemetry_mig <= 92'd0;
            completed_resolver_valid_mig <= 1'b0;
            capture_resolver_telemetry_mig <= 92'd0;
            capture_resolver_valid_mig <= 1'b0;
            completed_resolver_detail_mig <= 92'd0;
            capture_resolver_detail_mig <= 92'd0;
            payload_signature_valid <= 1'b0;
            payload_signature_generation <= 8'd0;
            capture_signature_generation_mig <= 8'd0;
            src_low_signature_sync1[0] <= 16'd0;
            src_low_signature_sync1[1] <= 16'd0;
            src_low_signature_sync1[2] <= 16'd0;
            src_low_signature_sync2[0] <= 16'd0;
            src_low_signature_sync2[1] <= 16'd0;
            src_low_signature_sync2[2] <= 16'd0;
            src_high_signature_sync1[0] <= 16'd0;
            src_high_signature_sync1[1] <= 16'd0;
            src_high_signature_sync1[2] <= 16'd0;
            src_high_signature_sync2[0] <= 16'd0;
            src_high_signature_sync2[1] <= 16'd0;
            src_high_signature_sync2[2] <= 16'd0;
            src_raw_low_signature_sync1[0] <= 16'd0;
            src_raw_low_signature_sync1[1] <= 16'd0;
            src_raw_low_signature_sync1[2] <= 16'd0;
            src_raw_low_signature_sync2[0] <= 16'd0;
            src_raw_low_signature_sync2[1] <= 16'd0;
            src_raw_low_signature_sync2[2] <= 16'd0;
            src_raw_high_signature_sync1[0] <= 16'd0;
            src_raw_high_signature_sync1[1] <= 16'd0;
            src_raw_high_signature_sync1[2] <= 16'd0;
            src_raw_high_signature_sync2[0] <= 16'd0;
            src_raw_high_signature_sync2[1] <= 16'd0;
            src_raw_high_signature_sync2[2] <= 16'd0;
            src_visual_stage_signatures_sync1[0] <= 64'd0;
            src_visual_stage_signatures_sync1[1] <= 64'd0;
            src_visual_stage_signatures_sync1[2] <= 64'd0;
            src_visual_stage_signatures_sync2[0] <= 64'd0;
            src_visual_stage_signatures_sync2[1] <= 64'd0;
            src_visual_stage_signatures_sync2[2] <= 64'd0;
            src_fetch_low_signature_sync1[0] <= 16'd0;
            src_fetch_low_signature_sync1[1] <= 16'd0;
            src_fetch_low_signature_sync1[2] <= 16'd0;
            src_fetch_low_signature_sync2[0] <= 16'd0;
            src_fetch_low_signature_sync2[1] <= 16'd0;
            src_fetch_low_signature_sync2[2] <= 16'd0;
            src_fetch_high_signature_sync1[0] <= 16'd0;
            src_fetch_high_signature_sync1[1] <= 16'd0;
            src_fetch_high_signature_sync1[2] <= 16'd0;
            src_fetch_high_signature_sync2[0] <= 16'd0;
            src_fetch_high_signature_sync2[1] <= 16'd0;
            src_fetch_high_signature_sync2[2] <= 16'd0;
            src_fetch_signature_generation_sync1[0] <= 8'd0;
            src_fetch_signature_generation_sync1[1] <= 8'd0;
            src_fetch_signature_generation_sync1[2] <= 8'd0;
            src_fetch_signature_generation_sync2[0] <= 8'd0;
            src_fetch_signature_generation_sync2[1] <= 8'd0;
            src_fetch_signature_generation_sync2[2] <= 8'd0;
            src_fetch_signature_valid_sync1 <= 3'b000;
            src_fetch_signature_valid_sync2 <= 3'b000;
            src_resolver_telemetry_sync1[0] <= 92'd0;
            src_resolver_telemetry_sync1[1] <= 92'd0;
            src_resolver_telemetry_sync1[2] <= 92'd0;
            src_resolver_telemetry_sync2[0] <= 92'd0;
            src_resolver_telemetry_sync2[1] <= 92'd0;
            src_resolver_telemetry_sync2[2] <= 92'd0;
            src_resolver_detail_sync1[0] <= 92'd0;
            src_resolver_detail_sync1[1] <= 92'd0;
            src_resolver_detail_sync1[2] <= 92'd0;
            src_resolver_detail_sync2[0] <= 92'd0;
            src_resolver_detail_sync2[1] <= 92'd0;
            src_resolver_detail_sync2[2] <= 92'd0;
            src_resolver_valid_sync1 <= 3'b000;
            src_resolver_valid_sync2 <= 3'b000;
            src_underrun_count_sync1 <= 32'd0;
            src_underrun_count_sync2 <= 32'd0;
            src_protocol_error_count_sync1 <= 32'd0;
            src_protocol_error_count_sync2 <= 32'd0;
            capture_cache_valid_mig <= 1'b0;
            capture_cache_index_mig <= 16'd0;
            capture_pending_mig <= 1'b0;
            capture_pending_index_mig <= 16'd0;
            capture_pending_toggle_mig <= 1'b0;
            capture_error_mig <= 1'b0;
            capture_command_sync_mig <= 3'b000;
            capture_command_seen_mig <= 1'b0;
            capture_command_mig1 <= 2'd0;
            capture_command_mig2 <= 2'd0;
            capture_word_index_mig1 <= 16'd0;
            capture_word_index_mig2 <= 16'd0;
            capture_ack_toggle <= 1'b0;
            capture_word_data <= 512'd0;
            capture_word_valid <= 1'b0;
            status_request_sync_mig <= 3'b000;
            status_src_ack_sync_mig <= 2'b00;
            status_video_ack_sync_mig <= 2'b00;
            status_request_seen_mig <= 1'b0;
            status_snapshot_pending_mig <= 1'b0;
            status_ack_toggle <= 1'b0;
            status <= 128'd0;
            signature_status <= 128'd0;
            stage_signature_status <= 128'd0;
            resolver_status <= 128'd0;
            resolver_detail_status <= 128'd0;
            visual_stage_status <= 128'd0;
            scan_service_status <= 128'd0;
            scan_latency_status <= 128'd0;
        end else begin
            read_fifo_count_lag_credit_mig <= read_fifo_wr_en;
            // The two elastic stages can each exchange and refill in one
            // cycle, preserving consecutive native-response throughput.  A
            // staged word belongs to the scan generation that admitted it.
            // Video-only reset can hold the XPM FIFO full while the restart
            // request crosses into this domain, so do not wait for stale
            // staged words to reach the reset FIFO.  Drop both as soon as the
            // consumer is absent or a newer generation is pending.
            if (!video_alive_mig || scan_restart_pending_mig) begin
                scan_response_stage_valid_mig <= 1'b0;
                scan_response_push_valid_mig <= 1'b0;
            end else begin
                if (scan_response_push_ready_mig) begin
                    scan_response_push_valid_mig <=
                        scan_response_stage_valid_mig;
                    if (scan_response_stage_valid_mig) begin
                        scan_response_data_push_mig <=
                            scan_response_data_stage_mig;
                        scan_response_index_push_mig <=
                            scan_response_index_stage_mig;
                        scan_response_epoch_push_mig <=
                            scan_response_epoch_stage_mig;
                    end
                end
                if (scan_response_admit_mig) begin
                    scan_response_data_stage_mig <= app_rd_data;
                    scan_response_index_stage_mig <=
                        scan_response_index_mig;
                    scan_response_epoch_stage_mig <= scan_epoch_mig;
                    scan_response_stage_valid_mig <= 1'b1;
                end else if (scan_response_push_ready_mig) begin
                    scan_response_stage_valid_mig <= 1'b0;
                end
            end
            // Read-only service accounting. None of these registers feeds the
            // request, FIFO, urgency, reset, or raster control paths.
            service_cycle_mig <= service_cycle_mig + 16'd1;
            if (scan_active_mig &&
                read_fifo_wr_count < service_min_resident_mig)
                service_min_resident_mig <= read_fifo_wr_count;

            if ((mig_state == ST_READ_CMD) && !op_is_capture) begin
                if (scan_read_cmd_fire) begin
                    if (service_command_count_mig != 16'hffff)
                        service_command_count_mig <=
                            service_command_count_mig + 16'd1;
                    if (service_command_wait_run_mig >
                        service_max_command_wait_mig)
                        service_max_command_wait_mig <=
                            service_command_wait_run_mig;
                    service_command_wait_run_mig <= 16'd0;
                end else if (app_en && !app_rdy &&
                             service_command_wait_run_mig != 16'hffff) begin
                    service_command_wait_run_mig <=
                        service_command_wait_run_mig + 16'd1;
                end
            end else begin
                service_command_wait_run_mig <= 16'd0;
            end

            if (scan_read_cmd_fire && !scan_read_response_fire &&
                (scan_read_outstanding + 3'd1 >
                 service_max_outstanding_mig))
                service_max_outstanding_mig <=
                    scan_read_outstanding + 3'd1;

            case ({scan_read_cmd_fire, scan_read_response_fire})
                2'b10: begin
                    case (scan_read_outstanding)
                        3'd0: service_timestamp_0_mig <= service_cycle_mig;
                        3'd1: service_timestamp_1_mig <= service_cycle_mig;
                        3'd2: service_timestamp_2_mig <= service_cycle_mig;
                        3'd3: service_timestamp_3_mig <= service_cycle_mig;
                        default: begin end
                    endcase
                end
                2'b01: begin
                    service_timestamp_0_mig <= service_timestamp_1_mig;
                    service_timestamp_1_mig <= service_timestamp_2_mig;
                    service_timestamp_2_mig <= service_timestamp_3_mig;
                    service_timestamp_3_mig <= 16'd0;
                end
                2'b11: begin
                    case (scan_read_outstanding)
                        3'd1: service_timestamp_0_mig <= service_cycle_mig;
                        3'd2: begin
                            service_timestamp_0_mig <=
                                service_timestamp_1_mig;
                            service_timestamp_1_mig <= service_cycle_mig;
                        end
                        3'd3: begin
                            service_timestamp_0_mig <=
                                service_timestamp_1_mig;
                            service_timestamp_1_mig <=
                                service_timestamp_2_mig;
                            service_timestamp_2_mig <= service_cycle_mig;
                        end
                        3'd4: begin
                            service_timestamp_0_mig <=
                                service_timestamp_1_mig;
                            service_timestamp_1_mig <=
                                service_timestamp_2_mig;
                            service_timestamp_2_mig <=
                                service_timestamp_3_mig;
                            service_timestamp_3_mig <= service_cycle_mig;
                        end
                        default: begin end
                    endcase
                end
                default: begin end
            endcase

            if (scan_read_response_fire) begin
                if (service_response_count_mig != 16'hffff)
                    service_response_count_mig <=
                        service_response_count_mig + 16'd1;
                if (service_response_latency_mig >
                    service_max_response_latency_mig)
                    service_max_response_latency_mig <=
                        service_response_latency_mig;
                if (service_response_latency_mig <= 16'd32) begin
                    if (service_response_fast_mig != 8'hff)
                        service_response_fast_mig <=
                            service_response_fast_mig + 8'd1;
                end else if (service_response_latency_mig <= 16'd128) begin
                    if (service_response_mid_mig != 8'hff)
                        service_response_mid_mig <=
                            service_response_mid_mig + 8'd1;
                end else if (service_response_slow_mig != 8'hff) begin
                    service_response_slow_mig <=
                        service_response_slow_mig + 8'd1;
                end
            end

            ownership_ack_sync_mig <= {
                ownership_ack_sync_mig[0], ownership_ack_toggle_src
            };
            // Publish only one atomic reservation record at a time.  The
            // previous record remains conservative while its acknowledgement
            // crosses back, and the guard retains superseded completions until
            // MIG has accepted the next video scan-bank event.
            if ((ownership_ack_sync_mig[1] ==
                 ownership_event_toggle_mig) &&
                (ownership_state_mig != ownership_payload_mig)) begin
                ownership_payload_mig <= ownership_state_mig;
                ownership_event_toggle_mig <=
                    ~ownership_event_toggle_mig;
            end
            src_low_signature_sync1[0] <= src_low_signature_bank[0];
            src_low_signature_sync1[1] <= src_low_signature_bank[1];
            src_low_signature_sync1[2] <= src_low_signature_bank[2];
            src_low_signature_sync2[0] <= src_low_signature_sync1[0];
            src_low_signature_sync2[1] <= src_low_signature_sync1[1];
            src_low_signature_sync2[2] <= src_low_signature_sync1[2];
            src_high_signature_sync1[0] <= src_high_signature_bank[0];
            src_high_signature_sync1[1] <= src_high_signature_bank[1];
            src_high_signature_sync1[2] <= src_high_signature_bank[2];
            src_high_signature_sync2[0] <= src_high_signature_sync1[0];
            src_high_signature_sync2[1] <= src_high_signature_sync1[1];
            src_high_signature_sync2[2] <= src_high_signature_sync1[2];
            src_raw_low_signature_sync1[0] <=
                src_raw_low_signature_bank[0];
            src_raw_low_signature_sync1[1] <=
                src_raw_low_signature_bank[1];
            src_raw_low_signature_sync1[2] <=
                src_raw_low_signature_bank[2];
            src_raw_low_signature_sync2[0] <=
                src_raw_low_signature_sync1[0];
            src_raw_low_signature_sync2[1] <=
                src_raw_low_signature_sync1[1];
            src_raw_low_signature_sync2[2] <=
                src_raw_low_signature_sync1[2];
            src_raw_high_signature_sync1[0] <=
                src_raw_high_signature_bank[0];
            src_raw_high_signature_sync1[1] <=
                src_raw_high_signature_bank[1];
            src_raw_high_signature_sync1[2] <=
                src_raw_high_signature_bank[2];
            src_raw_high_signature_sync2[0] <=
                src_raw_high_signature_sync1[0];
            src_raw_high_signature_sync2[1] <=
                src_raw_high_signature_sync1[1];
            src_raw_high_signature_sync2[2] <=
                src_raw_high_signature_sync1[2];
            src_visual_stage_signatures_sync1[0] <=
                src_visual_stage_signatures_bank[0];
            src_visual_stage_signatures_sync1[1] <=
                src_visual_stage_signatures_bank[1];
            src_visual_stage_signatures_sync1[2] <=
                src_visual_stage_signatures_bank[2];
            src_visual_stage_signatures_sync2[0] <=
                src_visual_stage_signatures_sync1[0];
            src_visual_stage_signatures_sync2[1] <=
                src_visual_stage_signatures_sync1[1];
            src_visual_stage_signatures_sync2[2] <=
                src_visual_stage_signatures_sync1[2];
            src_fetch_low_signature_sync1[0] <=
                src_fetch_low_signature_bank[0];
            src_fetch_low_signature_sync1[1] <=
                src_fetch_low_signature_bank[1];
            src_fetch_low_signature_sync1[2] <=
                src_fetch_low_signature_bank[2];
            src_fetch_low_signature_sync2[0] <=
                src_fetch_low_signature_sync1[0];
            src_fetch_low_signature_sync2[1] <=
                src_fetch_low_signature_sync1[1];
            src_fetch_low_signature_sync2[2] <=
                src_fetch_low_signature_sync1[2];
            src_fetch_high_signature_sync1[0] <=
                src_fetch_high_signature_bank[0];
            src_fetch_high_signature_sync1[1] <=
                src_fetch_high_signature_bank[1];
            src_fetch_high_signature_sync1[2] <=
                src_fetch_high_signature_bank[2];
            src_fetch_high_signature_sync2[0] <=
                src_fetch_high_signature_sync1[0];
            src_fetch_high_signature_sync2[1] <=
                src_fetch_high_signature_sync1[1];
            src_fetch_high_signature_sync2[2] <=
                src_fetch_high_signature_sync1[2];
            src_fetch_signature_generation_sync1[0] <=
                src_fetch_signature_generation_bank[0];
            src_fetch_signature_generation_sync1[1] <=
                src_fetch_signature_generation_bank[1];
            src_fetch_signature_generation_sync1[2] <=
                src_fetch_signature_generation_bank[2];
            src_fetch_signature_generation_sync2[0] <=
                src_fetch_signature_generation_sync1[0];
            src_fetch_signature_generation_sync2[1] <=
                src_fetch_signature_generation_sync1[1];
            src_fetch_signature_generation_sync2[2] <=
                src_fetch_signature_generation_sync1[2];
            src_fetch_signature_valid_sync1[0] <=
                src_fetch_signature_valid_bank[0];
            src_fetch_signature_valid_sync1[1] <=
                src_fetch_signature_valid_bank[1];
            src_fetch_signature_valid_sync1[2] <=
                src_fetch_signature_valid_bank[2];
            src_fetch_signature_valid_sync2 <=
                src_fetch_signature_valid_sync1;
            src_resolver_telemetry_sync1[0] <=
                src_resolver_telemetry_bank[0];
            src_resolver_telemetry_sync1[1] <=
                src_resolver_telemetry_bank[1];
            src_resolver_telemetry_sync1[2] <=
                src_resolver_telemetry_bank[2];
            src_resolver_telemetry_sync2[0] <=
                src_resolver_telemetry_sync1[0];
            src_resolver_telemetry_sync2[1] <=
                src_resolver_telemetry_sync1[1];
            src_resolver_telemetry_sync2[2] <=
                src_resolver_telemetry_sync1[2];
            src_resolver_detail_sync1[0] <= src_resolver_detail_bank[0];
            src_resolver_detail_sync1[1] <= src_resolver_detail_bank[1];
            src_resolver_detail_sync1[2] <= src_resolver_detail_bank[2];
            src_resolver_detail_sync2[0] <= src_resolver_detail_sync1[0];
            src_resolver_detail_sync2[1] <= src_resolver_detail_sync1[1];
            src_resolver_detail_sync2[2] <= src_resolver_detail_sync1[2];
            src_resolver_valid_sync1[0] <= src_resolver_valid_bank[0];
            src_resolver_valid_sync1[1] <= src_resolver_valid_bank[1];
            src_resolver_valid_sync1[2] <= src_resolver_valid_bank[2];
            src_resolver_valid_sync2 <= src_resolver_valid_sync1;
            src_underrun_count_sync1 <= src_underrun_count;
            src_underrun_count_sync2 <= src_underrun_count_sync1;
            src_protocol_error_count_sync1 <= src_protocol_error_count;
            src_protocol_error_count_sync2 <=
                src_protocol_error_count_sync1;
            scan_restart_sync_mig <=
                {scan_restart_sync_mig[1:0], scan_restart_toggle_video};
            scan_epoch_gray_sync_mig1 <= scan_epoch_gray_video;
            scan_epoch_gray_sync_mig2 <= scan_epoch_gray_sync_mig1;
            scan_epoch_gray_sync_mig3 <= scan_epoch_gray_sync_mig2;
            scan_bank_sync_mig1 <= display_bank_video;
            scan_bank_sync_mig2 <= scan_bank_sync_mig1;
            scan_valid_sync_mig <= {scan_valid_sync_mig[0], scan_valid_video};
            video_alive_sync_mig <=
                {video_alive_sync_mig[1:0], video_rst_n};
            write_urgent_sync_mig <=
                {write_urgent_sync_mig[0], write_urgent_src};

            // One resident FIFO word covers 16 output pixels (246.154 ns at
            // 65 MHz).  Trigger from returned/resident words, never commands
            // still outstanding in DDR.  Twenty words provide 4.92 us before
            // the explicit one-word FIFO pointer/output margin; the separate
            // reserved count still prevents response overflow.  Hysteresis
            // bounds a scan-only urgent refill at 28 resident words.
            if (!video_alive_mig || !scan_active_mig)
                scan_urgent_mig <= 1'b0;
            else if (read_fifo_wr_count <= SCAN_URGENT_LOW_WATER)
                scan_urgent_mig <= 1'b1;
            else if (read_fifo_wr_count >= SCAN_URGENT_HIGH_WATER)
                scan_urgent_mig <= 1'b0;
            capture_command_sync_mig <=
                {capture_command_sync_mig[1:0], capture_command_toggle};
            capture_command_mig1 <= capture_command;
            capture_command_mig2 <= capture_command_mig1;
            capture_word_index_mig1 <= capture_word_index;
            capture_word_index_mig2 <= capture_word_index_mig1;
            status_request_sync_mig <=
                {status_request_sync_mig[1:0], status_request_toggle};
            status_src_ack_sync_mig <=
                {status_src_ack_sync_mig[0], status_src_ack_toggle};
            status_video_ack_sync_mig <=
                {status_video_ack_sync_mig[0], status_video_ack_toggle};

            if (status_request_sync_mig[2] != status_request_seen_mig) begin
                status_request_seen_mig <= status_request_sync_mig[2];
                status_snapshot_pending_mig <= 1'b1;
            end

            if (status_snapshot_pending_mig &&
                status_src_ack_sync_mig[1] == status_request_seen_mig &&
                status_video_ack_sync_mig[1] == status_request_seen_mig) begin
                status <= {
                    8'h56,
                    completed_toggle_mig,
                    completed_bank_mig,
                    video_status_hold[19:18],
                    src_status_hold[38:37],
                    src_status_hold[36],
                    src_status_hold[35],
                    read_fifo_full,
                    video_status_hold[17],
                    scan_active_mig,
                    mig_state,
                    video_status_hold[16:1],
                    src_status_hold[34:19],
                    mig_read_count[15:0],
                    mig_write_count[15:0],
                    src_status_hold[18:3],
                    src_status_hold[2:0],
                    scan_valid_mig,
                    video_status_hold[0],
                    app_rd_data_end,
                    payload_signature_valid,
                    completed_src_low_signature_mig ==
                        completed_mig_low_signature,
                    completed_src_high_signature_mig ==
                        completed_mig_high_signature,
                    completed_src_low_signature_mig != 16'd0,
                    completed_src_high_signature_mig != 16'd0,
                    completed_mig_low_signature != 16'd0,
                    completed_mig_high_signature != 16'd0,
                    src_underrun_count_sync2 != 32'd0,
                    src_protocol_error_count_sync2 != 32'd0,
                    payload_signature_generation,
                    2'd0
                };
                signature_status <= {
                    8'h53, // "S"
                    8'h01, // raw payload-signature protocol version
                    completed_valid_mig,
                    signature_bank_mig,
                    signature_frame_mig,
                    signature_src_low_mig,
                    signature_src_high_mig,
                    signature_mig_low_mig,
                    signature_mig_high_mig,
                    signature_generation_mig,
                    21'd0
                };
                stage_signature_status <= {
                    8'h46, // "F" (fetch/raw stage signatures)
                    8'h01,
                    completed_valid_mig,
                    signature_bank_mig,
                    signature_frame_mig,
                    signature_raw_low_mig,
                    signature_raw_high_mig,
                    signature_fetch_low_mig,
                    signature_fetch_high_mig,
                    signature_generation_mig,
                    signature_fetch_valid_mig,
                    signature_fetch_generation_mig,
                    12'd0
                };
                resolver_status <= {
                    8'h52, // "R" (physical resolver frame record)
                    8'h01,
                    completed_valid_mig,
                    signature_bank_mig,
                    signature_frame_mig,
                    signature_resolver_valid_mig,
                    signature_resolver_telemetry_mig
                };
                resolver_detail_status <= {
                    8'h44, // "D" (physical resolver detail record)
                    8'h01,
                    completed_valid_mig,
                    signature_bank_mig,
                    signature_frame_mig,
                    signature_resolver_valid_mig,
                    signature_resolver_detail_mig
                };
                visual_stage_status <= {
                    8'h50, // "P" (physical resolver pipeline signatures)
                    8'h01,
                    completed_valid_mig,
                    signature_bank_mig,
                    signature_frame_mig,
                    signature_visual_stage_mig,
                    29'd0
                };
                scan_service_status <= {
                    8'h51, // "Q" (completed-frame scan miss record)
                    8'h01,
                    video_service_hold,
                    40'd0
                };
                scan_latency_status <= {
                    8'h4c, // "L" (completed-epoch DDR service ledger)
                    8'h01,
                    last_service_epoch_mig,
                    last_service_command_count_mig,
                    last_service_response_count_mig,
                    last_service_max_command_wait_mig,
                    last_service_max_response_latency_mig,
                    last_service_min_resident_mig,
                    last_service_max_outstanding_mig,
                    last_service_response_fast_mig,
                    last_service_response_mid_mig,
                    last_service_response_slow_mig,
                    last_service_valid_mig,
                    video_service_hold[71],
                    video_service_hold[70:63] == last_service_epoch_mig,
                    last_service_command_count_mig ==
                        last_service_response_count_mig,
                    last_service_min_resident_mig == 0,
                    last_service_response_slow_mig != 0
                };
                status_ack_toggle <= status_request_seen_mig;
                status_snapshot_pending_mig <= 1'b0;
            end

            if (capture_command_sync_mig[2] != capture_command_seen_mig) begin
                capture_command_seen_mig <= capture_command_sync_mig[2];
                capture_error_mig <= 1'b0;
                case (capture_command_mig2)
                    2'd1: begin
                        capture_pending_mig <= 1'b0;
                        if (completed_valid_mig) begin
                            capture_pinned_mig <= 1'b1;
                            capture_bank_mig <= completed_bank_mig;
                            capture_frame_mig <= completed_frame_count_mig;
                            capture_src_low_signature_mig <=
                                completed_src_low_signature_mig;
                            capture_src_high_signature_mig <=
                                completed_src_high_signature_mig;
                            capture_mig_low_signature <=
                                completed_mig_low_signature;
                            capture_mig_high_signature <=
                                completed_mig_high_signature;
                            capture_raw_low_signature_mig <=
                                completed_raw_low_signature_mig;
                            capture_raw_high_signature_mig <=
                                completed_raw_high_signature_mig;
                            capture_visual_stage_signatures_mig <=
                                completed_visual_stage_signatures_mig;
                            capture_fetch_low_signature_mig <=
                                completed_fetch_low_signature_mig;
                            capture_fetch_high_signature_mig <=
                                completed_fetch_high_signature_mig;
                            capture_fetch_signature_generation_mig <=
                                completed_fetch_signature_generation_mig;
                            capture_fetch_signature_valid_mig <=
                                completed_fetch_signature_valid_mig;
                            capture_resolver_telemetry_mig <=
                                completed_resolver_telemetry_mig;
                            capture_resolver_valid_mig <=
                                completed_resolver_valid_mig;
                            capture_resolver_detail_mig <=
                                completed_resolver_detail_mig;
                            capture_signature_generation_mig <=
                                payload_signature_generation;
                            capture_cache_valid_mig <= 1'b0;
                            capture_word_valid <= 1'b0;
                        end else begin
                            capture_pinned_mig <= 1'b0;
                            capture_error_mig <= 1'b1;
                        end
                        capture_ack_toggle <= capture_command_sync_mig[2];
                    end
                    2'd2: begin
                        capture_pending_mig <= 1'b0;
                        capture_pinned_mig <= 1'b0;
                        capture_cache_valid_mig <= 1'b0;
                        capture_word_valid <= 1'b0;
                        capture_ack_toggle <= capture_command_sync_mig[2];
                    end
                    2'd3: begin
                        capture_word_valid <= 1'b0;
                        if (!capture_pinned_mig ||
                            capture_word_index_mig2 >= FRAME_WORDS) begin
                            capture_error_mig <= 1'b1;
                            capture_ack_toggle <= capture_command_sync_mig[2];
                        end else if (capture_cache_valid_mig &&
                                     capture_cache_index_mig ==
                                     capture_word_index_mig2) begin
                            capture_word_valid <= 1'b1;
                            capture_ack_toggle <= capture_command_sync_mig[2];
                        end else begin
                            capture_pending_mig <= 1'b1;
                            capture_pending_index_mig <= capture_word_index_mig2;
                            capture_pending_toggle_mig <=
                                capture_command_sync_mig[2];
                        end
                    end
                    default: begin
                        capture_error_mig <= 1'b1;
                        capture_ack_toggle <= capture_command_sync_mig[2];
                    end
                endcase
            end

            if (scan_restart_accept_mig) begin
                last_service_valid_mig <= scan_valid_mig;
                last_service_epoch_mig <= scan_epoch_mig;
                last_service_command_count_mig <=
                    service_command_count_mig;
                last_service_response_count_mig <=
                    service_response_count_mig;
                last_service_max_command_wait_mig <=
                    service_max_command_wait_mig;
                last_service_max_response_latency_mig <=
                    service_max_response_latency_mig;
                last_service_min_resident_mig <=
                    service_min_resident_mig;
                last_service_max_outstanding_mig <=
                    service_max_outstanding_mig;
                last_service_response_fast_mig <=
                    service_response_fast_mig;
                last_service_response_mid_mig <=
                    service_response_mid_mig;
                last_service_response_slow_mig <=
                    service_response_slow_mig;
                service_command_wait_run_mig <= 16'd0;
                service_command_count_mig <= 16'd0;
                service_response_count_mig <= 16'd0;
                service_max_command_wait_mig <= 16'd0;
                service_max_response_latency_mig <= 16'd0;
                service_min_resident_mig <= 7'd63;
                service_max_outstanding_mig <= 3'd0;
                service_response_fast_mig <= 8'd0;
                service_response_mid_mig <= 8'd0;
                service_response_slow_mig <= 8'd0;
                scan_restart_seen_mig <= scan_restart_sync_mig[2];
                scan_epoch_gray_seen_mig <= scan_epoch_gray_sync_mig3;
                scan_epoch_mig <=
                    gray_to_binary8(scan_epoch_gray_sync_mig3);
                scan_index_mig <= 16'd0;
                scan_response_index_mig <= 16'd0;
                scan_bank_mig <= scan_bank_sync_mig2;
                scan_valid_mig <= scan_valid_sync_mig[1];
                scan_issue_active_mig <= scan_valid_sync_mig[1];
                scan_active_mig <= scan_valid_sync_mig[1];
                ownership_guard_mask_mig <= 3'b000;
                op_scan_discard_mig <= 1'b0;
            end else if (scan_restart_pending_mig) begin
                // Stop issuing the old epoch immediately. Its bounded return
                // set drains in order before the new bank/index is accepted.
                scan_issue_active_mig <= 1'b0;
                op_scan_discard_mig <= 1'b1;
            end

            if (!video_alive_mig) begin
                scan_issue_active_mig <= 1'b0;
                scan_active_mig <= 1'b0;
                scan_valid_mig <= 1'b0;
                // Queue a restart even if video reset returns its toggle to
                // the previously accepted parity before this engine becomes
                // idle.  A raw toggle edge is not a durable request across
                // the reset/relock interval; the inverted seen value is.
                scan_restart_seen_mig <= ~scan_restart_sync_mig[2];
                if (((mig_state == ST_READ_CMD) ||
                     (mig_state == ST_READ_WAIT)) && !op_is_capture)
                    op_scan_discard_mig <= 1'b1;
            end

            case ({scan_read_cmd_fire, scan_read_response_fire})
                2'b10: scan_read_outstanding <=
                    scan_read_outstanding + 3'd1;
                2'b01: scan_read_outstanding <=
                    scan_read_outstanding - 3'd1;
                default: scan_read_outstanding <= scan_read_outstanding;
            endcase

            if (scan_read_response_fire) begin
                mig_read_count <= mig_read_count + 32'd1;
                if (op_scan_discard_mig || !video_alive_mig ||
                    scan_restart_pending_mig) begin
                    if (scan_read_outstanding == 3'd1)
                        scan_active_mig <= 1'b0;
                end else if (scan_response_index_mig == FRAME_WORDS - 1) begin
                    scan_active_mig <= 1'b0;
                end else begin
                    scan_response_index_mig <=
                        scan_response_index_mig + 16'd1;
                end
            end

            case (mig_state)
                ST_IDLE: begin
                    write_cmd_accepted <= 1'b0;
                    write_data_accepted <= 1'b0;
                    // Prefetch the source-write head independently of which
                    // client wins this idle arbitration edge.  op_data is a
                    // write-only payload stage: clearing it for capture/scan
                    // reads needlessly made read-FIFO reset/availability
                    // drive all 512 payload clock enables at the MIG clock.
                    // A selected write necessarily has write_fifo_valid, so
                    // this same edge still captures its exact held head.
                    if (write_fifo_valid)
                        op_data <= write_fifo_out[511:0];
                    // Do not leave a completed urgent operation advertised
                    // if no successor can be selected.  When a successor is
                    // ready below, its own urgency overrides this default.
                    op_hi_pri <= 1'b0;
                    // The restart assignments above are nonblocking.  Give
                    // their acceptance this entire arbitration cycle so no
                    // operation can launch with the pre-restart bank/index,
                    // and do not pop a pending source write either.
                    if (scan_restart_pending_mig) begin
                        last_was_read <= 1'b0;
                    end else if (choose_capture) begin
                        op_is_write <= 1'b0;
                        op_is_capture <= 1'b1;
                        op_hi_pri <= 1'b0;
                        op_bank <= capture_bank_mig;
                        op_index <= capture_pending_index_mig;
                        op_last <= 1'b0;
                        mig_state <= ST_READ_CMD;
                        last_was_read <= 1'b1;
                    end else if (choose_read) begin
                        op_is_write <= 1'b0;
                        op_is_capture <= 1'b0;
                        op_hi_pri <= scan_urgent_mig;
                        op_bank <= scan_bank_mig;
                        op_index <= scan_index_mig;
                        op_last <= 1'b0;
                        op_scan_discard_mig <= 1'b0;
                        mig_state <= ST_READ_CMD;
                        last_was_read <= 1'b1;
                    end else if (choose_write) begin
                        op_is_write <= 1'b1;
                        op_is_capture <= 1'b0;
                        op_hi_pri <= write_urgent_mig;
                        op_bank <= write_fifo_out[WRITE_FIFO_WIDTH-2 -: 2];
                        op_index <= write_fifo_out[WRITE_FIFO_WIDTH-4 -: 16];
                        op_last <= write_fifo_out[WRITE_FIFO_WIDTH-1];
                        mig_state <= ST_WRITE;
                        last_was_read <= 1'b0;
                    end
                end

                ST_READ_CMD: begin
                    if (app_en && app_rdy) begin
                        if (op_is_capture) begin
                            mig_state <= ST_READ_WAIT;
                        end else begin
                            if (op_index == FRAME_WORDS - 1)
                                scan_issue_active_mig <= 1'b0;
                            else
                                scan_index_mig <= scan_index_mig + 16'd1;
                            mig_state <= ST_IDLE;
                        end
                    end
                end

                ST_READ_WAIT: begin
                    if (op_is_capture && app_rd_data_valid) begin
                        mig_read_count <= mig_read_count + 32'd1;
                        capture_word_data <= app_rd_data;
                        capture_word_valid <= 1'b1;
                        capture_cache_valid_mig <= 1'b1;
                        capture_cache_index_mig <= op_index;
                        capture_pending_mig <= 1'b0;
                        capture_error_mig <= 1'b0;
                        capture_ack_toggle <= capture_pending_toggle_mig;
                        mig_state <= ST_IDLE;
                    end
                end

                ST_WRITE: begin
                    if (write_cmd_fire)
                        write_cmd_accepted <= 1'b1;
                    if (write_data_fire)
                        write_data_accepted <= 1'b1;
                    if (write_complete) begin
                        mig_write_count <= mig_write_count + 32'd1;
                        mig_low_signature <= mig_low_signature_next;
                        mig_high_signature <= mig_high_signature_next;
                        if (op_last) begin
                            if (completed_valid_mig &&
                                (completed_bank_mig < 3))
                                ownership_guard_mask_mig <=
                                    ownership_guard_mask_mig |
                                    (3'b001 << completed_bank_mig);
                            completed_src_low_signature_mig <=
                                src_low_signature_sync2[op_bank];
                            completed_src_high_signature_mig <=
                                src_high_signature_sync2[op_bank];
                            completed_mig_low_signature <=
                                mig_low_signature_next;
                            completed_mig_high_signature <=
                                mig_high_signature_next;
                            completed_raw_low_signature_mig <=
                                src_raw_low_signature_sync2[op_bank];
                            completed_raw_high_signature_mig <=
                                src_raw_high_signature_sync2[op_bank];
                            completed_visual_stage_signatures_mig <=
                                src_visual_stage_signatures_sync2[op_bank];
                            completed_fetch_low_signature_mig <=
                                src_fetch_low_signature_sync2[op_bank];
                            completed_fetch_high_signature_mig <=
                                src_fetch_high_signature_sync2[op_bank];
                            completed_fetch_signature_generation_mig <=
                                src_fetch_signature_generation_sync2[op_bank];
                            completed_fetch_signature_valid_mig <=
                                src_fetch_signature_valid_sync2[op_bank];
                            completed_resolver_telemetry_mig <=
                                src_resolver_telemetry_sync2[op_bank];
                            completed_resolver_detail_mig <=
                                src_resolver_detail_sync2[op_bank];
                            completed_resolver_valid_mig <=
                                src_resolver_valid_sync2[op_bank];
                            payload_signature_valid <= 1'b1;
                            payload_signature_generation <=
                                payload_signature_generation + 8'd1;
                            completed_bank_mig <= op_bank;
                            completed_toggle_mig <= ~completed_toggle_mig;
                            completed_valid_mig <= 1'b1;
                            completed_frame_count_mig <=
                                completed_frame_count_mig + 16'd1;
                        end
                        mig_state <= ST_IDLE;
                    end
                end

                default: mig_state <= ST_IDLE;
            endcase
        end
    end

    assign capture_info = {
        8'h43, // "C"
        8'h01, // resolved-RGB capture protocol version
        H_ACTIVE[15:0],
        V_ACTIVE[15:0],
        FRAME_WORDS[15:0],
        capture_bank_mig,
        capture_pinned_mig,
        completed_valid_mig,
        capture_frame_mig,
        capture_cache_index_mig,
        capture_cache_valid_mig,
        capture_error_mig,
        26'd0
    };
endmodule

`default_nettype wire
