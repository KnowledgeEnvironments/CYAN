// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - complete clean LG1 on a coherent 512-bit line channel
//
// This assembly leaves the clean graphics pipeline and its RB/scanout arbiter
// unchanged, then terminates the ordered 32-bit memory boundary in the
// coherent 64-byte cache. It is the local logical-memory integration target;
// the KCU105 MIG mapping and arbitration remain board-level work on Watson.

`default_nettype none

module lg1_clean_line_core #(
    parameter ENABLE_SCANOUT = 1,
    parameter ENABLE_PHYSICAL_FETCH = 1,
    parameter ENABLE_DIAGNOSTIC_SAMPLES = 0
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         serial_clk,
    input  wire         serial_rst_n,

    input  wire [ 14:0] addr,
    input  wire [ 63:0] wdata,
    output wire [ 63:0] rdata,
    input  wire [  7:0] wmask,
    input  wire         valid,
    input  wire         write,
    output wire         ready,

    input  wire         flush,
    output wire         flush_done,
    output wire         cache_dirty,
    input  wire         scanout_pause,

    output wire         line_valid,
    input  wire         line_ready,
    input  wire         line_write_done,
    output wire         line_write,
    output wire [ 15:0] line_addr,
    output wire [511:0] line_wdata,
    output wire [ 63:0] line_wstrb,
    input  wire         line_rsp_valid,
    output wire         line_rsp_ready,
    input  wire [511:0] line_rsp_rdata,

    output wire         scan_mem_valid,
    input  wire         scan_mem_ready,
    output wire [ 15:0] scan_mem_addr,
    input  wire         scan_mem_rsp_valid,
    output wire         scan_mem_rsp_ready,
    input  wire [511:0] scan_mem_rsp_rdata,

    output wire         video_de,
    output wire         video_hsync,
    output wire         video_vsync,
    output wire [  7:0] video_r,
    output wire [  7:0] video_g,
    output wire [  7:0] video_b,
    output wire         video_valid,
    output wire         video_frame_start,
    output wire [  9:0] video_x,
    output wire [  9:0] video_y,
    output wire [  4:0] video_did,
    output wire [  7:0] video_raw_index,
    output wire [  7:0] video_index,
    output wire [ 15:0] video_xmap_mode,
    output wire [  1:0] video_palette_bank,
    output wire [  2:0] video_resolver_class,
    output wire [ 91:0] resolver_telemetry,
    output wire [ 91:0] resolver_detail,
    output wire         resolver_frame_done,
    output wire [ 15:0] front_response_low_signature,
    output wire [ 15:0] front_response_high_signature,
    output wire [  7:0] front_response_signature_generation,
    output wire         front_response_signature_valid,
    output wire         scanout_underrun,
    output wire [ 31:0] scanout_underrun_count,
    output wire [ 31:0] scanout_protocol_error_count,
    output wire         irq,

    input  wire         scan_sample_valid,
    input  wire [  9:0] scan_sample_x,
    input  wire [  9:0] scan_sample_y,
    input  wire [  7:0] scan_sample_raw_index,
    input  wire [  7:0] scan_sample_raw_back_index,
    input  wire [  1:0] scan_sample_overlay,
    output wire         scan_sample_de,
    output wire [  4:0] scan_sample_did,
    output wire [ 15:0] scan_sample_mode,
    output wire [  1:0] scan_sample_palette_bank,
    output wire [  7:0] scan_sample_display_index,
    output wire [  7:0] scan_sample_r,
    output wire [  7:0] scan_sample_g,
    output wire [  7:0] scan_sample_b,

    output wire         busy
);
    wire         word_valid;
    wire         word_ready;
    wire         word_write;
    wire [  1:0] word_plane;
    wire [ 17:0] word_addr;
    wire [ 31:0] word_wdata;
    wire [  3:0] word_wstrb;
    wire         word_rsp_valid;
    wire [ 31:0] word_rsp_rdata;
    wire         scan_line_valid;
    wire         scan_line_ready;
    wire [ 15:0] scan_line_addr;
    wire         scan_line_rsp_valid;
    wire [511:0] scan_line_rsp_rdata;
    wire         cache_busy;

    lg1_clean_core #(
        .ENABLE_SCANOUT(ENABLE_SCANOUT),
        .ENABLE_PHYSICAL_FETCH(ENABLE_PHYSICAL_FETCH),
        .ENABLE_DIAGNOSTIC_SAMPLES(ENABLE_DIAGNOSTIC_SAMPLES)
    ) u_core (
        .clk(clk),
        .rst_n(rst_n),
        .serial_clk(serial_clk),
        .serial_rst_n(serial_rst_n),
        .addr(addr),
        .wdata(wdata),
        .rdata(rdata),
        .wmask(wmask),
        .valid(valid),
        .write(write),
        .ready(ready),
        .mem_req_valid(word_valid),
        .mem_req_ready(word_ready),
        .mem_req_write(word_write),
        .mem_req_plane(word_plane),
        .mem_req_addr(word_addr),
        .mem_req_wdata(word_wdata),
        .mem_req_wstrb(word_wstrb),
        .mem_rsp_valid(word_rsp_valid),
        .mem_rsp_rdata(word_rsp_rdata),
        .mem_busy(cache_busy),
        .scan_req_valid(scan_line_valid),
        .scan_req_ready(scan_line_ready),
        .scan_req_addr(scan_line_addr),
        .scan_rsp_valid(scan_line_rsp_valid),
        .scan_rsp_rdata(scan_line_rsp_rdata),
        .scanout_pause(scanout_pause),
        .video_de(video_de),
        .video_hsync(video_hsync),
        .video_vsync(video_vsync),
        .video_r(video_r),
        .video_g(video_g),
        .video_b(video_b),
        .video_valid(video_valid),
        .video_frame_start(video_frame_start),
        .video_x(video_x),
        .video_y(video_y),
        .video_did(video_did),
        .video_raw_index(video_raw_index),
        .video_index(video_index),
        .video_xmap_mode(video_xmap_mode),
        .video_palette_bank(video_palette_bank),
        .video_resolver_class(video_resolver_class),
        .resolver_telemetry(resolver_telemetry),
        .resolver_detail(resolver_detail),
        .resolver_frame_done(resolver_frame_done),
        .front_response_low_signature(front_response_low_signature),
        .front_response_high_signature(front_response_high_signature),
        .front_response_signature_generation(
            front_response_signature_generation),
        .front_response_signature_valid(front_response_signature_valid),
        .scanout_underrun(scanout_underrun),
        .scanout_underrun_count(scanout_underrun_count),
        .scanout_protocol_error_count(scanout_protocol_error_count),
        .irq(irq),
        .scan_sample_valid(scan_sample_valid),
        .scan_sample_x(scan_sample_x),
        .scan_sample_y(scan_sample_y),
        .scan_sample_raw_index(scan_sample_raw_index),
        .scan_sample_raw_back_index(scan_sample_raw_back_index),
        .scan_sample_overlay(scan_sample_overlay),
        .scan_sample_de(scan_sample_de),
        .scan_sample_did(scan_sample_did),
        .scan_sample_mode(scan_sample_mode),
        .scan_sample_palette_bank(scan_sample_palette_bank),
        .scan_sample_display_index(scan_sample_display_index),
        .scan_sample_r(scan_sample_r),
        .scan_sample_g(scan_sample_g),
        .scan_sample_b(scan_sample_b),
        .busy(busy)
    );

    lg1_line_cache_64b u_line_cache (
        .clk(clk),
        .rst_n(rst_n),
        .word_valid(word_valid),
        .word_ready(word_ready),
        .word_write(word_write),
        .word_plane(word_plane),
        .word_addr(word_addr),
        .word_wdata(word_wdata),
        .word_wstrb(word_wstrb),
        .word_rsp_valid(word_rsp_valid),
        .word_rsp_ready(1'b1),
        .word_rsp_rdata(word_rsp_rdata),
        .scan_valid(scan_line_valid),
        .scan_ready(scan_line_ready),
        .scan_addr(scan_line_addr),
        .scan_rsp_valid(scan_line_rsp_valid),
        .scan_rsp_ready(1'b1),
        .scan_rsp_rdata(scan_line_rsp_rdata),
        .flush(flush),
        .flush_done(flush_done),
        .line_valid(line_valid),
        .line_ready(line_ready),
        .line_write_done(line_write_done),
        .line_write(line_write),
        .line_addr(line_addr),
        .line_wdata(line_wdata),
        .line_wstrb(line_wstrb),
        .line_rsp_valid(line_rsp_valid),
        .line_rsp_ready(line_rsp_ready),
        .line_rsp_rdata(line_rsp_rdata),
        .scan_line_valid(scan_mem_valid),
        .scan_line_ready(scan_mem_ready),
        .scan_line_addr(scan_mem_addr),
        .scan_line_rsp_valid(scan_mem_rsp_valid),
        .scan_line_rsp_ready(scan_mem_rsp_ready),
        .scan_line_rsp_rdata(scan_mem_rsp_rdata),
        .dirty(cache_dirty),
        .busy(cache_busy)
    );
endmodule

`default_nettype wire
