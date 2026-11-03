// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - ordered LG1 clients to coherent 64-byte line channel
//
// Preserves the clean core's RB and scanout word interfaces while replacing
// the external 32-bit memory boundary with the coherent 512-bit line cache.
// The existing owner-tracking arbiter routes read responses; the cache
// serializes the downstream channel for this correctness-first implementation.

`default_nettype none

module lg1_mem_line_adapter_64b (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         rb_req_valid,
    output wire         rb_req_ready,
    input  wire         rb_req_write,
    input  wire [  1:0] rb_req_plane,
    input  wire [ 17:0] rb_req_addr,
    input  wire [ 31:0] rb_req_wdata,
    input  wire [  3:0] rb_req_wstrb,
    output wire         rb_rsp_valid,
    output wire [ 31:0] rb_rsp_rdata,

    input  wire         scan_req_valid,
    output wire         scan_req_ready,
    input  wire         scan_req_write,
    input  wire [  1:0] scan_req_plane,
    input  wire [ 17:0] scan_req_addr,
    input  wire [ 31:0] scan_req_wdata,
    input  wire [  3:0] scan_req_wstrb,
    output wire         scan_rsp_valid,
    output wire [ 31:0] scan_rsp_rdata,

    input  wire         flush,
    output wire         flush_done,
    output wire         dirty,
    output wire         busy,

    output wire         line_valid,
    input  wire         line_ready,
    input  wire         line_write_done,
    output wire         line_write,
    output wire [ 15:0] line_addr,
    output wire [511:0] line_wdata,
    output wire [ 63:0] line_wstrb,
    input  wire         line_rsp_valid,
    output wire         line_rsp_ready,
    input  wire [511:0] line_rsp_rdata
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

    lg1_mem_arb_ssram u_client_arb (
        .clk(clk),
        .rst_n(rst_n),
        .rb_req_valid(rb_req_valid),
        .rb_req_ready(rb_req_ready),
        .rb_req_write(rb_req_write),
        .rb_req_plane(rb_req_plane),
        .rb_req_addr(rb_req_addr),
        .rb_req_wdata(rb_req_wdata),
        .rb_req_wstrb(rb_req_wstrb),
        .rb_rsp_valid(rb_rsp_valid),
        .rb_rsp_rdata(rb_rsp_rdata),
        .scan_req_valid(scan_req_valid),
        .scan_req_ready(scan_req_ready),
        .scan_req_write(scan_req_write),
        .scan_req_plane(scan_req_plane),
        .scan_req_addr(scan_req_addr),
        .scan_req_wdata(scan_req_wdata),
        .scan_req_wstrb(scan_req_wstrb),
        .scan_rsp_valid(scan_rsp_valid),
        .scan_rsp_rdata(scan_rsp_rdata),
        .mem_req_valid(word_valid),
        .mem_req_ready(word_ready),
        .mem_req_write(word_write),
        .mem_req_plane(word_plane),
        .mem_req_addr(word_addr),
        .mem_req_wdata(word_wdata),
        .mem_req_wstrb(word_wstrb),
        .mem_rsp_valid(word_rsp_valid),
        .mem_rsp_rdata(word_rsp_rdata)
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
        .scan_valid(1'b0),
        .scan_ready(),
        .scan_addr(16'h0),
        .scan_rsp_valid(),
        .scan_rsp_ready(1'b1),
        .scan_rsp_rdata(),
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
        .scan_line_valid(),
        .scan_line_ready(1'b0),
        .scan_line_addr(),
        .scan_line_rsp_valid(1'b0),
        .scan_line_rsp_ready(),
        .scan_line_rsp_rdata(512'd0),
        .dirty(dirty),
        .busy(busy)
    );
endmodule

`default_nettype wire
