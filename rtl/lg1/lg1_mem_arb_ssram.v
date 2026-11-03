// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - LG1 clean memory arbiter skeleton

`default_nettype none

module lg1_mem_arb_ssram (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        rb_req_valid,
    output wire        rb_req_ready,
    input  wire        rb_req_write,
    input  wire [ 1:0] rb_req_plane,
    input  wire [17:0] rb_req_addr,
    input  wire [31:0] rb_req_wdata,
    input  wire [ 3:0] rb_req_wstrb,
    output wire        rb_rsp_valid,
    output wire [31:0] rb_rsp_rdata,

    input  wire        scan_req_valid,
    output wire        scan_req_ready,
    input  wire        scan_req_write,
    input  wire [ 1:0] scan_req_plane,
    input  wire [17:0] scan_req_addr,
    input  wire [31:0] scan_req_wdata,
    input  wire [ 3:0] scan_req_wstrb,
    output wire        scan_rsp_valid,
    output wire [31:0] scan_rsp_rdata,

    output wire        mem_req_valid,
    input  wire        mem_req_ready,
    output wire        mem_req_write,
    output wire [ 1:0] mem_req_plane,
    output wire [17:0] mem_req_addr,
    output wire [31:0] mem_req_wdata,
    output wire [ 3:0] mem_req_wstrb,
    input  wire        mem_rsp_valid,
    input  wire [31:0] mem_rsp_rdata
);
    localparam OWNER_DEPTH = 16;

    reg [OWNER_DEPTH-1:0] response_owner;
    reg [3:0] owner_wr_ptr;
    reg [3:0] owner_rd_ptr;
    reg [4:0] owner_count;

    wire response_fire = mem_rsp_valid && (owner_count != 0);
    // A retiring response frees its owner slot on this edge, so that slot may
    // accept a new read without inserting a bubble. The old head still routes
    // the current response while the new owner is appended at the tail.
    wire owner_space = (owner_count < OWNER_DEPTH) || response_fire;

    // Scanout owns the port only while it is actively filling a line. Its
    // maximum burst is 512 reads, after which RB receives the remaining line
    // budget. The capture-only back artifact is not physical scanout traffic.
    wire choose_scan = scan_req_valid &&
                       (scan_req_write || owner_space);
    wire selected_valid = choose_scan ? scan_req_valid : rb_req_valid;
    wire selected_write = choose_scan ? scan_req_write : rb_req_write;
    wire response_space = selected_write || owner_space;

    assign mem_req_valid = selected_valid && response_space;
    assign mem_req_write = choose_scan ? scan_req_write : rb_req_write;
    assign mem_req_plane = choose_scan ? scan_req_plane : rb_req_plane;
    assign mem_req_addr = choose_scan ? scan_req_addr : rb_req_addr;
    assign mem_req_wdata = choose_scan ? scan_req_wdata : rb_req_wdata;
    assign mem_req_wstrb = choose_scan ? scan_req_wstrb : rb_req_wstrb;

    assign scan_req_ready = choose_scan && response_space && mem_req_ready;
    assign rb_req_ready = !choose_scan && response_space && mem_req_ready;

    wire request_fire = mem_req_valid && mem_req_ready;
    wire read_fire = request_fire && !mem_req_write;
    wire head_scan = response_owner[owner_rd_ptr];

    assign rb_rsp_valid = response_fire && !head_scan;
    assign rb_rsp_rdata = mem_rsp_rdata;
    assign scan_rsp_valid = response_fire && head_scan;
    assign scan_rsp_rdata = mem_rsp_rdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            response_owner <= {OWNER_DEPTH{1'b0}};
            owner_wr_ptr <= 4'h0;
            owner_rd_ptr <= 4'h0;
            owner_count <= 5'h0;
        end else begin
            if (read_fire) begin
                response_owner[owner_wr_ptr] <= choose_scan;
                owner_wr_ptr <= owner_wr_ptr + 4'd1;
            end
            if (response_fire)
                owner_rd_ptr <= owner_rd_ptr + 4'd1;

            case ({read_fire, response_fire})
                2'b10: owner_count <= owner_count + 5'd1;
                2'b01: owner_count <= owner_count - 5'd1;
                default: owner_count <= owner_count;
            endcase
        end
    end
endmodule

`default_nettype wire
