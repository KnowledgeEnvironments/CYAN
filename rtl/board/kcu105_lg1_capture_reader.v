// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - VIO-driven coherent LG1 framebuffer reader
//
// Arbitrates the normal LG1 coherent line client and a diagnostic read-only
// client onto the existing KCU105 DDR line bridge.  A toggle requests one
// sequential 64-byte front-plane line.  The full returned line is retained
// for VIO extraction.  This module observes DDR after the real LG1 cache has
// been flushed; it does not bypass or emulate the draw pipeline.

`default_nettype none

module kcu105_lg1_capture_reader (
    input  wire         clk,
    input  wire         rst_n,
    // The ownership wrapper and DDR bridge remain alive across a guest/debug
    // core reset.  Abandoned core traffic is drained below so it cannot
    // strand this single-owner interface.
    input  wire         core_rst_n,
    input  wire         hold,
    input  wire         reset_toggle,
    input  wire         next_toggle,

    input  wire         core_valid,
    output wire         core_ready,
    output wire         core_write_done,
    input  wire         core_write,
    input  wire [ 15:0] core_addr,
    input  wire [511:0] core_wdata,
    input  wire [ 63:0] core_wstrb,
    output wire         core_rsp_valid,
    input  wire         core_rsp_ready,
    output wire [511:0] core_rsp_rdata,

    output wire         mem_valid,
    input  wire         mem_ready,
    input  wire         mem_write_done,
    output wire         mem_write,
    output wire [ 15:0] mem_addr,
    output wire [511:0] mem_wdata,
    output wire [ 63:0] mem_wstrb,
    input  wire         mem_rsp_valid,
    output wire         mem_rsp_ready,
    input  wire [511:0] mem_rsp_rdata,

    output reg  [511:0] capture_data,
    output wire [127:0] capture_status
);
    reg        owner_valid;
    reg        owner_debug;
    reg        owner_discard;
    reg        core_write_pending;
    reg        core_write_discard;
    reg        reset_seen;
    reg        next_seen;
    reg [15:0] capture_index;
    reg [15:0] last_line_addr;
    reg        data_valid;
    reg        protocol_error;
    reg [23:0] response_count;

    wire reset_pending = reset_toggle != reset_seen;
    wire debug_pending = hold && !reset_pending &&
                         (next_toggle != next_seen);
    wire choose_debug = !owner_valid && !core_write_pending && debug_pending;
    wire choose_core = core_rst_n && !owner_valid &&
                       !core_write_pending && !choose_debug;
    wire mem_fire = mem_valid && mem_ready;
    wire rsp_fire = mem_rsp_valid && mem_rsp_ready;

    assign mem_valid = !owner_valid && !core_write_pending &&
                       (choose_debug ? 1'b1 :
                        (choose_core && core_valid));
    assign mem_write = choose_debug ? 1'b0 : core_write;
    assign mem_addr = choose_debug ? {2'b00, capture_index[13:0]} :
                                      core_addr;
    assign mem_wdata = choose_debug ? 512'd0 : core_wdata;
    assign mem_wstrb = choose_debug ? 64'd0 : core_wstrb;

    assign core_ready = choose_core && mem_ready;
    assign core_write_done = core_rst_n && core_write_pending &&
                             !core_write_discard && mem_write_done;
    assign core_rsp_valid = core_rst_n && owner_valid && !owner_debug &&
                            !owner_discard && mem_rsp_valid;
    assign core_rsp_rdata = mem_rsp_rdata;
    assign mem_rsp_ready = owner_valid &&
                           ((owner_debug || owner_discard || !core_rst_n) ?
                            1'b1 : core_rsp_ready);

    assign capture_status = {
        32'h43415030, // "CAP0"
        capture_index,
        last_line_addr,
        hold,
        owner_valid,
        data_valid,
        protocol_error,
        next_seen,
        next_toggle,
        reset_seen,
        reset_toggle,
        response_count,
        32'd0
    };

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            owner_valid <= 1'b0;
            owner_debug <= 1'b0;
            owner_discard <= 1'b0;
            core_write_pending <= 1'b0;
            core_write_discard <= 1'b0;
            reset_seen <= 1'b0;
            next_seen <= 1'b0;
            capture_index <= 16'd0;
            last_line_addr <= 16'd0;
            data_valid <= 1'b0;
            protocol_error <= 1'b0;
            response_count <= 24'd0;
            capture_data <= 512'd0;
        end else begin
            if (reset_pending && !owner_valid) begin
                reset_seen <= reset_toggle;
                next_seen <= next_toggle;
                capture_index <= 16'd0;
                last_line_addr <= 16'd0;
                data_valid <= 1'b0;
                protocol_error <= 1'b0;
                response_count <= 24'd0;
            end

            if (mem_fire && !mem_write) begin
                owner_valid <= 1'b1;
                owner_debug <= choose_debug;
                owner_discard <= 1'b0;
                if (choose_debug)
                    last_line_addr <= {2'b00, capture_index[13:0]};
            end

            if (mem_fire && mem_write) begin
                core_write_pending <= 1'b1;
                core_write_discard <= 1'b0;
            end

            // A guest/debug reset does not reset the ownership wrapper or
            // the CDC bridge.  Mark any already-accepted core transaction as
            // abandoned, then consume its eventual completion locally.  This
            // is the only safe way to preserve the one-request-at-a-time DDR
            // contract while allowing the core itself to reset immediately.
            if (!core_rst_n && owner_valid && !owner_debug)
                owner_discard <= 1'b1;
            if (!core_rst_n && core_write_pending)
                core_write_discard <= 1'b1;

            if (core_write_pending && mem_write_done) begin
                core_write_pending <= 1'b0;
                core_write_discard <= 1'b0;
            end

            if (rsp_fire) begin
                owner_valid <= 1'b0;
                owner_discard <= 1'b0;
                if (owner_debug) begin
                    capture_data <= mem_rsp_rdata;
                    capture_index <= capture_index + 16'd1;
                    next_seen <= next_toggle;
                    data_valid <= 1'b1;
                    response_count <= response_count + 24'd1;
                end
            end

            if (mem_rsp_valid && !owner_valid)
                protocol_error <= 1'b1;
            if (mem_write_done && !core_write_pending)
                protocol_error <= 1'b1;
        end
    end
endmodule

`default_nettype wire
