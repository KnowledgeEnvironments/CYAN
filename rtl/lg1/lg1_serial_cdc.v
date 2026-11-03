// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// Dedicated clock-domain boundaries for the Indigo LG1 serial section.
//
// Configuration is an ordered, one-outstanding bundled-data transaction.
// The machine side holds its payload until the destination has applied one
// cfg_valid pulse, captured the resulting read latch, and returned the exact
// request toggle.  Retrace is an event toggle because its minimum spacing is
// one complete video frame.

`default_nettype none

module lg1_config_cdc_bridge (
    input  wire        src_clk,
    input  wire        src_rst_n,
    input  wire        src_valid,
    input  wire        src_write,
    input  wire        src_go,
    input  wire [10:0] src_offset,
    input  wire [ 2:0] src_sel,
    input  wire [ 7:0] src_wbyte,
    output reg  [ 7:0] src_rbyte,
    output reg         src_ready,

    input  wire        dst_clk,
    input  wire        dst_rst_n,
    output reg         dst_valid,
    output reg         dst_write,
    output reg         dst_go,
    output reg  [10:0] dst_offset,
    output reg  [ 2:0] dst_sel,
    output wire [ 7:0] dst_wbyte,
    input  wire [ 7:0] dst_rbyte
);
    reg        src_req_toggle;
    reg        src_busy;
    reg        src_write_q;
    reg        src_go_q;
    reg [10:0] src_offset_q;
    reg [ 2:0] src_sel_q;
    reg [ 7:0] src_wbyte_q;

    reg        dst_req_seen;
    reg        dst_active;
    reg        dst_response_wait;
    reg        dst_ack_toggle;
    reg [ 7:0] dst_response_q;

    // This is the single physical capture bank for the asynchronous held
    // write-byte payload.  The serial consumers may fan out from its Q pins,
    // but synthesis must not create additional asynchronous samplers.
    (* DONT_TOUCH = "TRUE" *) reg [7:0] dst_wbyte_q;
    assign dst_wbyte = dst_wbyte_q;

    (* ASYNC_REG = "TRUE" *) reg dst_req_sync1;
    (* ASYNC_REG = "TRUE" *) reg dst_req_sync2;
    (* ASYNC_REG = "TRUE" *) reg src_ack_sync1;
    (* ASYNC_REG = "TRUE" *) reg src_ack_sync2;

    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            src_req_toggle <= 1'b0;
            src_busy <= 1'b0;
            src_write_q <= 1'b0;
            src_go_q <= 1'b0;
            src_offset_q <= 11'd0;
            src_sel_q <= 3'd0;
            src_wbyte_q <= 8'd0;
            src_ack_sync1 <= 1'b0;
            src_ack_sync2 <= 1'b0;
            src_rbyte <= 8'd0;
            src_ready <= 1'b0;
        end else begin
            src_ack_sync1 <= dst_ack_toggle;
            src_ack_sync2 <= src_ack_sync1;
            src_ready <= 1'b0;
            if (src_busy) begin
                if (src_ack_sync2 == src_req_toggle) begin
                    src_rbyte <= dst_response_q;
                    src_ready <= 1'b1;
                    src_busy <= 1'b0;
                end
            end else if (src_valid && !src_ready) begin
                src_write_q <= src_write;
                src_go_q <= src_go;
                src_offset_q <= src_offset;
                src_sel_q <= src_sel;
                src_wbyte_q <= src_wbyte;
                src_req_toggle <= ~src_req_toggle;
                src_busy <= 1'b1;
            end
        end
    end

    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            dst_req_sync1 <= 1'b0;
            dst_req_sync2 <= 1'b0;
            dst_req_seen <= 1'b0;
            dst_active <= 1'b0;
            dst_response_wait <= 1'b0;
            dst_ack_toggle <= 1'b0;
            dst_response_q <= 8'd0;
            dst_valid <= 1'b0;
            dst_write <= 1'b0;
            dst_go <= 1'b0;
            dst_offset <= 11'd0;
            dst_sel <= 3'd0;
            dst_wbyte_q <= 8'd0;
        end else begin
            dst_req_sync1 <= src_req_toggle;
            dst_req_sync2 <= dst_req_sync1;
            dst_valid <= 1'b0;
            if (!dst_active && (dst_req_sync2 != dst_req_seen)) begin
                // The bundled payload has been stable since before the first
                // request synchronizer stage.  Retain it through the sole
                // destination pulse and the read-latch response cycle.
                dst_req_seen <= dst_req_sync2;
                dst_write <= src_write_q;
                dst_go <= src_go_q;
                dst_offset <= src_offset_q;
                dst_sel <= src_sel_q;
                dst_wbyte_q <= src_wbyte_q;
                dst_valid <= 1'b1;
                dst_active <= 1'b1;
                dst_response_wait <= 1'b0;
            end else if (dst_active && !dst_response_wait) begin
                // dst_valid was observed by the serial peripheral on this
                // edge.  Wait one complete destination cycle so its
                // registered cfg_rbyte is available.
                dst_response_wait <= 1'b1;
            end else if (dst_active) begin
                dst_response_q <= dst_rbyte;
                dst_ack_toggle <= dst_req_seen;
                dst_active <= 1'b0;
                dst_response_wait <= 1'b0;
            end
        end
    end
endmodule

module lg1_event_cdc_bridge (
    input  wire src_clk,
    input  wire src_rst_n,
    input  wire src_pulse,
    input  wire dst_clk,
    input  wire dst_rst_n,
    output reg  dst_pulse
);
    reg src_toggle;
    reg dst_seen;
    (* ASYNC_REG = "TRUE" *) reg dst_sync1;
    (* ASYNC_REG = "TRUE" *) reg dst_sync2;

    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n)
            src_toggle <= 1'b0;
        else if (src_pulse)
            src_toggle <= ~src_toggle;
    end

    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            dst_sync1 <= 1'b0;
            dst_sync2 <= 1'b0;
            dst_seen <= 1'b0;
            dst_pulse <= 1'b0;
        end else begin
            dst_sync1 <= src_toggle;
            dst_sync2 <= dst_sync1;
            dst_pulse <= dst_sync2 != dst_seen;
            if (dst_sync2 != dst_seen)
                dst_seen <= dst_sync2;
        end
    end
endmodule

`default_nettype wire
