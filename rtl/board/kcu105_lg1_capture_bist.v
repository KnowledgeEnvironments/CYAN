// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - deterministic LG1 board-capture workload
//
// A VIO toggle starts a small, board-only REX transaction sequencer.  It
// draws an indexed test card through the real LG1 host port, waits for each
// rectangle to retire, then flushes the coherent line cache so the front
// plane can be read back from DDR.  This is diagnostic infrastructure, not a
// replacement for guest-driven graphics.

`default_nettype none

module kcu105_lg1_capture_bist (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        trigger_toggle,

    output wire        bus_valid,
    output wire [14:0] bus_addr,
    output wire [63:0] bus_wdata,
    output wire [ 7:0] bus_wmask,
    input  wire        bus_ready,
    input  wire        lg1_busy,

    output wire        flush,
    input  wire        flush_done,
    output wire        active,
    output reg         done_toggle,
    output wire [31:0] status
);
    localparam [3:0] ST_IDLE       = 4'd0;
    localparam [3:0] ST_ISSUE      = 4'd1;
    localparam [3:0] ST_WAIT_START = 4'd2;
    localparam [3:0] ST_WAIT_DONE  = 4'd3;
    localparam [3:0] ST_FLUSH      = 4'd4;
    localparam [3:0] ST_COMPLETE   = 4'd5;
    localparam [3:0] ST_ERROR      = 4'd15;

    localparam [31:0] CMD_SOLID_BLOCK =
        (32'd3 << 28) | // logic source
        (32'd1 << 9)  | // stop on Y
        (32'd1 << 8)  | // stop on X
        (32'd1 << 5)  | // quad mode
        (32'd1 << 3)  | // block
        (32'd1 << 0);   // draw

    localparam [4:0] RECT_COUNT = 5'd14;
    localparam [23:0] TIMEOUT_LIMIT = 24'hffffff;

    reg [3:0] state;
    reg       trigger_seen;
    reg [4:0] rect_index;
    reg [2:0] op_index;
    reg [23:0] timeout_count;

    reg [9:0] rect_x0;
    reg [9:0] rect_x1;
    reg [9:0] rect_y0;
    reg [9:0] rect_y1;
    reg [7:0] rect_color;

    always @* begin
        rect_x0 = 10'd0;
        rect_x1 = 10'd1023;
        rect_y0 = 10'd0;
        rect_y1 = 10'd767;
        rect_color = 8'h10;
        case (rect_index)
            5'd0:  begin rect_x0 = 10'd0;   rect_x1 = 10'd127;  rect_color = 8'h10; end
            5'd1:  begin rect_x0 = 10'd128; rect_x1 = 10'd255;  rect_color = 8'h30; end
            5'd2:  begin rect_x0 = 10'd256; rect_x1 = 10'd383;  rect_color = 8'h50; end
            5'd3:  begin rect_x0 = 10'd384; rect_x1 = 10'd511;  rect_color = 8'h70; end
            5'd4:  begin rect_x0 = 10'd512; rect_x1 = 10'd639;  rect_color = 8'h90; end
            5'd5:  begin rect_x0 = 10'd640; rect_x1 = 10'd767;  rect_color = 8'hb0; end
            5'd6:  begin rect_x0 = 10'd768; rect_x1 = 10'd895;  rect_color = 8'hd0; end
            5'd7:  begin rect_x0 = 10'd896; rect_x1 = 10'd1023; rect_color = 8'hf0; end
            5'd8:  begin rect_y0 = 10'd0;   rect_y1 = 10'd15;   rect_color = 8'hff; end
            5'd9:  begin rect_y0 = 10'd752; rect_y1 = 10'd767;  rect_color = 8'hff; end
            5'd10: begin rect_x0 = 10'd0;   rect_x1 = 10'd15;   rect_color = 8'hff; end
            5'd11: begin rect_x0 = 10'd1008; rect_x1 = 10'd1023; rect_color = 8'hff; end
            5'd12: begin
                rect_x0 = 10'd384; rect_x1 = 10'd639;
                rect_y0 = 10'd256; rect_y1 = 10'd511;
                rect_color = 8'h08;
            end
            5'd13: begin
                rect_x0 = 10'd416; rect_x1 = 10'd607;
                rect_y0 = 10'd288; rect_y1 = 10'd479;
                rect_color = 8'hf8;
            end
            default: begin end
        endcase
    end

    reg [14:0] issue_addr;
    reg [63:0] issue_wdata;
    reg [7:0]  issue_wmask;
    always @* begin
        issue_addr = 15'h0000;
        issue_wdata = 64'd0;
        issue_wmask = 8'h0f;
        case (op_index)
            3'd0: begin // AUX2: select pixel planes
                issue_addr = 15'h47a8;
                issue_wdata = 64'h00000000_20000000;
            end
            3'd1: begin // COLORREDI/COLOR0: indexed source color
                issue_addr = 15'h0038;
                issue_wdata = {56'd0, rect_color};
            end
            3'd2: begin
                issue_addr = 15'h000c;
                issue_wdata = {54'd0, rect_x0};
            end
            3'd3: begin
                issue_addr = 15'h0084;
                issue_wdata = {54'd0, rect_x1};
            end
            3'd4: begin
                issue_addr = 15'h001c;
                issue_wdata = {54'd0, rect_y0};
            end
            3'd5: begin
                issue_addr = 15'h0088;
                issue_wdata = {54'd0, rect_y1};
            end
            default: begin // GO|COMMAND uses the upper 32-bit lane
                issue_addr = 15'h0800;
                issue_wdata = {CMD_SOLID_BLOCK, 32'd0};
                issue_wmask = 8'hf0;
            end
        endcase
    end

    assign active = (state != ST_IDLE) && (state != ST_COMPLETE) &&
                    (state != ST_ERROR);
    assign bus_valid = (state == ST_ISSUE);
    assign bus_addr = issue_addr;
    assign bus_wdata = issue_wdata;
    assign bus_wmask = issue_wmask;
    assign flush = (state == ST_FLUSH);
    assign status = {
        8'hb1,
        done_toggle,
        trigger_seen,
        active,
        (state == ST_ERROR),
        state,
        rect_index,
        op_index,
        timeout_count[7:0]
    };

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            trigger_seen <= 1'b0;
            done_toggle <= 1'b0;
            rect_index <= 5'd0;
            op_index <= 3'd0;
            timeout_count <= 24'd0;
        end else begin
            if (state != ST_IDLE && state != ST_COMPLETE &&
                state != ST_ERROR) begin
                if (timeout_count == TIMEOUT_LIMIT)
                    state <= ST_ERROR;
                else
                    timeout_count <= timeout_count + 24'd1;
            end

            case (state)
                ST_IDLE, ST_COMPLETE: begin
                    timeout_count <= 24'd0;
                    if (trigger_toggle != trigger_seen) begin
                        trigger_seen <= trigger_toggle;
                        rect_index <= 5'd0;
                        op_index <= 3'd0;
                        state <= ST_ISSUE;
                    end
                end

                ST_ISSUE: begin
                    if (bus_ready) begin
                        timeout_count <= 24'd0;
                        if (op_index == 3'd6) begin
                            op_index <= 3'd0;
                            state <= ST_WAIT_START;
                        end else begin
                            op_index <= op_index + 3'd1;
                        end
                    end
                end

                ST_WAIT_START: begin
                    if (lg1_busy) begin
                        timeout_count <= 24'd0;
                        state <= ST_WAIT_DONE;
                    end
                end

                ST_WAIT_DONE: begin
                    if (!lg1_busy) begin
                        timeout_count <= 24'd0;
                        if (rect_index == RECT_COUNT - 5'd1)
                            state <= ST_FLUSH;
                        else begin
                            rect_index <= rect_index + 5'd1;
                            op_index <= 3'd0;
                            state <= ST_ISSUE;
                        end
                    end
                end

                ST_FLUSH: begin
                    if (flush_done) begin
                        timeout_count <= 24'd0;
                        done_toggle <= trigger_seen;
                        state <= ST_COMPLETE;
                    end
                end

                ST_ERROR: begin
                    if (trigger_toggle != trigger_seen) begin
                        trigger_seen <= trigger_toggle;
                        rect_index <= 5'd0;
                        op_index <= 3'd0;
                        timeout_count <= 24'd0;
                        state <= ST_ISSUE;
                    end
                end

                default: state <= ST_ERROR;
            endcase
        end
    end
endmodule

`default_nettype wire
