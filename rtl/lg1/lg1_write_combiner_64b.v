// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - two-entry 64-byte LG1 write combiner
//
// Converts accepted four-byte CI8 words into byte-strobed 512-bit logical
// lines. This module is deliberately below LG1/RB semantics: plane selection,
// clipping, logic operations, and copy ordering are resolved before entry.

`default_nettype none

module lg1_write_combiner_64b (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         in_valid,
    output wire         in_ready,
    input  wire [  1:0] in_plane,
    input  wire [ 17:0] in_word_addr,
    input  wire [ 31:0] in_wdata,
    input  wire [  3:0] in_wstrb,

    input  wire         flush,
    output reg          flush_done,

    output reg          line_valid,
    input  wire         line_ready,
    output reg  [ 15:0] line_addr,
    output reg  [511:0] line_wdata,
    output reg  [ 63:0] line_wstrb,
    output wire         busy
);
    wire [15:0] in_line_addr = {in_plane, in_word_addr[17:4]};

    reg          way0_valid;
    reg          way1_valid;
    reg [ 15:0] way0_addr;
    reg [ 15:0] way1_addr;
    reg [511:0] way0_data;
    reg [511:0] way1_data;
    reg [ 63:0] way0_strb;
    reg [ 63:0] way1_strb;
    reg          lru_way;
    reg          emit_way;
    reg          flushing;

    wire hit0 = way0_valid && (way0_addr == in_line_addr);
    wire hit1 = way1_valid && (way1_addr == in_line_addr);
    wire free0 = !way0_valid;
    wire free1 = !way1_valid;
    wire have_slot = hit0 || hit1 || free0 || free1;

    assign in_ready = !flush && !flushing && !line_valid && have_slot;
    assign busy = way0_valid || way1_valid || line_valid || flushing;
    function [511:0] merge_data;
        input [511:0] old_data;
        input [  3:0] word_index;
        input [ 31:0] new_data;
        input [  3:0] byte_strobe;
        integer i;
        reg [511:0] value;
        begin
            value = old_data;
            for (i = 0; i < 4; i = i + 1)
                if (byte_strobe[i])
                    value[511 - (word_index * 32) - (i * 8) -: 8] =
                        new_data[31 - (i * 8) -: 8];
            merge_data = value;
        end
    endfunction

    function [63:0] merge_strobe;
        input [63:0] old_strobe;
        input [ 3:0] word_index;
        input [ 3:0] byte_strobe;
        integer i;
        reg [63:0] value;
        begin
            value = old_strobe;
            for (i = 0; i < 4; i = i + 1)
                if (byte_strobe[i])
                    value[63 - (word_index * 4) - i] = 1'b1;
            merge_strobe = value;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            way0_valid <= 1'b0;
            way1_valid <= 1'b0;
            way0_addr <= 16'h0;
            way1_addr <= 16'h0;
            way0_data <= 512'h0;
            way1_data <= 512'h0;
            way0_strb <= 64'h0;
            way1_strb <= 64'h0;
            lru_way <= 1'b0;
            emit_way <= 1'b0;
            flushing <= 1'b0;
            flush_done <= 1'b0;
            line_valid <= 1'b0;
            line_addr <= 16'h0;
            line_wdata <= 512'h0;
            line_wstrb <= 64'h0;
        end else begin
            flush_done <= 1'b0;

            if (line_valid) begin
                if (flush)
                    flushing <= 1'b1;
                if (line_ready) begin
                    line_valid <= 1'b0;
                    if (emit_way)
                        way1_valid <= 1'b0;
                    else
                        way0_valid <= 1'b0;
                end
            end else if (flushing) begin
                if (way0_valid) begin
                    line_valid <= 1'b1;
                    line_addr <= way0_addr;
                    line_wdata <= way0_data;
                    line_wstrb <= way0_strb;
                    emit_way <= 1'b0;
                end else if (way1_valid) begin
                    line_valid <= 1'b1;
                    line_addr <= way1_addr;
                    line_wdata <= way1_data;
                    line_wstrb <= way1_strb;
                    emit_way <= 1'b1;
                end else begin
                    flushing <= 1'b0;
                    flush_done <= 1'b1;
                end
            end else if (flush) begin
                flushing <= 1'b1;
            end else if (in_valid) begin
                if (hit0) begin
                    way0_data <= merge_data(way0_data, in_word_addr[3:0],
                                            in_wdata, in_wstrb);
                    way0_strb <= merge_strobe(way0_strb,
                                              in_word_addr[3:0], in_wstrb);
                    lru_way <= 1'b1;
                end else if (hit1) begin
                    way1_data <= merge_data(way1_data, in_word_addr[3:0],
                                            in_wdata, in_wstrb);
                    way1_strb <= merge_strobe(way1_strb,
                                              in_word_addr[3:0], in_wstrb);
                    lru_way <= 1'b0;
                end else if (free0) begin
                    way0_valid <= 1'b1;
                    way0_addr <= in_line_addr;
                    way0_data <= merge_data(512'h0, in_word_addr[3:0],
                                            in_wdata, in_wstrb);
                    way0_strb <= merge_strobe(64'h0, in_word_addr[3:0],
                                              in_wstrb);
                    lru_way <= 1'b1;
                end else if (free1) begin
                    way1_valid <= 1'b1;
                    way1_addr <= in_line_addr;
                    way1_data <= merge_data(512'h0, in_word_addr[3:0],
                                            in_wdata, in_wstrb);
                    way1_strb <= merge_strobe(64'h0, in_word_addr[3:0],
                                              in_wstrb);
                    lru_way <= 1'b0;
                end else begin
                    // A full miss is not accepted. Emit the LRU line and rely
                    // on ready/valid to keep the input stable until a way is
                    // free on a later cycle.
                    line_valid <= 1'b1;
                    emit_way <= lru_way;
                    if (lru_way) begin
                        line_addr <= way1_addr;
                        line_wdata <= way1_data;
                        line_wstrb <= way1_strb;
                    end else begin
                        line_addr <= way0_addr;
                        line_wdata <= way0_data;
                        line_wstrb <= way0_strb;
                    end
                end
            end
        end
    end
endmodule

`default_nettype wire
