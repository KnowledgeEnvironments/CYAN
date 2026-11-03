// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - passive accepted-LG1 event encoder.
//
// This observer has no path into ready/backpressure or any functional LG1
// signal. It normalizes every accepted public LG1 access for downstream
// observer filtering and continuously follows CONFIGSEL so marker records can
// describe the selector at their exact boundary.

`default_nettype none

module cyan_lg1_config_event_encoder (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        accepted,
    input  wire [14:0] addr,
    input  wire [63:0] wdata,
    input  wire [ 7:0] wmask,
    input  wire [63:0] rdata,
    input  wire        write,
    input  wire [ 1:0] source,
    input  wire [31:0] cpu_dmem_pc,
    output wire        access_valid,
    output wire [87:0] access_payload,
    output wire [ 2:0] current_configsel
);
    localparam [10:0] CFG_CONFIGSEL = 11'h7ec;

    // Match lg1_clean_core.bus_wd_byte exactly. Configuration registers are
    // byte-wide and the CPU may address any architectural byte lane.
    function [7:0] bus_wd_byte;
        input [63:0] data;
        input [ 7:0] mask;
        reg [31:0] word;
        begin
            word = (mask[7] || mask[6] || mask[5] || mask[4]) ?
                   data[63:32] : data[31:0];
            bus_wd_byte =
                (mask == 8'h80) ? data[63:56] :
                (mask == 8'h40) ? data[55:48] :
                (mask == 8'h20) ? data[47:40] :
                (mask == 8'h10) ? data[39:32] :
                (mask == 8'h08) ? data[31:24] :
                (mask == 8'h04) ? data[23:16] :
                (mask == 8'h02) ? data[15:8]  :
                (mask == 8'h01) ? data[7:0]   :
                                  word[7:0];
        end
    endfunction

    wire        config_region = (addr[14:12] == 3'b100);
    wire [10:0] cfg_offset = {addr[10:2], 2'b00};
    wire        upper_word = |wmask[7:4];
    wire [31:0] normalized_word = write ?
        (upper_word ? wdata[63:32] : wdata[31:0]) : rdata[31:0];
    wire [ 3:0] normalized_mask = write ?
        (upper_word ? wmask[7:4] : wmask[3:0]) : 4'd0;
    wire [ 7:0] config_write_byte = bus_wd_byte(wdata, wmask);
    wire [31:0] normalized_pc = (source == 2'd0) ? cpu_dmem_pc :
                                                          32'd0;
    reg [2:0] configsel;

    assign access_valid = accepted;
    // This is record bits [95:8].  The ring appends an 8-bit cycle delta.
    assign access_payload = {
        2'b00, normalized_pc, addr, normalized_word,
        normalized_mask, source, write
    };
    assign current_configsel = configsel;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            configsel <= 3'd0;
        else if (access_valid && config_region && write &&
                 (cfg_offset == CFG_CONFIGSEL))
            configsel <= config_write_byte[2:0];
    end
endmodule

`default_nettype wire
