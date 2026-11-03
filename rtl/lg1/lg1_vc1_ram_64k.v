// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - VC1 64 KiB synchronous dual-port SRAM

`default_nettype none

module lg1_vc1_ram_64k (
    input  wire        clk,

    input  wire        cfg_en,
    input  wire        cfg_write,
    input  wire [15:0] cfg_addr,
    input  wire [15:0] cfg_wdata,
    output reg  [ 7:0] cfg_rdata,

    input  wire        scan_en,
    input  wire [15:0] scan_addr,
    output reg  [31:0] scan_rdata
);
    // No reset or initialization loop: software initializes VC1 SRAM and the
    // surrounding controller resets only validity metadata.  Two independent
    // clocked ports give Vivado the canonical true-dual-port BRAM template.
    (* ram_style = "block" *) reg [31:0] mem [0:16383];
    reg [31:0] cfg_read_word;
    reg [ 1:0] cfg_read_lane;

    always @(*) begin
        case (cfg_read_lane)
            2'd0: cfg_rdata = cfg_read_word[31:24];
            2'd1: cfg_rdata = cfg_read_word[23:16];
            2'd2: cfg_rdata = cfg_read_word[15:8];
            default: cfg_rdata = cfg_read_word[7:0];
        endcase
    end

    always @(posedge clk) begin
        if (cfg_en) begin
            if (cfg_write) begin
                // The host controller commits a complete big-endian pair
                // on its odd-byte access. Both memory bytes commit on one
                // edge; a simultaneous scan read retains read-before-write
                // behavior rather than exposing a partially updated pair.
                if (cfg_addr[1])
                    mem[cfg_addr[15:2]][15:0] <= cfg_wdata;
                else
                    mem[cfg_addr[15:2]][31:16] <= cfg_wdata;
            end
            cfg_read_word <= mem[cfg_addr[15:2]];
            cfg_read_lane <= cfg_addr[1:0];
        end
    end

    always @(posedge clk) begin
        if (scan_en)
            scan_rdata <= mem[scan_addr[15:2]];
    end
endmodule

`default_nettype wire
