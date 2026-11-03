// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - small synchronous byte RAM wrapper.

`default_nettype none

module cyan_sync_byte_ram #(
    parameter integer ADDR_W = 11
) (
    input  wire              clk,
    input  wire              we,
    input  wire [ADDR_W-1:0] waddr,
    input  wire [7:0]        wdata,
    input  wire [ADDR_W-1:0] raddr,
    output wire [7:0]        rdata
);
    cyan_simple_dual_port_ram #(
        .ADDR_W(ADDR_W),
        .DATA_W(8),
        .READ_DURING_WRITE("DONT_CARE")
    ) ram (
        .clk(clk),
        .raddr(raddr),
        .rdata(rdata),
        .we(we),
        .waddr(waddr),
        .wdata(wdata)
    );
endmodule

`default_nettype wire
