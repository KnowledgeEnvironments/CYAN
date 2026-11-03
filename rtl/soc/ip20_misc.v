// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - IP20 miscellaneous board registers
//
// Small CPU-board register block for IP20 addresses that are not part of MC,
// HPC1 PBUS, INT2, or LG1, but are real hardware and must not bus-float.

`default_nettype none

module ip20_misc (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [31:0] addr,
    input  wire [63:0] wdata,
    output reg  [63:0] rdata,
    input  wire [ 7:0] wmask,
    input  wire        valid,
    input  wire        write,
    output reg         ready
);

    // IP20.h:
    //   BOARD_REV_ADDR = 0x1fbd0000
    //   board_rev() = (~reg & 0x7000) >> 12
    //   is_v50() also checks REV_HP1 through bit 15.
    //
    // Return CPU board revision 2, normal/non-V50:
    //   bits [14:12] = ~2 = 3'b101
    //   bit  [15]    = 1 so REV_HP1 is not asserted through inversion
    localparam [31:0] BOARD_REV_VALUE = 32'h0000_D000;

    wire board_rev_sel = (addr[31:3] == 29'h03F7_A000);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ready <= 1'b0;
            rdata <= 64'h0;
        end else if (!valid) begin
            ready <= 1'b0;
        end else if (!ready) begin
            ready <= 1'b1;
            if (board_rev_sel)
                rdata <= {BOARD_REV_VALUE, BOARD_REV_VALUE};
            else
                rdata <= 64'h0;
        end
    end

    // Writes are ignored; these board-ID straps are read-only.
    wire unused = write ^ ^wdata ^ ^wmask;

endmodule

`default_nettype wire
