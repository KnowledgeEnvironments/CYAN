// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - vendor-aware simple dual-port synchronous RAM.
//
// One write port, one registered-read port, common clock. The Quartus path
// keeps explicit altsyncram control; the Xilinx/Verilator paths use behavioral
// inference with old-data read ordering on same-cycle mixed-port collisions.

`default_nettype none

module cyan_simple_dual_port_ram #(
    parameter integer ADDR_W = 11,
    parameter integer DATA_W = 8,
    parameter READ_DURING_WRITE = "DONT_CARE"
) (
    input  wire              clk,
    input  wire [ADDR_W-1:0] raddr,
    output wire [DATA_W-1:0] rdata,
    input  wire              we,
    input  wire [ADDR_W-1:0] waddr,
    input  wire [DATA_W-1:0] wdata
);
    localparam integer DEPTH = (1 << ADDR_W);

`ifdef VERILATOR
    reg [DATA_W-1:0] mem [0:DEPTH-1];
    reg [DATA_W-1:0] rdata_q;

    assign rdata = rdata_q;

    always @(posedge clk) begin
        rdata_q <= mem[raddr];
        if (we)
            mem[waddr] <= wdata;
    end
`elsif FPGA_XILINX
    (* ram_style = "block" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
    reg [DATA_W-1:0] rdata_q;

    assign rdata = rdata_q;

    always @(posedge clk) begin
        rdata_q <= mem[raddr];
        if (we)
            mem[waddr] <= wdata;
    end
`else
    altsyncram ram (
        .clock0    (clk),
        .address_a (waddr),
        .data_a    (wdata),
        .wren_a    (we),
        .address_b (raddr),
        .q_b       (rdata)
    );
    defparam ram.operation_mode = "DUAL_PORT";
    defparam ram.width_a = DATA_W;
    defparam ram.widthad_a = ADDR_W;
    defparam ram.numwords_a = DEPTH;
    defparam ram.width_b = DATA_W;
    defparam ram.widthad_b = ADDR_W;
    defparam ram.numwords_b = DEPTH;
    defparam ram.address_aclr_a = "NONE";
    defparam ram.address_aclr_b = "NONE";
    defparam ram.address_reg_b = "CLOCK0";
    defparam ram.outdata_reg_b = "UNREGISTERED";
    defparam ram.outdata_aclr_b = "NONE";
    defparam ram.indata_aclr_a = "NONE";
    defparam ram.wrcontrol_aclr_a = "NONE";
    defparam ram.read_during_write_mode_mixed_ports = READ_DURING_WRITE;
`ifdef FPGA_CYCLONE_IV_GX
    defparam ram.intended_device_family = "Cyclone IV GX";
    defparam ram.ram_block_type = "M9K";
`else
    defparam ram.intended_device_family = "Cyclone IV E";
`endif
    defparam ram.lpm_type = "altsyncram";
`endif
endmodule

`default_nettype wire
