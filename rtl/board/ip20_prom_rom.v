// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// Dual-read IP20 PROM ROM.
//
// The R4000 fetch path needs a 32-bit instruction word while D-side PROM
// loads need a second 32-bit read port to assemble an aligned 64-bit beat.

`default_nettype none

module ip20_prom_rom_bank #(
    parameter INIT_HEX_FILE = "UNUSED"
) (
    input  wire        clk_a,
    input  wire [12:0] addr_a,
    output wire [31:0] q_a,
    input  wire        clk_b,
    input  wire [12:0] addr_b,
    output wire [31:0] q_b
);
    (* ram_style = "block", ramstyle = "M9K" *) reg [31:0] mem [0:8191];
    reg [31:0] q_a_r;
    reg [31:0] q_b_r;

    assign q_a = q_a_r;
    assign q_b = q_b_r;

    initial begin
        $readmemh(INIT_HEX_FILE, mem);
    end

    always @(posedge clk_a) begin
        q_a_r <= mem[addr_a];
    end

    always @(posedge clk_b) begin
        q_b_r <= mem[addr_b];
    end
endmodule

module ip20_prom_rom #(
    parameter INIT_FILE = "UNSPECIFIED",
    parameter INIT_HEX_B00 = "UNSPECIFIED",
    parameter INIT_HEX_B01 = "UNSPECIFIED",
    parameter INIT_HEX_B02 = "UNSPECIFIED",
    parameter INIT_HEX_B03 = "UNSPECIFIED",
    parameter INIT_HEX_B04 = "UNSPECIFIED",
    parameter INIT_HEX_B05 = "UNSPECIFIED",
    parameter INIT_HEX_B06 = "UNSPECIFIED",
    parameter INIT_HEX_B07 = "UNSPECIFIED",
    parameter INIT_HEX_B08 = "UNSPECIFIED",
    parameter INIT_HEX_B09 = "UNSPECIFIED",
    parameter INIT_HEX_B0A = "UNSPECIFIED",
    parameter INIT_HEX_B0B = "UNSPECIFIED",
    parameter INIT_HEX_B0C = "UNSPECIFIED",
    parameter INIT_HEX_B0D = "UNSPECIFIED",
    parameter INIT_HEX_B0E = "UNSPECIFIED",
    parameter INIT_HEX_B0F = "UNSPECIFIED"
) (
    input  wire        clk_a,
    input  wire [16:0] addr_a,
    output wire [31:0] q_a,
    input  wire        clk_b,
    input  wire [16:0] addr_b,
    output wire [31:0] q_b
);
`ifdef SYNTHESIS
    wire [31:0] qa0, qa1, qa2, qa3, qa4, qa5, qa6, qa7;
    wire [31:0] qa8, qa9, qaa, qab, qac, qad, qae, qaf;
    wire [31:0] qb0, qb1, qb2, qb3, qb4, qb5, qb6, qb7;
    wire [31:0] qb8, qb9, qba, qbb, qbc, qbd, qbe, qbf;
    wire [31:0] q_a_raw;
    wire [31:0] q_b_raw;
    reg  [3:0]  bank_a_q;
    reg  [3:0]  bank_b_q;

    always @(posedge clk_a) begin
        bank_a_q <= addr_a[16:13];
    end

    always @(posedge clk_b) begin
        bank_b_q <= addr_b[16:13];
    end

    assign q_a_raw = (bank_a_q == 4'h0) ? qa0 :
                     (bank_a_q == 4'h1) ? qa1 :
                     (bank_a_q == 4'h2) ? qa2 :
                     (bank_a_q == 4'h3) ? qa3 :
                     (bank_a_q == 4'h4) ? qa4 :
                     (bank_a_q == 4'h5) ? qa5 :
                     (bank_a_q == 4'h6) ? qa6 :
                     (bank_a_q == 4'h7) ? qa7 :
                     (bank_a_q == 4'h8) ? qa8 :
                     (bank_a_q == 4'h9) ? qa9 :
                     (bank_a_q == 4'ha) ? qaa :
                     (bank_a_q == 4'hb) ? qab :
                     (bank_a_q == 4'hc) ? qac :
                     (bank_a_q == 4'hd) ? qad :
                     (bank_a_q == 4'he) ? qae : qaf;
    assign q_b_raw = (bank_b_q == 4'h0) ? qb0 :
                     (bank_b_q == 4'h1) ? qb1 :
                     (bank_b_q == 4'h2) ? qb2 :
                     (bank_b_q == 4'h3) ? qb3 :
                     (bank_b_q == 4'h4) ? qb4 :
                     (bank_b_q == 4'h5) ? qb5 :
                     (bank_b_q == 4'h6) ? qb6 :
                     (bank_b_q == 4'h7) ? qb7 :
                     (bank_b_q == 4'h8) ? qb8 :
                     (bank_b_q == 4'h9) ? qb9 :
                     (bank_b_q == 4'ha) ? qba :
                     (bank_b_q == 4'hb) ? qbb :
                     (bank_b_q == 4'hc) ? qbc :
                     (bank_b_q == 4'hd) ? qbd :
                     (bank_b_q == 4'he) ? qbe : qbf;
    assign q_a = q_a_raw;
    assign q_b = q_b_raw;

    ip20_prom_rom_bank #(.INIT_HEX_FILE(INIT_HEX_B00)) b00 (.clk_a(clk_a), .addr_a(addr_a[12:0]), .q_a(qa0), .clk_b(clk_b), .addr_b(addr_b[12:0]), .q_b(qb0));
    ip20_prom_rom_bank #(.INIT_HEX_FILE(INIT_HEX_B01)) b01 (.clk_a(clk_a), .addr_a(addr_a[12:0]), .q_a(qa1), .clk_b(clk_b), .addr_b(addr_b[12:0]), .q_b(qb1));
    ip20_prom_rom_bank #(.INIT_HEX_FILE(INIT_HEX_B02)) b02 (.clk_a(clk_a), .addr_a(addr_a[12:0]), .q_a(qa2), .clk_b(clk_b), .addr_b(addr_b[12:0]), .q_b(qb2));
    ip20_prom_rom_bank #(.INIT_HEX_FILE(INIT_HEX_B03)) b03 (.clk_a(clk_a), .addr_a(addr_a[12:0]), .q_a(qa3), .clk_b(clk_b), .addr_b(addr_b[12:0]), .q_b(qb3));
    ip20_prom_rom_bank #(.INIT_HEX_FILE(INIT_HEX_B04)) b04 (.clk_a(clk_a), .addr_a(addr_a[12:0]), .q_a(qa4), .clk_b(clk_b), .addr_b(addr_b[12:0]), .q_b(qb4));
    ip20_prom_rom_bank #(.INIT_HEX_FILE(INIT_HEX_B05)) b05 (.clk_a(clk_a), .addr_a(addr_a[12:0]), .q_a(qa5), .clk_b(clk_b), .addr_b(addr_b[12:0]), .q_b(qb5));
    ip20_prom_rom_bank #(.INIT_HEX_FILE(INIT_HEX_B06)) b06 (.clk_a(clk_a), .addr_a(addr_a[12:0]), .q_a(qa6), .clk_b(clk_b), .addr_b(addr_b[12:0]), .q_b(qb6));
    ip20_prom_rom_bank #(.INIT_HEX_FILE(INIT_HEX_B07)) b07 (.clk_a(clk_a), .addr_a(addr_a[12:0]), .q_a(qa7), .clk_b(clk_b), .addr_b(addr_b[12:0]), .q_b(qb7));
    ip20_prom_rom_bank #(.INIT_HEX_FILE(INIT_HEX_B08)) b08 (.clk_a(clk_a), .addr_a(addr_a[12:0]), .q_a(qa8), .clk_b(clk_b), .addr_b(addr_b[12:0]), .q_b(qb8));
    ip20_prom_rom_bank #(.INIT_HEX_FILE(INIT_HEX_B09)) b09 (.clk_a(clk_a), .addr_a(addr_a[12:0]), .q_a(qa9), .clk_b(clk_b), .addr_b(addr_b[12:0]), .q_b(qb9));
    ip20_prom_rom_bank #(.INIT_HEX_FILE(INIT_HEX_B0A)) b0a (.clk_a(clk_a), .addr_a(addr_a[12:0]), .q_a(qaa), .clk_b(clk_b), .addr_b(addr_b[12:0]), .q_b(qba));
    ip20_prom_rom_bank #(.INIT_HEX_FILE(INIT_HEX_B0B)) b0b (.clk_a(clk_a), .addr_a(addr_a[12:0]), .q_a(qab), .clk_b(clk_b), .addr_b(addr_b[12:0]), .q_b(qbb));
    ip20_prom_rom_bank #(.INIT_HEX_FILE(INIT_HEX_B0C)) b0c (.clk_a(clk_a), .addr_a(addr_a[12:0]), .q_a(qac), .clk_b(clk_b), .addr_b(addr_b[12:0]), .q_b(qbc));
    ip20_prom_rom_bank #(.INIT_HEX_FILE(INIT_HEX_B0D)) b0d (.clk_a(clk_a), .addr_a(addr_a[12:0]), .q_a(qad), .clk_b(clk_b), .addr_b(addr_b[12:0]), .q_b(qbd));
    ip20_prom_rom_bank #(.INIT_HEX_FILE(INIT_HEX_B0E)) b0e (.clk_a(clk_a), .addr_a(addr_a[12:0]), .q_a(qae), .clk_b(clk_b), .addr_b(addr_b[12:0]), .q_b(qbe));
    ip20_prom_rom_bank #(.INIT_HEX_FILE(INIT_HEX_B0F)) b0f (.clk_a(clk_a), .addr_a(addr_a[12:0]), .q_a(qaf), .clk_b(clk_b), .addr_b(addr_b[12:0]), .q_b(qbf));
`else
    reg [31:0] mem [0:131071];
    reg [31:0] q_a_r;
    reg [31:0] q_b_r;

    assign q_a = q_a_r;
    assign q_b = q_b_r;

    initial begin
        $readmemh(INIT_FILE, mem);
    end

    always @(posedge clk_a) begin
        q_a_r <= mem[addr_a];
    end

    always @(posedge clk_b) begin
        q_b_r <= mem[addr_b];
    end
`endif
endmodule

`default_nettype wire
