// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

//////////////////////////////////////////////////////////////////////////////
// DSP56001 External SRAM - 32K x 24-bit Dual-Port
//
// Dual-port SRAM shared between CPU (system bus at 0x1FBE0000) and DSP.
// CPU port: R/W from system bus via HPC1.
// DSP port: R/W for external P/X/Y memory accesses.
//
// Y-space address mapping: XOR bit 14 to convert P->Y address:
//   sram_addr = (space==Y) ? {addr[14]^1, addr[13:0]} : addr[14:0]
//////////////////////////////////////////////////////////////////////////////

module dsp56001_sram (
    input  wire        clk,
    input  wire        rst_n,

    // ========================================================================
    // CPU Port (system bus side, active during CPU SRAM access)
    // ========================================================================
    input  wire [14:0] cpu_addr,          // 32K word address
    input  wire [23:0] cpu_wdata,         // Write data (24-bit)
    output reg  [23:0] cpu_rdata,         // Read data (24-bit)
    input  wire        cpu_wr_en,         // Write enable
    input  wire        cpu_rd_en,         // Read enable
    output wire        cpu_ready,         // Access complete

    // ========================================================================
    // DSP Port (core side, active during external memory access)
    // ========================================================================
    input  wire [14:0] dsp_addr,          // 32K word address
    input  wire [23:0] dsp_wdata,         // Write data (24-bit)
    output reg  [23:0] dsp_rdata,         // Read data (24-bit)
    input  wire        dsp_wr_en,         // Write enable
    input  wire        dsp_rd_en,         // Read enable
    output wire        dsp_ready          // Access complete
);

    // ========================================================================
    // SRAM Storage (32K x 24-bit)
    // Inferred as block RAM by synthesis tools
    // ========================================================================
    reg [23:0] mem [0:32767];

    // Initialize the communication area deterministically.  Word zero is the
    // "DSP started" value probed by boot firmware before runtime loading;
    // dsploader subsequently replaces the external program/data contents.
    integer i;
    initial begin
        for (i = 0; i < 32768; i = i + 1)
            mem[i] = 24'h0;
        mem[0] = 24'h000001;
        cpu_rdata = 24'h0;
        dsp_rdata = 24'h0;
    end

    // ========================================================================
    // CPU Port (Port A) - synchronous read/write
    // ========================================================================
    reg cpu_ready_r;
    wire cpu_req = cpu_wr_en | cpu_rd_en;
    wire cpu_accept = cpu_req && !cpu_ready_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cpu_ready_r <= 1'b0;
        else
            cpu_ready_r <= cpu_accept;
    end

    always @(posedge clk) begin
        if (cpu_accept && cpu_wr_en)
            mem[cpu_addr] <= cpu_wdata;
        if (cpu_accept && cpu_rd_en)
            cpu_rdata <= mem[cpu_addr];
    end

    // ========================================================================
    // DSP Port (Port B) - synchronous read/write
    // ========================================================================
    // CPU port: immediate ready (C++ controls timing)
    assign cpu_ready = cpu_ready_r;

    // DSP port.  Vivado requires each write port of an inferred true-dual-port
    // BRAM in its own clocked process, even though both ports currently share
    // the SoC clock.
    always @(posedge clk) begin
        if (dsp_wr_en)
            mem[dsp_addr] <= dsp_wdata;
        if (dsp_rd_en)
            dsp_rdata <= mem[dsp_addr];
    end

    // The request is accepted without a bus wait.  Read data is still the
    // registered BRAM value above and is consumed by the core's following
    // memory/writeback state.  `ready` therefore describes acceptance of the
    // current request, not combinational availability of read data.
    assign dsp_ready = dsp_wr_en | dsp_rd_en;

endmodule
